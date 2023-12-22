use std::fmt::{Debug, Display};




use std::sync::atomic::{AtomicU32, Ordering};


static BRANCH_ID_COUNTER: AtomicU32 = AtomicU32::new(0);



#[derive(Clone)]
pub struct BranchComparator<BranchingFilter : IBranchFilter,DualStorage : UserDualStore>  {
    has_bound: bool,
    _bf : PhantomData<BranchingFilter>,
    _ds : PhantomData<DualStorage>
}

impl<BranchingFilter : IBranchFilter,DualStorage : UserDualStore> BranchComparator<BranchingFilter, DualStorage>  {

    pub fn with_bound() -> Self {
        Self {
            has_bound : true,
            _bf : PhantomData,
            _ds : PhantomData
        }
    }

    pub fn without_bound() -> Self {
        Self {
            has_bound : false,
            _bf : PhantomData,
            _ds : PhantomData
        }
    }

    // higher score -> earlier in queue
    fn score_branch(b: &Branch<BranchingFilter, DualStorage>) -> u32 {
        if b.special.is_some() {
            return u32::MAX;
        }

        10000 + b.depth
    }
}

impl<BranchingFilter : IBranchFilter,DualStorage : UserDualStore> Compare<Branch<BranchingFilter, DualStorage>> for BranchComparator<BranchingFilter, DualStorage>  {
    fn compare(
        &self,
        l: &Branch<BranchingFilter, DualStorage>,
        r: &Branch<BranchingFilter, DualStorage>,
    ) -> core::cmp::Ordering {
        if let (
            Some(SpecialType::LDSBranch {
                     taboo_filters: ref taboo_filters_l,
                 }),
            Some(SpecialType::LDSBranch {
                     taboo_filters: ref taboo_filters_r,
                 }),
        ) = (&l.special, &r.special)
        {
            // in lds branches, go for discrepancy then depth

            // prefer fewest filters (least backtracking), then max depth
            taboo_filters_r
                .len()
                .cmp(&taboo_filters_l.len())
                .then(l.depth.cmp(&r.depth))
        } else if self.has_bound {
            r.old_obj_bound.total_cmp(&l.old_obj_bound)
        } else {
            BranchComparator::score_branch(l).cmp(&BranchComparator::score_branch(r))
        }
    }
}


#[derive(Debug, Clone)]
/// Indicates a special type of branch
pub enum SpecialType<BranchFilter : IBranchFilter> {
    Root,
    Integer,
    LDSBranch { taboo_filters : HashSet<BranchFilter> }
}
use core::fmt::Formatter;
use std::marker::PhantomData;
use compare::Compare;

impl<BranchFilter : IBranchFilter> Display for SpecialType<BranchFilter > {

    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            SpecialType::Root =>  write!(f,"Root"),
            SpecialType::Integer =>  write!(f,"Integer"),
            LDSBranch { taboo_filters} => {
                write!(f, "LDS(").unwrap();
                taboo_filters.iter().for_each(|filter| write!(f, "{},", filter).unwrap());
                write!(f, ")")
            }
        }

    }


}

#[derive(Debug, Clone)]
/// Primary struct for branch in branching tree
/// Holds all information about its state
pub struct Branch<BranchFilter, DualStore>
where
    BranchFilter: IBranchFilter,
        DualStore : UserDualStore
{
    pub id: u32,
    pub parent: u32,
    pub depth: u32,
    pub old_obj_bound: f64, // objective value before adding filtr
    pub old_obj_heuristic: bool,
    pub old_obj_dual_center: Option<(f64,DualStore)>,
    pub filters: Vec<BranchFilter>,
    pub allow_heuristic_pricing : bool,
    pub special: Option<SpecialType<BranchFilter>>,
}

use crate::{IBranchFilter, UserDualStore};
use crate::branch::SpecialType::LDSBranch;
use crate::misc::HashSet;



impl<BranchFilter, DualStore> Branch<BranchFilter, DualStore>
where
    BranchFilter: IBranchFilter,
    DualStore : UserDualStore

{

    /// Returns the initial root node, initialized with default values
    pub fn default() -> Self {
        let id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        Branch {
            id,
            depth: 0,
            parent: id,
            old_obj_bound: f64::NEG_INFINITY,
            old_obj_heuristic: false,
            old_obj_dual_center : None,
            allow_heuristic_pricing : true,
            filters: Vec::default(),
            special: Some(SpecialType::Root),
        }
    }

    /// Returns a root node for limited discrepancy search
    pub fn create_lds_root() -> Self {
        let id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        Branch {
            id,
            depth: 0,
            parent: id,
            allow_heuristic_pricing : true,
            old_obj_bound: f64::NEG_INFINITY,
            old_obj_heuristic: false,
            old_obj_dual_center : None,
            filters: Vec::default(),
            special: Some(SpecialType::LDSBranch {
                taboo_filters : HashSet::default(),
            }),
        }
    }

    /// transforms the current node into a copy that
    /// will be solved as integer linear program.
    /// Can be used to get better primal bounds
    ///
    /// `local` setting influences whether existing branches should apply
    ///         or (local=false) all branching constraints are removed
    pub fn create_integer(&self, local : bool) -> Self {
        let new_id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);

        Branch {
            id: new_id,
            parent: self.id,
            depth: self.depth,
            allow_heuristic_pricing : false,
            old_obj_bound: self.old_obj_bound,
            old_obj_heuristic: self.old_obj_heuristic,
            old_obj_dual_center : self.old_obj_dual_center.clone(),
            filters: if local { self.filters.clone()} else { vec![] },
            special: Some(SpecialType::Integer),
        }
    }

    /// Takes a heuristic pricing branching node
    /// and returns a normal branching node
    pub fn upgrade_to_non_heuristic_pricing(&self) -> Self {
        let new_id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);

        Branch {
            id: new_id,
            parent: self.id,
            depth: self.depth,
            allow_heuristic_pricing : false,
            old_obj_bound: self.old_obj_bound,
            old_obj_heuristic: self.old_obj_heuristic,
            old_obj_dual_center : self.old_obj_dual_center.clone(),
            filters: self.filters.clone(),
            special: self.special.clone(),
        }
    }

    /// Given a current lds branching node,
    /// create the zero branch that expands the taboo list
    pub fn create_lds_zero_child(&self, parent_obj: f64, filters_of_one_child: &[BranchFilter], parent_partial : bool, parent_dual_center : Option<(f64, DualStore)>) -> Self {


        // get current taboo list
        let mut taboo_list = if let Some(SpecialType::LDSBranch{ taboo_filters}) = &self.special { taboo_filters.clone()} else { unreachable!()};

        for filter in filters_of_one_child {
            taboo_list.insert(filter.clone());
        }



        let new_id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        Branch {
            id: new_id,
            parent: self.id,
            depth: self.depth,
            allow_heuristic_pricing : self.allow_heuristic_pricing,
            old_obj_bound: parent_obj,
            old_obj_heuristic: parent_partial,
            old_obj_dual_center : parent_dual_center,
            filters: self.filters.clone(),
            special : Some(LDSBranch { taboo_filters : taboo_list})
        }
    }

    /// Given a current lds branching node,
    /// create the branch forcing a column
    pub fn create_lds_one_child(&self, parent_obj: f64, filter: Vec<BranchFilter>, parent_partial : bool, parent_dual_center : Option<(f64, DualStore)>) -> Self {

        let mut new_filters  = self.filters.clone();
        new_filters.extend(filter);


        let new_id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        Branch {
            id: new_id,
            parent: self.id,
            depth: self.depth + 1,
            old_obj_bound: parent_obj,
            allow_heuristic_pricing : self.allow_heuristic_pricing,
            old_obj_heuristic: parent_partial,
            old_obj_dual_center : parent_dual_center,
            filters: new_filters,
            special : self.special.clone()
        }

    }

    /// Given a branching node, create a child node with
    /// additional branching filter applied
    pub fn create_child(&self, parent_obj: f64, filter: Vec<BranchFilter>, parent_partial : bool, parent_dual_center : Option<(f64,DualStore)>) -> Self {


        let mut new_filters = self.filters.clone();
        new_filters.extend(filter);

        let new_id = BRANCH_ID_COUNTER.fetch_add(1, Ordering::SeqCst);

        Branch {
            id: new_id,
            parent: self.id,
            depth: self.depth + 1,
            allow_heuristic_pricing : self.allow_heuristic_pricing,
            old_obj_bound: parent_obj,
            old_obj_heuristic: parent_partial,
            old_obj_dual_center : parent_dual_center,
            filters: new_filters,
            special : None
        }
    }
}

#[derive(Debug, Clone)]
/// A branch group contains a list of branching options
/// Collectively, they must not cut of an optimal solution.
/// Individually, they must cut of the current solution.
pub struct BranchGroup<BranchFilter, BranchGroupType>
where
    BranchGroupType: Debug,
{
    pub filters: Vec<Vec<BranchFilter>>,
    pub branch_group_type: BranchGroupType,
}

impl<BranchFilter, BranchGroupType> BranchGroup<BranchFilter, BranchGroupType>
where
    BranchGroupType: Debug,
{
    pub fn new(filters : Vec<Vec<BranchFilter>>, branch_group_type: BranchGroupType) -> Self {
        Self {
           filters,
            branch_group_type,
        }
    }
}
