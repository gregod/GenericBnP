use std::ffi::c_schar;
use core::hash::Hash;
use std::cell::Cell;
use std::fmt::{Debug, Display};
use std::marker::PhantomData;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};
use crate::generic_master_problem::branch::{Branch, BranchGroup, SpecialType};
use crate::generic_master_problem::column_pool::{ColumnId, ColumnPool};

use crate::misc::blocking_queue::{BlockingQueue};

use crate::column_pool::{Column, ColumnPoolFilter};

pub mod branch;
pub mod column_pool;

/// Factory used for LPModels
pub trait LPEnv: Sized {
    fn new_with_seed(seed: i32) -> Self;
    fn set_time_limit(&mut self, seconds: f64);
}

/// Trait for a linear programming model
/// Panics on errors
pub trait LPModel<Constr, Var, Env: LPEnv>: Sized {
    // Use factory LPEnv to create new model
    fn new(name: &str, env: &mut Env) -> Self;
    // Return dual values for list of constraints
    fn get_dual_list(&self, constrs: &[Constr]) -> Vec<f64>;
    // Return solution coefficents for the list of variables
    fn get_x_list(&self, vars: &[Var]) -> Vec<f64>;

    // Return optimal objective function
    fn get_objective(&self) -> f64;

    // Return runtime of optimization in seconds
    fn get_runtime(&self) -> f64;

    // Return number of optimal solutions found
    fn get_num_solutions(&self) -> i32;

    // Run optimization.
    fn optimize(&mut self);
}


/// Describes the user branching filter
/// This should be implemented as an enum describing all potential branching operations
pub trait IBranchFilter: Debug + Clone + PartialEq + Display + Eq + Hash {

    /// Optional strengthening function that can translate
    /// between branch filter types
    /// Example could be an EdgeFlow(Edge, 0) => Forbid(Edge)
    /// Where the latter can be applied in the pricing problem
    fn strengthen(self) -> Self { return self }
}

/// Integer tolerance. Same as gurobi
pub const INT_FEAS_TOL: f64 = 1e-5;

/// Primary struct holding all problem information
pub struct MasterProblem<
    ProblemInstance : Clone,
    Solver: UserPricingSolver<ProblemInstance, BranchFilter, ColumnType, DualStore>,
    ModelMeta: UserModelMeta<Constr, Var, Env, Model>,
    ColumnType: PartialEq + 'static,
    DualStore: UserDualStore,
    BranchFilter: IBranchFilter,
    BranchGroupType: std::clone::Clone + Debug,
    UMasterProblem: UserMasterProblem<ProblemInstance, Solver, ColumnType, DualStore, ModelMeta, BranchFilter, BranchGroupType, Constr, Env, Model, Var>,
    C: Compare<Branch<BranchFilter, DualStore>> + Clone,
    Constr: Clone, Env: LPEnv, Model: LPModel<Constr, Var, Env>, Var: Clone
>
    where ColumnPool<ColumnType, BranchFilter>: ColumnPoolFilter<ColumnType, BranchFilter>
{
    pub(crate) user_problem: UMasterProblem,
    pricing_solver: Solver,
    pub(crate) shared: Arc<SharedState<ColumnType, BranchFilter, C, DualStore>>,
    _bpt: PhantomData<BranchGroupType>,
    _mm: PhantomData<ModelMeta>,
    _ds: PhantomData<DualStore>,
    _cs: PhantomData<Constr>,
    _ev: PhantomData<Env>,
    _md: PhantomData<Model>,
    _vr: PhantomData<Var>,
    _pi : PhantomData<ProblemInstance>
}


impl<
    ProblemInstance : Clone,
    Solver: UserPricingSolver<ProblemInstance, BranchFilter, ColumnType, DualStore>,
    ModelMeta: UserModelMeta<Constr, Var, Env, Model>,
    ColumnType: PartialEq,
    DualStore: UserDualStore,
    BranchFilter: std::clone::Clone,
    BranchGroupType: std::clone::Clone + Debug,
    UMasterProblem: UserMasterProblem<
        ProblemInstance,
        Solver,
        ColumnType,
        DualStore,
        ModelMeta,
        BranchFilter,
        BranchGroupType,
        Constr, Env, Model, Var
    >,
    C: Compare<Branch<BranchFilter, DualStore>> + Clone,
    Constr: Clone, Env: LPEnv, Model: LPModel<Constr, Var, Env>, Var: Clone
> MasterProblem<ProblemInstance,Solver, ModelMeta, ColumnType, DualStore, BranchFilter, BranchGroupType, UMasterProblem, C, Constr, Env, Model, Var>
    where
        BranchGroupType: Clone + Debug,
        BranchFilter: IBranchFilter,
        ColumnPool<ColumnType, BranchFilter>: ColumnPoolFilter<ColumnType, BranchFilter>
{
    pub fn new(
        shared: Arc<SharedState<ColumnType, BranchFilter, C, DualStore>>,
        master_problem: UMasterProblem,
        pricing_solver: Solver,
    ) -> Self {
        MasterProblem {
            _bpt: PhantomData::default(),
            pricing_solver,
            shared,
            user_problem: master_problem,
            _mm: PhantomData::default(),
            _ds: PhantomData::default(),

            _cs: PhantomData::default(),
            _ev: PhantomData::default(),
            _md: PhantomData::default(),
            _vr: PhantomData::default(),
            _pi : PhantomData
        }
    }


    /// Launch the worker's loop
    ///
    /// Worker will pop open branches and process them
    pub fn run_worker(&mut self, real_bound_from: f64, consider_infeasible_over: f64, seed: i32) {
        let _inst = Instant::now();
        // setup main brancher
        let mut env = Env::new_with_seed(seed);
        // go


        let mut processed_branches = 0;

        self.shared.ui_sender.send(UIUserMessage::StartWorker);

        #[cfg(feature = "validity_assertions")]
        self.shared.ui_sender.send(UIUserMessage::LogS("Validity Assertions Active"));

        while let Some(branch) = self.shared.open_branches.get_job() {
            let start_time_branch = Instant::now();

            if let Some(time_limit) = self.shared.time_limit {
                if start_time_branch > time_limit {
                    self.shared.ui_sender.send(UIUserMessage::TimeLimitReached);
                    self.shared.open_branches.flush_and_terminate();
                    break;
                }
            }

            let curr_integer_best = {
                let lock = self.shared.best.read().unwrap();
                let val = lock.0;
                drop(lock);
                val
            };

            // test if we can still be better than the best found solution
            if self.user_problem.is_a_cutoff_by_bound_b(&branch.old_obj_bound, &curr_integer_best)
                || branch.old_obj_bound > consider_infeasible_over
                || (matches!(branch.special, Some(SpecialType::LDSBranch{..})) && curr_integer_best <= self.user_problem.get_lds_settings().early_termination_after_bound_leq as f64)
            {
                let some_root_node = match branch.special {
                    Some(SpecialType::LDSBranch { ref taboo_filters }) => {
                        // root lds branch has discrepancy + depth == 0
                        taboo_filters.len() == 0 && branch.depth == 0
                    }
                    Some(SpecialType::Root) => true,
                    _ => false
                };

                // if the obj wasn't heuristic, then we can skip job
                if !branch.old_obj_heuristic && !some_root_node {
                    self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.parent });
                    self.shared.open_branches.job_done();
                    continue;
                }
            }

            self.shared.ui_sender.send(UIUserMessage::BranchStart(BranchUIState {
                branch_id: branch.id,
                branch_parent: branch.parent,
                num_columns: {
                    let cp_lock = self.shared.column_pool.read().unwrap();
                    let val = cp_lock.count();
                    drop(cp_lock);
                    val
                },
                current_open: self.shared.open_branches.len(),
                num_filters: branch.filters.len(),
                last_filter: {
                    if let Some(filter) = branch.filters.last() {
                        format!("{}", filter)
                    } else {
                        "-".to_string()
                    }
                },
                special: branch.special.clone().map(|s| format!("{}", s)).unwrap_or("-".to_string()),
                before_obj: branch.old_obj_bound,
                after_obj: None,
                best_obj: curr_integer_best,
            }));


            let ColGenResult {
                obj,
                column_x: patterns,
                time_limit_reached,
                infeasible,
                early_branched: was_heuristic_pricing,
                smoothing_center: dual_store
            } = match branch.special {
                Some(SpecialType::Integer) => self.user_problem.run_column_generation(
                    &mut env,
                    &branch,
                    &self.shared.best,
                    branch.old_obj_bound,
                    &self.shared.column_pool,
                    &mut self.pricing_solver,
                    self.shared.time_limit,
                    true,
                    branch.old_obj_dual_center.clone(),
                ),

                Some(SpecialType::Root) | Some(SpecialType::LDSBranch { .. }) /* within col gen no difference for lds */ | None => self.user_problem.run_column_generation(
                    &mut env,
                    &branch,
                    &self.shared.best,
                    branch.old_obj_bound,
                    &self.shared.column_pool,
                    &mut self.pricing_solver,
                    self.shared.time_limit,
                    false,
                    branch.old_obj_dual_center.clone(),
                ),
            };

            // record root node timings
            match branch.special {
                Some(SpecialType::Root) => {
                    *self.shared.obj_root_node.lock().unwrap() = Some(obj);
                    *self.shared.duration_root_node.lock().unwrap() = Some(start_time_branch.elapsed())
                }
                Some(SpecialType::LDSBranch { ref taboo_filters }) if taboo_filters.len() == 0 && branch.depth == 0 => {
                    *self.shared.obj_root_node.lock().unwrap() = Some(obj);
                    *self.shared.duration_lds_root_node.lock().unwrap() = Some(start_time_branch.elapsed())
                }
                None | Some(_) => {}
            };

            processed_branches += 1;

            let cp_lock = self.shared.column_pool.read().unwrap();
            let best_lock = self.shared.best.read().unwrap();

            self.shared.ui_sender.send(UIUserMessage::BranchFinish(BranchUIState {
                branch_id: branch.id,
                branch_parent: branch.parent,
                num_columns: cp_lock.count(),
                current_open: self.shared.open_branches.len(),
                num_filters: branch.filters.len(),
                last_filter: {
                    if let Some(filter) = branch.filters.last() {
                        format!("{}", filter)
                    } else {
                        "-".to_string()
                    }
                },
                special: branch.special.clone().map(|s| format!("{}", s)).unwrap_or("-".to_string()),
                before_obj: branch.old_obj_bound,
                after_obj: Some(obj),
                best_obj: best_lock.0,
            }));
            drop(best_lock);
            drop(cp_lock);

            let lock = self.shared.best.read().unwrap();
            let best = lock.0;
            drop(lock);


            if !obj.is_finite() && !infeasible /* infeasible */ && was_heuristic_pricing {
                // if we are infeasible but partial, we must solve the full node
                self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
            } else {


                // now match the special type on how to proceed
                // if we are a primal heuristic branch, then process primal heuristic type
                // otherwise process normally
                match branch.special {
                    Some(SpecialType::LDSBranch { ref taboo_filters }) => {
                        #[cfg(feature = "validity_assertions")]
                        assert!(matches!(*self.shared.phase.read().unwrap(), Phase::PrimalHeuristic));


                        if obj.is_finite() /*feasible*/ && !infeasible && (!self.user_problem.is_a_cutoff_by_bound_b(&obj, &best) /* better than best */ || /* or still partial */ was_heuristic_pricing) {
                            let chosen_patterns: Vec<&(f64, ColumnId)> = patterns
                                .iter()
                                .filter(|(v, _)| *v > INT_FEAS_TOL)
                                .collect();

                            let all_integer = chosen_patterns
                                .iter().all(|(val, _)| val.fract() < INT_FEAS_TOL);


                            if all_integer {
                                //  branch is done
                                if obj < best {
                                    // update best
                                    {
                                        let mut best = self.shared.best.write().unwrap();
                                        *best = (obj, patterns.clone());
                                    }


                                    self.shared.ui_sender.send(UIUserMessage::NewBest { obj, branch_id: branch.id })
                                }

                                if was_heuristic_pricing {
                                    self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                                }
                            } else {
                                // we need to fix columns !

                                let cp_lock = self.shared.column_pool.read().unwrap();
                                let columns_to_fix = self.user_problem.select_fractional_columns_to_fix_in_lds(
                                    &mut self.pricing_solver,
                                    &branch.filters,
                                    &*cp_lock,
                                    &patterns,
                                    taboo_filters,
                                );
                                drop(cp_lock);

                                if !columns_to_fix.is_empty() {
                                    if branch.depth < self.user_problem.get_lds_settings().max_depth {
                                        if taboo_filters.len() < self.user_problem.get_lds_settings().max_discrepancy as usize {
                                            self.shared.open_branches.add_job(branch.create_lds_zero_child(obj, &columns_to_fix,
                                                                                                           was_heuristic_pricing, dual_store.clone())
                                            );
                                        }


                                        self.shared.open_branches.add_job(branch.create_lds_one_child(obj, columns_to_fix,
                                                                                                      was_heuristic_pricing, dual_store.clone())
                                        );


                                        if let Some(new_columns_required) = self.user_problem.get_periodic_integer_solve_interval() /* perioid integer */ {
                                            let cp_lock = self.shared.column_pool.read().unwrap();
                                            let cp_size = cp_lock.count();
                                            if branch.id > 0
                                                && cp_size
                                                > self.shared.last_integer_at_cp_size.load(Ordering::Relaxed) + new_columns_required
                                            {
                                                self.shared.open_branches.add_job(branch.create_integer(true));
                                                self.shared
                                                    .last_integer_at_cp_size
                                                    .store(cp_size, Ordering::SeqCst);
                                            }
                                            drop(cp_lock);
                                        }
                                    } else {
                                        if was_heuristic_pricing {
                                            self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                                        }
                                        self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.id })
                                    }
                                } else {
                                    if was_heuristic_pricing {
                                        self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                                    }
                                    self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.id })
                                }
                            }
                        } else {
                            if was_heuristic_pricing {
                                self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                            }
                            self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.id })
                        }
                    }

                    _ => {


                        // test if branch should be cut.
                        if obj.is_finite() /*feasible*/ && !infeasible && (!self.user_problem.is_a_cutoff_by_bound_b(&obj, &best) /* better than best */ || /* or still partial */ was_heuristic_pricing) {
                            let cp_lock = self.shared.column_pool.read().unwrap();


                            let fractional_options = self.user_problem.find_fractional_solutions(
                                &mut self.pricing_solver,
                                &branch.filters,
                                &*cp_lock,
                                &patterns,
                            );

                            drop(cp_lock);


                            if let Some(branch_group) = self.user_problem.choose_fractional_solution(
                                obj,
                                fractional_options,
                                best,
                                &branch,
                                &mut self.pricing_solver,
                                &mut env,
                                &self.shared.column_pool,
                                &patterns,
                            ) {
                                // println!("Chose this one {:?}", branch_pair.branch_pair_type);

                                for filter in branch_group.filters {
                                    self.shared.open_branches.add_job(branch.create_child(obj, filter, was_heuristic_pricing, dual_store.clone()));
                                }

                                if let Some(new_columns_required) = self.user_problem.get_periodic_integer_solve_interval() /* perioid integer */ {
                                    let cp_lock = self.shared.column_pool.read().unwrap();
                                    let cp_size = cp_lock.count();
                                    if branch.id > 0
                                        && cp_size
                                        > self.shared.last_integer_at_cp_size.load(Ordering::Relaxed) + new_columns_required
                                    {
                                        self.shared.open_branches.add_job(branch.create_integer(true));
                                        self.shared
                                            .last_integer_at_cp_size
                                            .store(cp_size, Ordering::SeqCst);
                                    }
                                    drop(cp_lock);
                                }
                            } else {
                                // has found non-fractional
                                // must test if not infeasible by dummy columns
                                if !patterns.is_empty() {
                                    {
                                        let mut best = self.shared.best.write().unwrap();

                                        if true /* swap cmp after bound */ {
                                            // only swap if we have a real solution (no dummy costs)!
                                            if obj <= real_bound_from {
                                                self.shared.ui_sender.send(UIUserMessage::LogS("Swapping branch comparator"));
                                                self.shared.open_branches.now_has_bound();
                                            }
                                        }
                                        *best = (obj, patterns.clone())
                                    }

                                    self.shared.ui_sender.send(UIUserMessage::NewBest { obj, branch_id: branch.id })
                                }
                                if was_heuristic_pricing {
                                    self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                                }
                                self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.id })
                            }
                        } else {
                            if was_heuristic_pricing {
                                self.shared.open_branches.add_job(branch.upgrade_to_non_heuristic_pricing());
                            }
                            self.shared.ui_sender.send(UIUserMessage::BranchCut { branch_id: branch.id })
                        }
                    }
                }
            }


            self.shared.open_branches.job_done();
        }
    }
}


/// Trait to be implemented on ModelMeta data.
pub trait UserModelMeta<Constr: Clone, Var: Clone, Env: LPEnv, Model: LPModel<Constr, Var, Env>> {

    /// Return the column selection decision variables
    fn get_pattern_vars(&mut self) -> &mut HashMap<ColumnId, Var>;

    /// Callback that is called just prior to optimizing the LP model
    /// Can be used to make custom modifications
    fn callback_before_optimize(&mut self, lower_bound: f64, best_known: &RwLock<(f64, Vec<(f64, ColumnId)>)>, model: &mut Model) {
        // nop default
    }
}

/// Primary trait to implement the pricing problem
pub trait UserPricingSolver<ProlblemInstance, BranchFilter: IBranchFilter, ColumnType, DualStorage: UserDualStore> {

    fn new(problem_instance : &ProlblemInstance, ui : UISender) -> Self;

    /// Given a list of active branching filters and a dual storage, find a list of negative reduced cost columns
    fn solve(&mut self, active_filters: &[BranchFilter], duals: &DualStorage) -> Vec<(f64, ColumnType)>;

    /// For a given column, report exclusive resource usage.
    /// This is used to find non conflicting columns
    fn unique_conflicting_resources_of_column(&self, column: &ColumnType) -> Option<HashMap<usize, bool>> { None }

    /// Return only the primal cost of the column
    fn primal_cost_column(&self, column: &ColumnType) -> f64;

    /// Return the price of the column given the dual information provided
    ///
    /// Used in the dual smoothing to identify missprices
    fn price_column_with_duals(&self, column: &ColumnType, duals: &DualStorage) -> f64;

    /// Return a bound on the total number of potentially active columns
    /// Works best when a single column must be chosen
    fn bound_num_active_columns(&self, best_int_obj: f64, current_best_used_columns: usize) -> usize;
}

use std::borrow::Cow;

/// Trait to implement for the dual storage
pub trait UserDualStore: Clone {
    /// Perform the linear combination following Wentges rule
    /// Return the combined result as Cow.
    /// If alpha <= 0, return borrowed cow
    fn linear_combination<'a, 'b>(&'b self, alpha: f64, out_duals: &'a Self) -> Cow<'a, Self>;
}

#[derive(Clone)]
/// Settings for the limited discrepancy search
pub struct LDSSettings {
    pub columns_to_fix_each_iteration: usize,
    pub selection: LDSSelection,
    pub max_depth: u32,
    pub max_discrepancy: i32,
    pub early_termination_after_bound_leq: u32,
    pub do_only_lds: bool,
}

#[derive(Clone)]
/// Selection mode for columns in limted discrepancy search
/// Two options: Either select column that is closest to one
///              Or, select least fractional
/// If closest to one, we always force torwards one
/// If closest to int, we always branch torwards closests
pub enum LDSSelection {
    ClosestToOne,
    ClosestToInt,
}

#[derive(Clone)]
/// Settings for dual stabilization following wentgest rule
pub struct StabilizationSettings {
    pub cross_iteration_memory: bool,
    pub initial_smoothing_alpha: f64,
    pub maximum_smoothing_alpha: f64,

    pub alpha_steps_up: f64,
    pub alpha_steps_down: f64,
}


#[derive(Clone)]
pub struct GeneralSettings {
    pub non_overlapping_column_selection: bool,
    pub num_columns_to_choose: usize,
    pub early_branching_in_lds: bool,
    pub early_branching_in_main: bool,

}

/// Internal struct storing the results of a column generation iteration
pub struct ColGenResult<DualStore> {
    pub obj: f64,
    pub column_x: Vec<(f64, ColumnId)>,
    pub early_branched: bool,
    pub time_limit_reached: bool,
    pub infeasible: bool,
    pub smoothing_center: Option<(f64, DualStore)>,
}

/// Primary trait for the master problem
/// Contains the primary user implementation
pub trait UserMasterProblem<
    ProblemInstance : Clone,
    PricingSolver: UserPricingSolver<ProblemInstance, BranchFilter, ColumnType, DualStore>,
    ColumnType: PartialEq + 'static,
    DualStore: UserDualStore,
    ModelMeta: UserModelMeta<Constr, Var, Env, Model>,
    BranchFilter: IBranchFilter,
    BranchGroupType: std::clone::Clone + Debug,
    Constr: Clone, Env: LPEnv, Model: LPModel<Constr, Var, Env>, Var: Clone
> where ColumnPool<ColumnType, BranchFilter>: ColumnPoolFilter<ColumnType, BranchFilter>
{

    fn new(problem_instance : &ProblemInstance, ui : UISender) -> Self;

    /// Rule to cutoff branches if a primary bound was found.
    ///
    /// Basic implementation provided. But can be tighened based on problem
    /// Suppose the objective function is known to be integral, the current
    /// bound b is 5, and the candidate value a is 4.5:
    ///     The branch may be cut early
    ///
    /// Be careful with rounding issues
    fn is_a_cutoff_by_bound_b(&self, a: &f64, b: &f64) -> bool {
        // generic placeholder, can cut a if greater or equal to b
        // be carefull with rounding in actual implementation
        a >= b
    }

    fn get_ui(&self) -> &UISender;
    fn get_lds_settings(&self) -> &LDSSettings;
    fn get_periodic_integer_solve_interval(&self) -> &Option<usize> { &None }
    fn get_stabilization_settings(&self) -> &StabilizationSettings;

    /// Return a custom branching filter that represents
    /// forcing a column to be either used or not be used
    fn create_column_forcing_filter(&self, column: &Column<ColumnType>, direction: bool) -> BranchFilter;

    /// Test if a branching filter is a column forcing filter. If so return column id that is forced.
    fn is_column_forcing_filter(&self, filter: &BranchFilter) -> Option<ColumnId>;

    /// Given an ordered list of negative reduced cost columns
    /// Return iterator selecting the ones to add to the master problem
    ///
    /// Default implementation selecting the first is provided.
    fn filter_columns_to_add_per_iteration(&self, new_paths: Vec<(f64, ColumnType)>, env: &mut Env, solver: &PricingSolver) -> Box<dyn Iterator<Item=(f64, ColumnType)>> {
        Box::new(new_paths.into_iter().take(1))
    }


    #[inline]
    /// Callback that can terminate pricing a node early.
    /// Such a node is marked, and will be reevaluated before pruning
    fn should_exit_pricing_early(&self, model_meta: &ModelMeta, obj: f64, total_new_columns: usize, total_columns: usize, elapsed: Duration, branch: &Branch<BranchFilter, DualStore>, iterations_counter: i32) -> bool {
        false
    }
    /// Main function running the column generation.
    /// Is provided by the library, should only be overwritten if required.
    fn run_column_generation(
        &self,
        env: &mut Env,
        branch: &Branch<BranchFilter, DualStore>,
        best_known: &RwLock<(f64, Vec<(f64, ColumnId)>)>,
        lower_bound: f64,
        column_pool: &RwLock<ColumnPool<ColumnType, BranchFilter>>,
        solver: &mut PricingSolver,
        time_limit: Option<Instant>,
        integer: bool,
        parent_dual_center: Option<(f64, DualStore)>,
    ) -> ColGenResult<DualStore> {
        let stabilization_settings = self.get_stabilization_settings();


        if integer {
            env.set_time_limit(30.0);
            self.get_ui().send(LogS("Starting integer solve"));
        } else {
            env.set_time_limit(f64::MAX);
        }
        let mut master = Model::new("master", env);


        // create model meta +  restore patterns from column pool
        // grouped so lock on column pool is dropped early

        let mut num_columns = 0;
        let mut model_meta = self.initialize_model(solver, &branch.filters, integer, &mut master);

        // get + sync columns
        // this is required here, as the fix column stage below requires a polulated column pool
        let column_pool_lock = column_pool.read().unwrap();
        let (column_ticket_read, col_iter) = column_pool_lock.get_columns(&branch.filters, None);
        for column in col_iter {
            let var = self.add_var_from_column(
                column,
                &mut master,
                &model_meta,
                &solver,
                integer,
            );
            model_meta.get_pattern_vars().insert(column.id, var);
            num_columns += 1;
        }
        drop(column_pool_lock);
        let mut last_column_ticket = Some(column_ticket_read);


        if let Err(_e) = self.fix_columns_from_filters_in_model(solver, &branch.filters, &mut master, &mut model_meta) {
            // columns couldn't be fixed -> invalid branch

            self.get_ui().send(UIUserMessage::LogS("int columns could not be fixed"));
            return ColGenResult {
                column_x: vec![],
                time_limit_reached: false,
                early_branched: false,
                smoothing_center: branch.old_obj_dual_center.clone(),
                infeasible: true,
                obj: branch.old_obj_bound,
            };
        }


        let mut previous_pricing_runtime = 0.0;

        let mut total_new_columns = 0;
        let mut partial_exit_early = false;
        let mut hit_time_limit = false;

        let colgen_start = Instant::now();

        // initial with parent center
        let mut best_dual_bound_with_store: Option<(f64, DualStore)> = parent_dual_center;


        let mut current_smoothing_alpha = stabilization_settings.initial_smoothing_alpha;

        let mut iterations_counter = 0;
        loop {
            if let Some(tl) = time_limit {
                if Instant::now() > tl {
                    self.get_ui().send(UIUserMessage::TimeLimitReached);
                    hit_time_limit = true;
                    break;
                }
            }

            // get + sync columns

            let column_pool_lock = column_pool.read().unwrap();
            let (column_ticket_read, col_iter) = column_pool_lock.get_columns(&branch.filters, last_column_ticket);
            for column in col_iter {
                let var = self.add_var_from_column(
                    column,
                    &mut master,
                    &model_meta,
                    &solver,
                    integer,
                );
                model_meta.get_pattern_vars().insert(column.id, var);
                num_columns += 1;
            }
            drop(column_pool_lock);
            last_column_ticket = Some(column_ticket_read);


            if !stabilization_settings.cross_iteration_memory {
                current_smoothing_alpha = stabilization_settings.initial_smoothing_alpha;
            }


            model_meta.callback_before_optimize(lower_bound, best_known, &mut master);


            iterations_counter += 1;


            master.optimize();


            // might have hit time limit (in partial pricing oä), therefore test sol count instead of optimality
            if master.get_num_solutions() == 0 {
                if integer { self.get_ui().send(UIUserMessage::LogS("int no solutions")); }
                return ColGenResult {
                    obj: f64::INFINITY,
                    column_x: vec![],
                    early_branched: false,
                    time_limit_reached: false,
                    infeasible: true,
                    smoothing_center: None,
                };
            }


            let master_obj = master.get_objective();
            let runtime = master.get_runtime();


            self.get_ui().send(
                // we show the previous pricing runtime, as that was the pricing that was done for this lp
                UIUserMessage::LPSolveIterationFinish( LPSolveIterationUIState { best_dual_bound: best_dual_bound_with_store.clone().map(|db| db.0), obj: master_obj, lp_runtime: runtime, previous_pricing_runtime, num_columns: num_columns, num_total_pool_columns: total_new_columns })
            );


            if integer {
                // in an integer solve, there are no duals or generated columns
                // so terminate

                break;
            }


            let real_duals = self.get_duals_from_model(solver, &master, &model_meta);


            if branch.allow_heuristic_pricing && self.should_exit_pricing_early(&model_meta, master_obj, total_new_columns, num_columns + total_new_columns, colgen_start.elapsed(), branch, iterations_counter) {
                partial_exit_early = true;
                self.get_ui().send(
                    UIUserMessage::LogS("exit pricing early")
                );
                break;
            }


            let (best_int_obj, best_int_num_columns) = {
                let retrieve = best_known.read().unwrap();
                (retrieve.0, retrieve.1.len())
            };

            let start_pricing = Instant::now();


            let (new_paths, used_duals) = if let (Some((ref in_bound, ref in_duals)), true) = (&best_dual_bound_with_store, current_smoothing_alpha > INT_FEAS_TOL) {
                let mut new_paths: Vec<(f64, ColumnType)>;
                let mut smooth_duals;

                loop {
                    smooth_duals = in_duals.linear_combination(current_smoothing_alpha, &real_duals);
                    // find paths via smooth duals, that are then still neg reduced cost on real duall
                    new_paths =
                        solver.solve(&branch.filters, &smooth_duals).into_iter()
                            .filter_map(|(smoothed_cost, column)| {
                                let real_price = solver.price_column_with_duals(&column, &real_duals);
                                if real_price < -INT_FEAS_TOL {
                                    Some((smoothed_cost, column))
                                } else {
                                    None
                                }
                            })
                            .collect();

                    // if we have none, we must reduce smoothing
                    if new_paths.is_empty() {
                        // if we havent found anything, trust center less and directly reattempt
                        if current_smoothing_alpha > INT_FEAS_TOL {
                            self.get_ui().send(UIUserMessage::LogS("Misprice"));
                            current_smoothing_alpha = (current_smoothing_alpha - stabilization_settings.alpha_steps_down).max(0.0);
                            continue;
                        }
                    } else {
                        // had found paths, trust center more
                        if current_smoothing_alpha <= stabilization_settings.maximum_smoothing_alpha && stabilization_settings.alpha_steps_up > INT_FEAS_TOL {
                            current_smoothing_alpha = (current_smoothing_alpha + stabilization_settings.alpha_steps_up).min(stabilization_settings.maximum_smoothing_alpha);
                        }
                    }

                    // exit (return current paths and used duals)
                    break;
                }

                (new_paths, smooth_duals)
            } else {

                // if we have the real duals, we can just find neg red cost
                let new_paths: Vec<(f64, ColumnType)> =
                    solver.solve(&branch.filters, &real_duals).into_iter()
                        .filter(|fp| fp.0 < -INT_FEAS_TOL)
                        .collect();

                (new_paths, Cow::Borrowed(&real_duals))
            };


            previous_pricing_runtime = start_pricing.elapsed().as_secs_f64();


            // problem here is that multiple columns can be in the optimal solution
            // https://www.or.rwth-aachen.de/files/research/publications/colgen.pdf  Eq (5)

            let bound_on_num_vehicles = solver.bound_num_active_columns(best_int_obj, best_int_num_columns);
            let traditional_dual_bound: f64 = master_obj + new_paths.first().map(|p| p.0).unwrap_or(0.0) * bound_on_num_vehicles as f64;
            //let alternative_dual_bound : f64 = master_obj / (1.0 - new_paths.first().map(|p| p.0).unwrap_or(0.0));
            let dual_bound = traditional_dual_bound;


            // update the best known dual bound
            let best_dual_bound = best_dual_bound_with_store.clone().map(|(val, _)| val).unwrap_or(f64::MIN);

            if dual_bound > best_dual_bound {
                best_dual_bound_with_store = Some((dual_bound, used_duals.into_owned()));
            }

            self.get_ui().send(
                UIUserMessage::PricingProblemFinish(PricingProblemUIState { runtime: previous_pricing_runtime, num_columns: new_paths.len(), dual_bound })
            );


            if new_paths.is_empty() {
                break;
            }


            // apply dual bound similar to gurobi if, no smoothing is active
            if stabilization_settings.initial_smoothing_alpha < INT_FEAS_TOL {
                if (master_obj - best_dual_bound) / master_obj < /* epsilon */ INT_FEAS_TOL {
                    self.get_ui().send(
                        UIUserMessage::Log(format!("Exit pricing due to pr\
                        imal/dual bound {best_dual_bound} / {master_obj}"))
                    );
                    break;
                }

                if (best_int_obj - best_dual_bound) / best_int_obj < /* epsilon */ INT_FEAS_TOL {
                    self.get_ui().send(
                        UIUserMessage::Log(format!("Exit pricing due to best/dual bound {best_dual_bound} / {master_obj}"))
                    );
                    break;
                }
            }


            let mut has_new_column = false;


            {
                let mut rw_column_pool = column_pool.write().unwrap();
                for (cost, path) in self.filter_columns_to_add_per_iteration(new_paths, env, solver)
                {
                    rw_column_pool.add_column(path, column_ticket_read);
                    has_new_column = true;
                    total_new_columns += 1;
                }
            }


            if !has_new_column {
                break;
            }
        }

        let patterns = master.get_x_list(
            &model_meta.get_pattern_vars()
                .iter()
                .map(|(_, v)| v.clone())
                .collect::<Vec<Var>>(),
        )
            .into_iter()
            .zip(model_meta.get_pattern_vars().iter().map(|(i, _)| *i))
            .collect::<Vec<(f64, ColumnId)>>();


        return ColGenResult {
            obj: master.get_objective(),
            column_x: patterns,
            early_branched: partial_exit_early,
            time_limit_reached: hit_time_limit,
            infeasible: false,
            smoothing_center: best_dual_bound_with_store,
        };
    }


    /// Insert the column into the LP model
    /// An user implementation must create a variable,
    /// insert that variable in the constraint matrix
    /// and include it in the model_meta data.
    fn add_var_from_column(
        &self,
        col: &Column<ColumnType>,
        master: &mut Model,
        model_meta: &ModelMeta,
        solver: &PricingSolver,
        integer: bool,
    ) -> Var;



    /// Create an initial lp model constraining problem constraints
    /// Variables from column will be inserted later
    fn initialize_model(&self, solver: &PricingSolver, active_filters: &[BranchFilter], integer: bool, master: &mut Model) -> ModelMeta;

    /// Takes a list of branching filters and based on filters applies
    /// column fixing in the master model
    fn fix_columns_from_filters_in_model(&self, solver: &PricingSolver, active_filters: &[BranchFilter], master: &mut Model, model_meta: &mut ModelMeta) -> Result<(), ()>;

    /// Extract dual values from the underlying model and store them in a
    /// DualStorage object
    fn get_duals_from_model(&self, solver: &PricingSolver, master: &Model, model_meta: &ModelMeta) -> DualStore;

    /// Given a LP Solution, return a list of all possible branching options
    fn find_fractional_solutions(
        &self,
        solver: &PricingSolver,
        active_filters: &[BranchFilter],
        column_pool: &ColumnPool<ColumnType, BranchFilter>,
        patterns: &Vec<(f64, ColumnId)>,
    ) -> Vec<BranchGroup<BranchFilter, BranchGroupType>>;

    /// Selects the column to branch on in limted discrepancy search
    /// Library provided implementation should only be overriden if required.
    fn select_fractional_columns_to_fix_in_lds(
        &self,
        solver: &PricingSolver,
        active_filters: &[BranchFilter],
        column_pool: &ColumnPool<ColumnType, BranchFilter>,
        patterns: &Vec<(f64, ColumnId)>,
        taboo_filters: &HashSet<BranchFilter>,
    ) -> Vec<BranchFilter> {
        let already_fixed_columns: HashSet<ColumnId> = active_filters.iter().filter_map(|f| self.is_column_forcing_filter(f)).collect();


        let mut chosen_patterns: Vec<(f64, &Column<ColumnType>)> = patterns
            .iter()
            .filter(|(v, c)| !already_fixed_columns.contains(&c))
            .map(|(v, c)| (*v, column_pool.get_column(*c)))
            .collect();


        if chosen_patterns.is_empty() {
            return vec![];
        }


        //unique_conflicting_resources

        match self.get_lds_settings().selection {
            LDSSelection::ClosestToOne => {
                // sort by distance to 1.0
                chosen_patterns.sort_unstable_by(|a, b| {
                    let distance_to_a = (1.0 - a.0).abs();
                    let distance_to_b = (1.0 - b.0).abs();
                    distance_to_a.partial_cmp(&distance_to_b).unwrap()
                });
            }
            LDSSelection::ClosestToInt => {
                chosen_patterns.sort_unstable_by(|a, b| {
                    let distance_to_a = (a.0.round() - a.0).abs();
                    let distance_to_b = (b.0.round() - b.0).abs();
                    distance_to_a.partial_cmp(&distance_to_b).unwrap()
                });
            }
        }


        let mut forced_columns = Vec::default();
        let mut forced_true_resources: Vec<HashMap<usize, bool>> = Vec::default();

        let mut pattern_iter = chosen_patterns.iter();
        'patternWhile: while let (Some((colval, col)), true) = (pattern_iter.next(), forced_columns.len() < self.get_lds_settings().columns_to_fix_each_iteration) {
            if matches!(self.get_lds_settings().selection, LDSSelection::ClosestToOne ) || *colval > 0.5 {
                let candidate = self.create_column_forcing_filter(col, true);

                if !taboo_filters.contains(&candidate) {
                    forced_columns.push(candidate)
                }
            } else {
                if let Some(potential_used_resources) = solver.unique_conflicting_resources_of_column(&col.data) {
                    // test if we can actually fix this togerher with the other we wanted to fix
                    for old_res in &forced_true_resources {
                        if old_res.iter().any(|(k, v)| *v == true && *potential_used_resources.get(k).unwrap_or(&false) == true) {
                            // consider next pattern due to resource conflict
                            continue 'patternWhile;
                        }
                    }


                    let candidate = self.create_column_forcing_filter(col, false);

                    if !taboo_filters.contains(&candidate) {
                        forced_columns.push(candidate);
                        forced_true_resources.push(potential_used_resources);
                    }
                }
            }
        }


        return forced_columns;
    }

    /// Given the list of identified branching groups, select the best
    /// Default implementation simply selects first
    /// Most fractional or similar might be more appropriate
    fn choose_fractional_solution(
        &self,
        obj: f64,
        fractional_options: Vec<BranchGroup<BranchFilter, BranchGroupType>>,
        best: f64,
        branch: &Branch<BranchFilter, DualStore>,
        solver: &mut PricingSolver,
        env: &mut Env,
        column_pool: &RwLock<ColumnPool<ColumnType, BranchFilter>>,
        patterns: &Vec<(f64, ColumnId)>,
    ) -> Option<BranchGroup<BranchFilter, BranchGroupType>> {
        fractional_options.first().cloned()
    }
}

use compare::Compare;


use crate::ui::{BranchUIState, UIUserMessage};
use crate::{LPSolveIterationUIState, NewBestUIState, PricingProblemUIState, UI, UISender};
use crate::misc::{HashMap, HashSet};
use crate::UIUserMessage::LogS;

/// Struct storing information about branch and price progress
/// That is shared among all workers.
///
/// Contains open branches, bound and progress information
pub struct SharedState<ColumnType, BranchFilter, C, DualStore>
    where
        DualStore: UserDualStore,
        BranchFilter: std::clone::Clone + IBranchFilter,
        C: Compare<Branch<BranchFilter, DualStore>> + Clone
{
    pub best: RwLock<(f64, Vec<(f64, ColumnId)>)>,
    pub open_branches: BlockingQueue<BranchFilter, C, DualStore>,
    pub column_pool: RwLock<ColumnPool<ColumnType, BranchFilter>>,
    pub last_integer_at_cp_size: AtomicUsize,
    pub ui_sender: UISender,
    pub start_time: Instant,
    pub duration_root_node: Mutex<Option<Duration>>,
    pub obj_root_node: Mutex<Option<f64>>,
    pub duration_lds_root_node: Mutex<Option<Duration>>,
    pub time_limit: Option<Instant>,
    pub phase: RwLock<Phase>,
}

impl<ColumnType, BranchFilter : std::clone::Clone + IBranchFilter, C : Compare<Branch<BranchFilter, DualStore>> + Clone, DualStore : UserDualStore>
    SharedState<ColumnType, BranchFilter, C, DualStore> {


    pub fn new(column_pool : ColumnPool<ColumnType,BranchFilter>, ui : &UI, timelimit : Option<Instant>, open_branches : BlockingQueue<BranchFilter, C, DualStore>) -> Self {
        SharedState {
            phase: RwLock::new(Phase::Preparation),
            best: RwLock::new((f64::INFINITY, Vec::default())),
            column_pool: RwLock::new(column_pool),
            last_integer_at_cp_size: AtomicUsize::new(0),
            open_branches,
            duration_root_node: Mutex::new(None),
            obj_root_node: Mutex::new(None),
            duration_lds_root_node: Mutex::new(None),
            ui_sender: ui.get_sender(),
            start_time: Instant::now(),
            time_limit: timelimit,
        }
    }

}

#[derive(Clone, Copy)]
/// Phase of the branch-and-price progress
/// Currently only Primal Heuristic or Main.
/// Primarily used for defensive coding practices
/// To ensure Limited Discrepancy Search and the
/// Main Method is not mixed
pub enum Phase {
    Preparation,
    PrimalHeuristic,
    Main,
}
