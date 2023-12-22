#![warn(warnings)]
#![warn(clippy::all, clippy::pedantic)]
#![allow(non_upper_case_globals)]
#![allow(clippy::needless_return)]
#![allow(clippy::items_after_statements)]
#![allow(unused_variables, unused_imports, dead_code)]

pub mod misc;
mod generic_master_problem;
mod ui;
pub mod solvers;

use std::fmt::Debug;
use std::marker::PhantomData;
use std::sync::Arc;
use std::thread;
pub use generic_master_problem::*;

pub use ui::*;
use crate::branch::{Branch, BranchComparator};
use crate::column_pool::{ColumnId, ColumnPool, ColumnPoolFilter};
use crate::misc::blocking_queue::BlockingQueue;


pub trait BnPFactory<Type, ProblemInstance> {
    fn new(&self, instance : &ProblemInstance) -> Type;
}

pub struct BranchAndPrice<BranchFilter: IBranchFilter, Column : PartialEq + 'static, DualStore : UserDualStore + Sync,  ModelMeta : UserModelMeta<Constr,Var,Env,Model>,
    BranchGroupType : Clone + Debug + Send + Sync, Constr : Clone, Env : LPEnv,Model : LPModel<Constr,Var,Env>,Var : Clone,
    PricingSolver : UserPricingSolver<ProblemInstance, BranchFilter, Column, DualStore> ,
    ProblemInstance : Clone + Send + Sync,
    UMasterProblem : UserMasterProblem<ProblemInstance, PricingSolver, Column, DualStore, ModelMeta, BranchFilter, BranchGroupType, Constr,Env,Model,Var> + Clone,
>
where ColumnPool<Column, BranchFilter> : ColumnPoolFilter<Column, BranchFilter>{

    _b : PhantomData<BranchFilter>,
    _c : PhantomData<Column>,
    _bpt: PhantomData<BranchGroupType>,
    _mm: PhantomData<ModelMeta>,
    _ds: PhantomData<DualStore>,
    _cs: PhantomData<Constr>,
    _ev: PhantomData<Env>,
    _md: PhantomData<Model>,
    _vr: PhantomData<Var>,
    _pi: PhantomData<ProblemInstance>,
    _ps : PhantomData<PricingSolver>,
    _mp : PhantomData<UMasterProblem>,


    problem_instance : ProblemInstance,
}

impl<BranchFilter: IBranchFilter + Sync + Send, Column : PartialEq + 'static + Clone + Send + Sync, DualStore : UserDualStore + Sync + Send,  ModelMeta : UserModelMeta<Constr,Var,Env,Model>,
    BranchGroupType : Clone + Debug + Send + Sync, Constr : Clone, Env : LPEnv,Model : LPModel<Constr,Var,Env>,Var : Clone,
    PricingSolver : UserPricingSolver<ProblemInstance, BranchFilter, Column, DualStore>,
    ProblemInstance: Clone + Send + Sync,
    UMasterProblem : UserMasterProblem<ProblemInstance,PricingSolver, Column, DualStore, ModelMeta, BranchFilter, BranchGroupType, Constr,Env,Model,Var> + Clone + Send + Sync,
> BranchAndPrice<BranchFilter, Column, DualStore, ModelMeta, BranchGroupType, Constr, Env, Model, Var, PricingSolver, ProblemInstance, UMasterProblem> where ColumnPool<Column, BranchFilter> : ColumnPoolFilter<Column, BranchFilter>{
    
    
    pub fn new(   problem_instance : ProblemInstance) -> Self {
        
        BranchAndPrice {
            _b: PhantomData,
            _c: PhantomData,
            _bpt: PhantomData,
            _mm: PhantomData,
            _ds: PhantomData,
            _cs: PhantomData,
            _ev: PhantomData,
            _md: PhantomData,
            _vr: PhantomData,
            _pi : PhantomData,
            _ps: PhantomData,
            _mp: PhantomData,

            problem_instance
        }
    }


    pub fn solve(&self, ui : &UI, num_threads : usize, column_pool : ColumnPool<Column, BranchFilter>) -> (f64, Vec<(f64, Column)>) {

        let open_branches = BlockingQueue::new(
            BranchComparator::with_bound(),
            BranchComparator::without_bound()
        );



        let shared = Arc::new(SharedState::new(column_pool, &ui, None, open_branches));






        // start with phase main
        {
            *shared.phase.write().unwrap() = Phase::PrimalHeuristic;
        }
        shared
            .open_branches
            .add_job(Branch::create_lds_root());


        shared.ui_sender.send(UIUserMessage::StartPhase("Primal Heuristic", 0));
        // run main
        thread::scope(|s| {
            for i in 0..num_threads {
                let shared = shared.clone();

                let pi = self.problem_instance.clone();
                s.spawn(move || {
                    let mp  = UMasterProblem::new(&pi, shared.ui_sender.clone());
                    let ps = PricingSolver::new(&pi, shared.ui_sender.clone());
                    let mut master_problem = MasterProblem::new(
                        shared,
                        mp, ps
                    );
                    master_problem.run_worker(0.0, f64::INFINITY, (0 + i) as i32);
                });
            }
        });

        // start with phase main
        {
            *shared.phase.write().unwrap() = Phase::Main;
        }
        shared
            .open_branches
            .add_job(Branch::default());

        shared.ui_sender.send(UIUserMessage::StartPhase("Main Phase", 0));
        thread::scope(|s| {
            for i in 0..num_threads {
                let shared = shared.clone();

                let pi = self.problem_instance.clone();
                s.spawn(move || {
                    let mp  = UMasterProblem::new(&pi, shared.ui_sender.clone());
                    let ps = PricingSolver::new(&pi, shared.ui_sender.clone());
                    let mut master_problem = MasterProblem::new(
                        shared,
                        mp, ps
                    );
                    master_problem.run_worker(0.0, f64::INFINITY, (0 + i) as i32);
                });
            }
        });

        ui.get_sender().send(UIUserMessage::ExitUi {
            root_node: shared.duration_root_node.lock().unwrap().clone(),
            lds_root_node_duration: shared.duration_lds_root_node.lock().unwrap().clone(),
        });


        let (best,pattern) =  { shared.best.read().unwrap().to_owned() };


        let columns: Vec<(f64, Column)> = pattern
            .iter()
            .filter(|(v, _)| *v > 0.0)
            .map(|(v, c)| {
                let c = shared
                    .column_pool
                    .read()
                    .unwrap()
                    .get_column(*c)
                    .data
                    .clone();
                (*v,c)
            })
            .collect();


        (best, columns)
    }
    
    
}