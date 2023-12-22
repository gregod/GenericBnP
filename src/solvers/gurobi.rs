use crate::{LPEnv, LPModel, UserModelMeta};
use gurobi::{param,attr};

pub struct  GurobiEnv(pub gurobi::Env);



impl LPEnv for GurobiEnv {
    fn new_with_seed(seed: i32) -> Self {

        let mut env = gurobi::Env::new("").unwrap();
        env.set(param::Threads, 1).unwrap();
        env.set(param::Seed, seed).unwrap();
        env.set(param::OutputFlag, 0).unwrap();
        GurobiEnv(env)
    }

    fn set_time_limit(&mut self, seconds: f64) {
        self.0.set(param::TimeLimit, seconds).unwrap()
    }


}

impl LPModel<gurobi::Constr, gurobi::Var, GurobiEnv> for gurobi::Model {
    fn new(name: &str, env: &mut GurobiEnv) -> Self {
        gurobi::Model::new(name, &mut env.0).unwrap()
    }



    fn get_dual_list(&self, constrs: &[gurobi::Constr]) -> Vec<f64> {
        self.get_values(attr::Pi, constrs).unwrap()
    }

    fn get_x_list(&self, vars: &[gurobi::Var]) -> Vec<f64> {
        self.get_values(attr::X, vars).unwrap()
    }

    fn get_objective(&self) -> f64 {
        self.get(attr::ObjVal).unwrap()
    }

    fn get_runtime(&self) -> f64 {
        self.get(attr::Runtime).unwrap()
    }

    fn get_num_solutions(&self) -> i32 {
        self.get(attr::SolCount).unwrap()
    }

    fn optimize(&mut self) {
        self.update().unwrap();
        self.optimize().unwrap();
    }
}
