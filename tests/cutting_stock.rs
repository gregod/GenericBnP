#![feature(int_roundings)]
// Problem adapted from https://www2.imm.dtu.dk/courses/02717/columngeneration/columngeneration.pdf

#[cfg(test)]
mod tests {
    use generic_bnp::branch::BranchGroup;
    use generic_bnp::column_pool::{ColumnId, ColumnPool, ColumnPoolFilter, ColumnTicket};

    use generic_bnp::misc::{FullHashMap, HashMap};
    use generic_bnp::solvers::gurobi::GurobiEnv;
    use generic_bnp::{
        BnPFactory, BranchAndPrice, IBranchFilter, LDSSelection, LDSSettings, LPModel,
        MasterProblem, Phase, SharedState, StabilizationSettings, UISender, UIUserMessage,
        UserDualStore, UserMasterProblem, UserModelMeta, UserPricingSolver, INT_FEAS_TOL, UI,
    };
    use gurobi::{Constr, ConstrSense, Less, LinExpr, Maximize, Model, Var, VarType};
    use itertools::Itertools;
    use std::borrow::Cow;
    use std::cmp::Ordering;
    use std::fmt::{Debug, Display, Formatter};
    use std::sync::{Arc, RwLock};

    struct PricingSolver {
        instance: CuttingStockProblem,
        gurobi_env: gurobi::Env,
    }

    impl<'a> UserPricingSolver<CuttingStockProblem, Branch, Column, DualStorage> for PricingSolver {
        fn new(problem_instance: &CuttingStockProblem, ui: UISender) -> Self {
            let mut env = gurobi::Env::new("/tmp/gurobi.log").unwrap();
            env.set(gurobi::param::OutputFlag, 0).unwrap();

            PricingSolver {
                gurobi_env: env,
                instance: problem_instance.clone(),
            }
        }
        fn solve(&mut self, active_filters: &[Branch], duals: &DualStorage) -> Vec<(f64, Column)> {
            // create knapsack problem

            let mut model = gurobi::Model::new("pricing", &self.gurobi_env).unwrap();
            model.set_objective(LinExpr::new(), Maximize).unwrap();

            let length_constraint = model
                .add_constr(
                    "length",
                    LinExpr::new(),
                    Less,
                    f64::from(self.instance.length_rod),
                )
                .unwrap();
            let count_vars: Vec<Var> = self
                .instance
                .demand_for_pieces
                .iter()
                .map(|d| {
                    model
                        .add_var(
                            &format!("demand_{:?}", d.size),
                            VarType::Integer,
                            duals.sizes[&d.size],
                            0.0,
                            f64::INFINITY,
                            &[length_constraint.clone()],
                            &[f64::from(d.size.0)],
                        )
                        .unwrap()
                })
                .collect();

            model.update().unwrap();
            model.optimize().unwrap();

            let results = model.get_x_list(&count_vars);

            let pieces = self
                .instance
                .demand_for_pieces
                .iter()
                .zip(results)
                .map(|(p, res)| PieceItem {
                    size: p.size,
                    count: PieceCount(res as u8),
                })
                .collect();

            vec![(1.0 - model.get_objective(), Column { cuts: pieces })]
        }

        fn primal_cost_column(&self, column: &Column) -> f64 {
            1.0
        }

        fn price_column_with_duals(&self, column: &Column, duals: &DualStorage) -> f64 {
            1.0 - column
                .cuts
                .iter()
                .map(|p| duals.sizes[&p.size] * f64::from(p.count.0))
                .sum::<f64>()
        }

        fn bound_num_active_columns(
            &self,
            best_int_obj: f64,
            current_best_used_columns: usize,
        ) -> usize {
            usize::from(
                self.instance
                    .demand_for_pieces
                    .iter()
                    .map(|d| d.count.0)
                    .sum::<u8>(),
            )
        }
    }

    #[derive(Clone, Debug)]
    struct DualStorage {
        sizes: FullHashMap<PieceSize, f64>,
    }

    impl UserDualStore for DualStorage {
        fn linear_combination<'a, 'b>(&'b self, alpha: f64, out_duals: &'a Self) -> Cow<'a, Self> {
            if alpha < INT_FEAS_TOL {
                return Cow::Borrowed(out_duals);
            }

            let mut out: FullHashMap<PieceSize, f64> = FullHashMap::default();


            // self * alpha + out * (1-alpha)

            for self_size in &self.sizes {
                let other_value = out_duals.sizes[self_size.0];
                out.insert(
                    self_size.0.clone(),
                    self_size.1 * alpha + other_value * (1.0 - alpha),
                );
            }

            return Cow::Owned(DualStorage { sizes: out });
        }
    }

    #[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
    struct PieceItem {
        pub size: PieceSize,
        pub count: PieceCount,
    }

    #[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
    struct PieceCount(u8);

    #[derive(PartialEq, Eq, Hash, Debug, Clone, Copy)]
    struct PieceSize(u8);

    #[derive(PartialEq, Eq, Hash, Clone)]
    struct Column {
        cuts: Vec<PieceItem>,
    }

    impl Debug for Column {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            for r in self.cuts.iter().filter(|p| p.count.0 > 0) {
                write!(f, "{} x {}cm, ", r.count.0, r.size.0)?
            }
            write!(f, "")
        }
    }

    #[derive(Clone, Debug, Eq, PartialEq, Hash)]
    enum Branch {
        ForceColumn(ColumnId, bool),
    }

    impl IBranchFilter for Branch {}

    impl Display for Branch {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            writeln!(f, "{:?}", self)
        }
    }

    struct ModelMeta {
        column_vars: HashMap<ColumnId, Var>,
        demand_constr: FullHashMap<PieceSize, Constr>,
    }

    impl UserModelMeta<Constr, Var, GurobiEnv, gurobi::Model> for ModelMeta {
        fn get_pattern_vars(&mut self) -> &mut HashMap<ColumnId, Var> {
            &mut self.column_vars
        }
        fn callback_before_optimize(
            &mut self,
            lower_bound: f64,
            best_known: &RwLock<(f64, Vec<(f64, ColumnId)>)>,
            model: &mut Model,
        ) {
            model.write("/tmp/model.lp").unwrap();
        }
    }

    #[derive(Debug, Clone)]
    enum BranchGroupType {}

    #[derive(Debug, Clone)]
    struct CuttingStockProblem {
        length_rod: u8,
        demand_for_pieces: Vec<PieceItem>,
    }

    #[derive(Clone)]
    struct CuttingStockMasterProblem {
        ui: UISender,
        instance: CuttingStockProblem,
    }

    impl
        UserMasterProblem<
            CuttingStockProblem,
            PricingSolver,
            Column,
            DualStorage,
            ModelMeta,
            Branch,
            BranchGroupType,
            gurobi::Constr,
            GurobiEnv,
            gurobi::Model,
            gurobi::Var,
        > for CuttingStockMasterProblem
    {
        fn new(problem_instance: &CuttingStockProblem, ui: UISender) -> Self {
            CuttingStockMasterProblem {
                instance: problem_instance.clone(),
                ui,
            }
        }

        fn get_ui(&self) -> &UISender {
            &self.ui
        }

        fn get_lds_settings(&self) -> &LDSSettings {
            &LDSSettings {
                columns_to_fix_each_iteration: 0,
                selection: LDSSelection::ClosestToOne,
                max_depth: 0,
                max_discrepancy: 0,
                early_termination_after_bound_leq: 0,
                do_only_lds: false,
            }
        }

        fn get_stabilization_settings(&self) -> &StabilizationSettings {
            &StabilizationSettings {
                cross_iteration_memory: false,
                initial_smoothing_alpha: 0.9,
                maximum_smoothing_alpha: 0.99,
                alpha_steps_up: 0.0,
                alpha_steps_down: 0.1,
            }
        }

        fn create_column_forcing_filter(
            &self,
            column: &generic_bnp::column_pool::Column<Column>,
            direction: bool,
        ) -> Branch {
            Branch::ForceColumn(column.id, direction)
        }

        fn is_column_forcing_filter(&self, filter: &Branch) -> Option<ColumnId> {
            if let Branch::ForceColumn(col_id, dir) = filter {
                return Some(*col_id);
            }
            None
        }

        fn add_var_from_column(
            &self,
            col: &generic_bnp::column_pool::Column<Column>,
            master: &mut Model,
            model_meta: &ModelMeta,
            solver: &PricingSolver,
            integer: bool,
        ) -> Var {
            let mut constr_vec: Vec<Constr> = Vec::default();
            let mut vald_vec: Vec<f64> = Vec::default();

            for item in &col.data.cuts {
                if item.count.0 > 0 {
                    constr_vec.push(model_meta.demand_constr[&item.size].clone());
                    vald_vec.push(f64::from(item.count.0))
                }
            }

            master
                .add_var(
                    &format!("column_var[{:?}]", col.id),
                    if integer {
                        VarType::Integer
                    } else {
                        VarType::Continuous
                    },
                    1.0,
                    0.0,
                    f64::INFINITY,
                    &constr_vec,
                    &vald_vec,
                )
                .unwrap()
        }

        fn initialize_model(
            &self,
            solver: &PricingSolver,
            active_filters: &[Branch],
            integer: bool,
            master: &mut Model,
        ) -> ModelMeta {
            let mut constr: FullHashMap<PieceSize, Constr> = FullHashMap::default();

            for item in &self.instance.demand_for_pieces {
                constr.insert(
                    item.size.clone(),
                    master
                        .add_constr(
                            &format!("demand_for_{:?}", item.size),
                            LinExpr::new(),
                            ConstrSense::Greater,
                            f64::from(item.count.0),
                        )
                        .unwrap(),
                );
            }

            ModelMeta {
                column_vars: Default::default(),
                demand_constr: constr,
            }
        }

        fn fix_columns_from_filters_in_model(
            &self,
            solver: &PricingSolver,
            active_filters: &[Branch],
            master: &mut Model,
            model_meta: &mut ModelMeta,
        ) -> Result<(), ()> {
            for filter in active_filters {
                if let Branch::ForceColumn(col, dir) = filter {
                    if let Some(var) = model_meta.column_vars.get(&col) {
                        master
                            .add_constr(
                                &format!("forceColumnActive[{:?}]", col),
                                1.0 * var,
                                gurobi::Equal,
                                if *dir { 1.0 } else { 0.0 },
                            )
                            .unwrap();
                    } else {
                        return Err(());
                    }
                }
            }

            Ok(())
        }

        fn get_duals_from_model(
            &self,
            solver: &PricingSolver,
            master: &Model,
            model_meta: &ModelMeta,
        ) -> DualStorage {
            let dual_vec: FullHashMap<PieceSize, f64> = model_meta
                .demand_constr
                .iter()
                .map(|(size, constr)| (size.clone(), master.get_dual_list(&[constr.clone()])[0]))
                .collect();

            DualStorage { sizes: dual_vec }
        }

        fn find_fractional_solutions(
            &self,
            solver: &PricingSolver,
            active_filters: &[Branch],
            column_pool: &ColumnPool<Column, Branch>,
            patterns: &Vec<(f64, ColumnId)>,
        ) -> Vec<BranchGroup<Branch, BranchGroupType>> {
            return Vec::default();
        }
    }

    impl ColumnPoolFilter<Column, Branch> for ColumnPool<Column, Branch> {
        fn get_columns(
            &self,
            filters: &[Branch],
            ticket: Option<ColumnTicket>,
        ) -> (ColumnTicket, Vec<&generic_bnp::column_pool::Column<Column>>) {
            // if we have an inbound ticket: only get those columns after the ticket
            let (new_ticket, iter) = {
                let (tick, iter) = self.get_all_columns();
                (
                    tick,
                    iter.skip(if let Some(ticket) = ticket {
                        ticket.0
                    } else {
                        0
                    }),
                )
            };

            return (new_ticket, iter.collect());
        }
    }

    #[test]
    fn run() {
        let ui = UI::new();

        let problem_instance = CuttingStockProblem {
            length_rod: 9,
            demand_for_pieces: vec![
                PieceItem {
                    size: PieceSize(2),
                    count: PieceCount(4),
                },
                PieceItem {
                    size: PieceSize(3),
                    count: PieceCount(2),
                },
                PieceItem {
                    size: PieceSize(4),
                    count: PieceCount(6),
                },
                PieceItem {
                    size: PieceSize(5),
                    count: PieceCount(6),
                },
                PieceItem {
                    size: PieceSize(6),
                    count: PieceCount(2),
                },
                PieceItem {
                    size: PieceSize(7),
                    count: PieceCount(2),
                },
                PieceItem {
                    size: PieceSize(8),
                    count: PieceCount(2),
                },
            ],
        };

        let bnp: BranchAndPrice<
            Branch,
            Column,
            DualStorage,
            ModelMeta,
            BranchGroupType,
            gurobi::Constr,
            GurobiEnv,
            Model,
            gurobi::Var,
            PricingSolver,
            CuttingStockProblem,
            CuttingStockMasterProblem,
        > = BranchAndPrice::new(problem_instance.clone());

        let mut column_pool = ColumnPool::new();
        // seed initial columns
        problem_instance
            .demand_for_pieces
            .iter()
            .map(|p| Column {
                cuts: vec![PieceItem {
                    size: p.size,
                    count: PieceCount(problem_instance.length_rod.div_floor(p.size.0)),
                }],
            })
            .for_each(|c| {
                column_pool.add_column(c, ColumnTicket(0));
            });

        let (best,results) = bnp.solve(&ui, 1, column_pool);

        assert_eq!(best.ceil(), 13.0);

        println!("Results: {:?}", results);

    }
}
