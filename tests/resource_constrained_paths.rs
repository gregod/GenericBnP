#![feature(int_roundings)]
// Problem taken from:
//  Desrosiers, Jacques & Lübbecke, Marco. (2006). A Primer in Column Generation. 10.1007/0-387-25486-2_1.

#[cfg(test)]
mod tests {
    use generic_bnp::branch::BranchGroup;
    use generic_bnp::column_pool::{ ColumnId, ColumnPool, ColumnPoolFilter, ColumnTicket};

    use generic_bnp::misc::{FullHashMap, HashMap};
    use generic_bnp::solvers::gurobi::GurobiEnv;
    use generic_bnp::{
         BranchAndPrice, IBranchFilter, LDSSelection, LDSSettings, LPModel, StabilizationSettings, UISender, UserDualStore, UserMasterProblem, UserModelMeta, UserPricingSolver, INT_FEAS_TOL, UI
    };
    use gurobi::ConstrSense::Equal;
    use gurobi::{Constr,  Less, LinExpr, Model, Var};

    use std::borrow::Cow;
    use std::fmt::{Debug, Display, Formatter};
    use std::hash::Hash;
    use std::sync::RwLock;


    use petgraph::algo::simple_paths;
    use petgraph::prelude::*;
    
    struct PricingSolver {
        instance: ShortestPathsProblem,
        ui: UISender,
    }


    impl<'a> UserPricingSolver<ShortestPathsProblem, Branch, Column, DualStorage> for PricingSolver {
        fn new(problem_instance: &ShortestPathsProblem, ui: UISender) -> Self {
            PricingSolver {
                instance: problem_instance.clone(),
                ui,
            }
        }
        fn solve(&mut self, active_filters: &[Branch], duals: &DualStorage) -> Vec<(f64, Column)> {
            

            // create local copy of graph to apply branching constraints
            let mut local_graph = self.instance.graph.clone();

            for filter in active_filters {
                match filter {
                    Branch::ForceEdge(edge, dir) => {
                        if *dir == true {
                            // if we must take an edge -> find all other edges leaving that node and remove them.

                            let edge_nodes = local_graph.edge_endpoints(*edge).unwrap();
                            let edges_to_remove: Vec<EdgeIndex> = local_graph
                                .edges_directed(edge_nodes.0, Outgoing)
                                .map(|e| e.id())
                                .filter(|other_edge| *other_edge != *edge)
                                .collect();

                            for e in edges_to_remove {
                                local_graph.remove_edge(e);
                            }

                        } else {
                            // if we must NOT take an edge -> simply remove the edge
                            local_graph.remove_edge(*edge);
                        }
                    }
                    Branch::ForceColumn(_, _) => { /* not applicable in pricing */ }
                }
            }


            /* very naive shortest path implementation through enumeration:
                Find all paths, calculate costs, then pick the one with lowest cost.
                Allows for easy debugging of dual cost vectors + is feasible on the toy problem.
                
                (The petgraph bellman-ford implementation has no easy way to customize cost function,
                and djikstra/a* do not work due to negative edge costs)
             */

            let all_paths = simple_paths::all_simple_paths::<Vec<_>, _>(
                &local_graph,
                self.instance.graph_start,
                self.instance.graph_end,
                0,
                None,
            );


            let min_cost_path = all_paths
                .map(|path| {
                    // turn nodes into edges
                    let mut visited_edges = Vec::new();
                    for node_index in path.windows(2) {
                        let source_node = node_index[0];
                        let target_node = node_index[1];
                        if let Some(edge) = local_graph.find_edge(source_node, target_node) {
                            visited_edges.push(edge);
                        }
                    }
                    // compute costs
                    let cost_sum = visited_edges
                        .iter()
                        .map(|e| {
                            f64::from(local_graph[*e].cost)
                                - duals.time * f64::from(local_graph[*e].time)
                        })
                        .sum::<f64>();

                    (cost_sum, visited_edges)
                })
                // pick smallest cost path
                .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

            // if we have found a suitable path, build a column
            if let Some((result_cost, result_path)) = min_cost_path {
                
                let cost_sum = result_path
                    .iter()
                    .map(|e| u32::from(local_graph[*e].cost))
                    .sum();
                let time_sum = result_path
                    .iter()
                    .map(|e| u32::from(local_graph[*e].time))
                    .sum();

                let column = Column {
                    path: result_path.iter().map(|e| *e).collect(),
                    time: time_sum,
                    cost: cost_sum,
                };

                vec![(result_cost - duals.convexity, column)]
            } else {
                // otherwise report that no columns were found
                vec![]
            }
        }

        fn primal_cost_column(&self, column: &Column) -> f64 {
            f64::from(column.cost)
        }

        fn price_column_with_duals(&self, column: &Column, duals: &DualStorage) -> f64 {
            // run cost function on existing column and dual object
            column
                    .path
                    .iter()
                    .map(|e| {
                        f64::from(self.instance.graph[*e].cost)
                            - duals.time * f64::from(self.instance.graph[*e].time)
                    })
                    .sum::<f64>()
                - duals.convexity
        }

        fn bound_num_active_columns(
            &self,
            _best_int_obj: f64,
            _current_best_used_columns: usize,
        ) -> usize {
            // optimal solution consists of a single column
            1
        }

    }

    #[derive(Clone, Debug)]
    struct DualStorage {
        time: f64,
        convexity: f64,
    }

    impl UserDualStore for DualStorage {
        fn linear_combination<'a, 'b>(&'b self, alpha: f64, out_duals: &'a Self) -> Cow<'a, Self> {
            if alpha < INT_FEAS_TOL {
                return Cow::Borrowed(out_duals);
            }

            return Cow::Owned(DualStorage {
                time: self.time * alpha + out_duals.time * (1.0 - alpha),
                convexity: self.convexity * alpha + out_duals.convexity * (1.0 - alpha),
            });
        }
    }

    #[derive(PartialEq, Eq, Hash, Clone, Debug)]
    struct Column {
        path: Vec<EdgeIndex>,
        cost: u32,
        time: u32,
    }

    #[derive(Clone, Debug, Eq, PartialEq, Hash)]
    enum Branch {
        ForceColumn(ColumnId, bool),
        ForceEdge(EdgeIndex, bool),
    }

    impl IBranchFilter for Branch {}

    impl Display for Branch {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            writeln!(f, "{:?}", self)
        }
    }

    struct ModelMeta {
        column_vars: HashMap<ColumnId, Var>,
        time_constraint: Constr,
        convexity_constraint: Constr,
    }

    impl UserModelMeta<Constr, Var, GurobiEnv, gurobi::Model> for ModelMeta {
        fn get_pattern_vars(&mut self) -> &mut HashMap<ColumnId, Var> {
            &mut self.column_vars
        }
    }

    #[derive(Debug, Clone)]
    enum BranchGroupType {
        Default,
    }

    #[derive(Debug, Clone)]
    struct ShortestPathsProblem {
        graph: StableDiGraph<ProblemNode, ProblemEdge>,
        graph_start: NodeIndex,
        graph_end: NodeIndex,
        max_time: u8,
    }

    #[derive(Debug, Clone)]
    struct ProblemEdge {
        cost: u8,
        time: u8,
    }

    #[derive(Debug, Clone)]
    struct ProblemNode {}

    #[derive(Clone)]
    struct ShortestPathsMasterProblem {
        ui: UISender,
        instance: ShortestPathsProblem,
    }

    impl
        UserMasterProblem<
            ShortestPathsProblem,
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
        > for ShortestPathsMasterProblem
    {
        fn new(problem_instance: &ShortestPathsProblem, ui: UISender) -> Self {
            ShortestPathsMasterProblem {
                instance: problem_instance.clone(),
                ui,
            }
        }

        fn get_ui(&self) -> &UISender {
            &self.ui
        }

        fn get_stabilization_settings(&self) -> &generic_bnp::StabilizationSettings {
            &StabilizationSettings {
                cross_iteration_memory: false,
                initial_smoothing_alpha: 0.90,
                maximum_smoothing_alpha: 0.99,
                alpha_steps_up: 0.0,
                alpha_steps_down: 0.50,
            }
        }

        fn initialize_model(
            &self,
            _solver: &PricingSolver,
            _active_filters: &[Branch],
            _integer: bool,
            master: &mut Model,
        ) -> ModelMeta {

            let convexity_constraint = master
                .add_constr("convexity", LinExpr::new(), Equal, 1.0)
                .unwrap();

            let time_constraint = master
                .add_constr(
                    "max_time",
                    LinExpr::new(),
                    Less,
                    f64::from(self.instance.max_time),
                )
                .unwrap();

            ModelMeta {
                column_vars: Default::default(),
                convexity_constraint,
                time_constraint,
            }
        }

        fn get_duals_from_model(
            &self,
            _solver: &PricingSolver,
            master: &Model,
            model_meta: &ModelMeta,
        ) -> DualStorage {
            let duals_time_convexity = master.get_dual_list(&[
                model_meta.time_constraint.clone(),
                model_meta.convexity_constraint.clone(),
            ]);

            DualStorage {
                time: duals_time_convexity[0],
                convexity: duals_time_convexity[1],
            }
        }

        fn find_fractional_solutions(
            &self,
            _solver: &PricingSolver,
            _active_filters: &[Branch],
            column_pool: &ColumnPool<Column, Branch>,
            patterns: &Vec<(f64, ColumnId)>,
        ) -> Vec<BranchGroup<Branch, BranchGroupType>> {
            
            let mut edge_hash: FullHashMap<EdgeIndex, f64> = FullHashMap::default();
        
            // aggregate edge usage of all selected columns
            patterns
                .iter()
                .filter(|(z, _)| *z > 0.0)
                .for_each(|(z, cid)| {
                    column_pool
                        .get_column(*cid)
                        .data
                        .path
                        .iter()
                        .for_each(|pe| {
                            *(edge_hash.entry(*pe).or_insert(0.0)) += *z;
                        })
                });


            // for now -> just find first one that is fractional
            let first_fractional = edge_hash.iter().filter(|v| v.1.fract() > 0.0).next();

            
            if let Some((edge, _value)) = first_fractional {
                // return single branch opportunity, with branches for the fix & forbid options
                vec![BranchGroup::new(
                    vec![
                        vec![Branch::ForceEdge(*edge, true)],
                        vec![Branch::ForceEdge(*edge, false)],
                    ],
                    BranchGroupType::Default,
                )]
            } else {
                // otherwise, we are integral!
                vec![]
            }
        }

        fn create_column_forcing_filter(
            &self,
            column: &generic_bnp::column_pool::Column<Column>,
            direction: bool,
        ) -> Branch {
            // return non-generic branch type
            Branch::ForceColumn(column.id, direction)
        }

        fn is_column_forcing_filter(&self, filter: &Branch) -> Option<ColumnId> {
            // link non-generic branch type
            match filter {
                Branch::ForceColumn(id, _) => Some(*id),
                _ => None,
            }
        }

        fn get_lds_settings(&self) -> &LDSSettings {
            &LDSSettings {
                columns_to_fix_each_iteration: 1,
                selection: LDSSelection::ClosestToOne,
                max_depth: 20,
                max_discrepancy: 20,
                early_termination_after_bound_leq: 0,
                do_only_lds: false,
            }
        }

        fn add_var_from_column(
            &self,
            col: &generic_bnp::column_pool::Column<Column>,
            master: &mut gurobi::Model,
            model_meta: &ModelMeta,
            _solver: &PricingSolver,
            integer: bool,
        ) -> gurobi::Var {

            // setup row: set constraints & coefficients
            let colconstrs: Vec<Constr> = vec![
                model_meta.convexity_constraint.clone(),
                model_meta.time_constraint.clone(),
            ];
            let colvals: Vec<f64> = vec![1.0, f64::from(col.data.time)];

            // add var from column with obj of column cost
            master
                .add_var(
                    &format!("col_{}", col.id.0),
                    if integer {
                        gurobi::Integer
                    } else {
                        gurobi::Continuous
                    },
                    f64::from(col.data.cost),
                    0.0,
                    gurobi::INFINITY,
                    &colconstrs,
                    &colvals,
                )
                .unwrap()
        }

        fn fix_columns_from_filters_in_model(
            &self,
            _solver: &PricingSolver,
            active_filters: &[Branch],
            master: &mut gurobi::Model,
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
    }

    impl ColumnPoolFilter<Column, Branch> for ColumnPool<Column, Branch> {
        fn filter_columns<'a>(&self, filters: &[Branch], columns: impl Iterator<Item=&'a generic_bnp::column_pool::Column<Column>>) -> impl Iterator<Item=&'a generic_bnp::column_pool::Column<Column>> {
            columns.filter(move |col| {
                for filter in filters {
                    match filter {

                        Branch::ForceColumn(filter_col, dir) => {
                            if *filter_col == col.id && *dir == false {
                                // skip column if explicitly removed
                                return  false;
                            }
                        }

                        Branch::ForceEdge(edge, dir) => {
                            if col.data.path.contains(edge) != *dir {
                                // skip column if it does not match filter's restriction;
                                return false;
                            }
                        }
                    }
                }

                return true;
            })
        }
    }

    #[test]
    fn run() {
        // setup problem

        // Time constrained shortest path problem, Ahuja et al., 1993, p. 599
        let mut problem_graph = StableDiGraph::new();

        let n1 = problem_graph.add_node(ProblemNode { });
        let n2 = problem_graph.add_node(ProblemNode { });
        let n3 = problem_graph.add_node(ProblemNode { });
        let n4 = problem_graph.add_node(ProblemNode { });
        let n5 = problem_graph.add_node(ProblemNode { });
        let n6 = problem_graph.add_node(ProblemNode { });

        problem_graph.add_edge(n1, n2, ProblemEdge { cost: 1, time: 10 });
        problem_graph.add_edge(n1, n3, ProblemEdge { cost: 10, time: 3 });

        problem_graph.add_edge(n2, n4, ProblemEdge { cost: 1, time: 1 });
        problem_graph.add_edge(n2, n5, ProblemEdge { cost: 2, time: 3 });

        problem_graph.add_edge(n3, n2, ProblemEdge { cost: 1, time: 2 });
        problem_graph.add_edge(n3, n4, ProblemEdge { cost: 5, time: 7 });
        problem_graph.add_edge(n3, n5, ProblemEdge { cost: 12, time: 3 });

        problem_graph.add_edge(n4, n5, ProblemEdge { cost: 10, time: 1 });
        problem_graph.add_edge(n4, n6, ProblemEdge { cost: 1, time: 7 });

        problem_graph.add_edge(n5, n6, ProblemEdge { cost: 2, time: 2 });

        let problem_instance = ShortestPathsProblem {
            graph: problem_graph,
            graph_start: n1,
            graph_end: n6,
            max_time: 14,
        };

        // create column pool and add expensive initialy dummy column

        let mut column_pool = ColumnPool::new();
        let dummy_column = Column {
            cost: 100,
            time: 0,
            path: vec![],
        };
        column_pool.add_column(dummy_column, ColumnTicket(0));

        // create solver instance

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
            ShortestPathsProblem,
            ShortestPathsMasterProblem,
        > = BranchAndPrice::new(problem_instance.clone());

        // run problem
        let ui = UI::new();
        let (best, results) = bnp.solve(&ui, 1, column_pool);

        // compare result to known optimal
        assert_eq!(best.ceil(), 13.0);
        println!("Results: {:?}", results);
    }
}
