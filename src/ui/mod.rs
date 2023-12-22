use std::fmt::{Display, Formatter, Write as FmtWrite};


use std::io::Write;
use std::time::{Duration, Instant};
use crate::misc::HashSet;

use std::sync::mpsc::{channel, Sender};
use std::thread::ThreadId;
use std::io::BufWriter;

use console::{pad_str_with,Alignment,style,pad_str};
use std::fs::OpenOptions;
use crate::branch::SpecialType;

/// Struct to hold the UI
/// Particulary the receiver channel
pub struct UI {
    sender : UISender
}


#[derive(Clone)]
pub struct UISender {
    sender : Sender<UIMessage>
}

// needed in stable rust, unstable auto detects
unsafe impl Send for UISender {}
unsafe impl Sync for UISender {}


impl UISender {
    /// Send typed UIMessage to internal channel
    pub fn send(&self, user_msg : UIUserMessage) {
            #[cfg(not(feature = "disable_ui"))]
            self.sender.send(
                UIMessage {
                    thread_id: std::thread::current().id(),
                    message: user_msg
                }
            ).unwrap();
    }

}

impl UI {


    pub fn get_sender(&self) -> UISender {
        self.sender.clone()
    }
    pub fn new() -> Self {

        let (sender,receiver) = channel();


        #[cfg(not(feature = "disable_ui"))]
        std::thread::spawn(move|| {


            #[cfg(feature = "branch-graphviz")]
                let mut graphviz_branch_file = OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open("/tmp/branching_tree.dot")
                .unwrap();

            #[cfg(feature = "branch-graphviz")]
            graphviz_branch_file.write(b"digraph {\n").unwrap();






            #[cfg(not(feature = "locked_out"))]
                let stdout = std::io::stdout();
            #[cfg(feature = "locked_out")]
                let stdout = std::io::stdout().lock();

            #[cfg(not(feature = "buffered_out"))]
                let mut buffered_out = stdout;
            #[cfg(feature = "buffered_out")]
                let mut buffered_out = BufWriter::with_capacity(512,stdout);

            let start_time = Instant::now();

            let mut total_pricing_runtime = 0.0;
            let mut total_lp_runtime = 0.0;

            let mut seen_threads_this_phase = HashSet::default();

            let mut branch_start_order = 0;
            let mut branch_finish_order = 0;

            loop {
                match receiver.recv().unwrap() {
                    UIMessage { thread_id, message : UIUserMessage::BranchCut { branch_id} } => {
                        #[cfg(feature = "branch-graphviz")]
                        graphviz_branch_file.write(format!("{} [style=\"filled\" fillcolor=\"lightyellow\"] ", branch_id
                        ).as_bytes()).unwrap();
                    }
                    UIMessage { thread_id, message :  UIUserMessage::TimeLimitReached }  =>   writeln!(&mut buffered_out,"{}",style("Time Limit Reached").yellow().bold()).unwrap(),
                    UIMessage { thread_id, message :  UIUserMessage::Log(msg) }  =>   writeln!(&mut buffered_out,"[{:?}] {:>6.2}  {}",  thread_id, start_time.elapsed().as_secs_f64(),msg).unwrap(),
                    UIMessage { thread_id, message : UIUserMessage::LogS( msg)  }=>   writeln!(&mut buffered_out,"[{:?}] {:>6.2}  {}", thread_id, start_time.elapsed().as_secs_f64(),msg).unwrap(),
                    UIMessage { thread_id, message : UIUserMessage::StartWorker } => {
                        seen_threads_this_phase.insert(thread_id);
                        writeln!(&mut buffered_out,"{}",style(format!("Worker #{} activated", seen_threads_this_phase.len())).yellow().bold()).unwrap();
                    }

                    UIMessage { thread_id, message : UIUserMessage::StartPhase(title, level ) } => {

                        // reset seen threads
                        seen_threads_this_phase = HashSet::default();

                        writeln!(&mut buffered_out,"{}",pad_str_with(&format!("{:?}", thread_id),30,Alignment::Center, None, '⎯')).unwrap();
                        writeln!(&mut buffered_out,"{}", style(pad_str(title, 30, Alignment::Center,None)).green()).unwrap();
                        writeln!(&mut buffered_out,"{}","⎯".repeat(30)).unwrap();
                        buffered_out.flush().unwrap();
                    },
                    UIMessage { thread_id, message : UIUserMessage::ExitUi { root_node: root_node_duration, lds_root_node_duration} } => {

                        let final_time = start_time.elapsed().as_secs_f64();
                        writeln!(&mut buffered_out,"{}",pad_str_with("Statistics",30,Alignment::Center, None, '⎯')).unwrap();
                        writeln!(&mut buffered_out,"total_lp_time: {:>8.2}s / total_pricing_time: {:>8.2}s", total_lp_runtime, total_pricing_runtime).unwrap();

                        let num_threads = seen_threads_this_phase.len();


                        let overhead =    (final_time  * num_threads  as f64) - total_lp_runtime -  total_pricing_runtime
                            // subtract root node as there only one thread is busy
                            - root_node_duration.map(|rn| rn.as_secs_f64() * (num_threads  - 1) as f64).unwrap_or(0.0) - lds_root_node_duration.map(|rn| rn.as_secs_f64() * (num_threads  - 1) as f64).unwrap_or(0.0);


                        writeln!(&mut buffered_out,"'overhead': {:>8.2}s  ({:>3.1}%) (in {} threads)", overhead, overhead/(final_time *  num_threads as f64) * 100.0, num_threads).unwrap();
                        if num_threads > 1 {
                            writeln!(&mut buffered_out, "{}", style(" in multithreaded env: idle time during root node is not included!").dim()).unwrap();
                        }
                        writeln!(&mut buffered_out,"{:>3.1}% spent in pricing vs lp", total_pricing_runtime / (total_lp_runtime + total_pricing_runtime) * 100.0).unwrap();
                        writeln!(&mut buffered_out,"{}","⎯".repeat(30)).unwrap();


                        buffered_out.flush().unwrap();

                        break
                    },

                    UIMessage { thread_id, message : UIUserMessage::BranchStart(branch_state) } => {
                        writeln!(&mut buffered_out,"[{t:?}] {time:>6.2} started   branch {branch}",
                                 t=thread_id,
                                 time = start_time.elapsed().as_secs_f64(),
                                 branch=branch_state
                        ).unwrap();
                        buffered_out.flush().unwrap();

                        branch_start_order += 1;

                        #[cfg(feature = "branch-graphviz")]
                        graphviz_branch_file.write(format!("{id} [shape=\"plaintext\" label=<<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\"><TR><TD>{id}</TD><TD>s#{branch_start_order}</TD><TD>{before_obj:.2} → ?</TD><TD>{best_obj:2}</TD></TR><TR><TD>f:{num_filters}</TD><TD COLSPAN=\"3\">{last_filter}</TD></TR><TR><TD COLSPAN=\"4\">{special}</TD></TR></TABLE>>];\n {parent} -> {id};\n",
                                                           id=branch_state.branch_id,
                                                            branch_start_order=branch_start_order,
                                                           parent = branch_state.branch_parent,
                                                           before_obj = branch_state.before_obj,
                                                           best_obj = branch_state.best_obj,
                                                           last_filter = branch_state.last_filter,
                                                           num_filters = branch_state.num_filters,
                                                           special = branch_state.special
                        ).as_bytes()).unwrap();

                    },
                    UIMessage { thread_id, message : UIUserMessage::BranchFinish(branch_state) } => {

                        writeln!(&mut buffered_out,"[{t:?}] {time:>6.2} completed branch {branch}",
                                 t=thread_id,
                                 time = start_time.elapsed().as_secs_f64(),
                                 branch=branch_state
                        ).unwrap();

                        branch_finish_order += 1;


                        #[cfg(feature = "branch-graphviz")]
                        graphviz_branch_file.write(format!("{id} [shape=\"plaintext\" label=<<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\"><TR><TD>{id}</TD><TD>#{branch_finish_order}</TD><TD>{before_obj:.5} → {after_obj:.5}</TD><TD>{best_obj:.2}</TD></TR><TR><TD>f:{num_filters}</TD><TD COLSPAN=\"3\">{last_filter}</TD></TR><TR><TD COLSPAN=\"4\">{special}</TD></TR></TABLE>>];\n",
                                           id=branch_state.branch_id,
                                                            branch_finish_order = branch_finish_order,
                                           before_obj = branch_state.before_obj,
                                                           after_obj = branch_state.after_obj.unwrap(),
                                           last_filter = branch_state.last_filter,
                                                           best_obj = branch_state.best_obj,
                                                            num_filters = branch_state.num_filters,
                                           special = branch_state.special
                        ).as_bytes()).unwrap();

                    }

                    UIMessage { thread_id, message : UIUserMessage::LPSolveIterationFinish(state) } => {
                        total_lp_runtime += state.lp_runtime;

                        writeln!(&mut buffered_out, "{}", style(format!("[{t:?}] {time:>6.2} lp iteration {state}",
                                 t=thread_id,
                                 time = start_time.elapsed().as_secs_f64(),
                                 state=state
                        )).dim()).unwrap()

                    }
                    UIMessage { thread_id, message : UIUserMessage::PricingProblemFinish(state) } => {

                        // always needed for statistics
                        total_pricing_runtime += state.runtime;


                        /* noisy*/
                        writeln!(&mut buffered_out, "{}", style(format!("[{t:?}] {time:>6.2} pricing iteration {state}",
                                 t=thread_id,
                                 time = start_time.elapsed().as_secs_f64(),
                                 state=state
                        )).dim()).unwrap();

                    }

                    UIMessage { thread_id, message : UIUserMessage::NewBest{branch_id, obj} } => {

                        writeln!(&mut buffered_out,"[{:?}] {:>6.2}  {} {}", thread_id,start_time.elapsed().as_secs_f64(),style("Has new best:").black().on_green().bold(),  style(obj.to_string()).bold()).unwrap();
                        buffered_out.flush().unwrap();

                        #[cfg(feature = "branch-graphviz")]
                        graphviz_branch_file.write(format!("{id} [style=\"filled\" fillcolor=\"lightgreen\"];\n",
                                                           id=branch_id

                        ).as_bytes()).unwrap();
                    }



                }
            }

            buffered_out.flush().unwrap();
        });


        Self {
            sender : UISender { sender }
        }

    }
}

#[derive(Clone)]
pub struct BranchUIState {
    pub branch_id : u32,
    pub branch_parent : u32,
    pub num_columns : usize,
    pub current_open : usize,
    pub num_filters : usize,
    pub last_filter : String,
    pub special : String,
    pub before_obj : f64,
    pub after_obj : Option<f64>, // not set in branch start
    pub best_obj : f64
}

impl Display for BranchUIState {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "b{id}  parent=<{parent}> open=<{current_open}> cols=<{num_columns}> filter=<{num_filters}> last=<{last_filter}> obj_before=<{before_obj}> obj_now=<{after_obj}> obj*=<{best_obj}>",
        id = self.branch_id, current_open = self.current_open, parent  = self.branch_parent, num_columns = self.num_columns, before_obj = self.before_obj,
            after_obj = self.after_obj.map(|v| format!("{}", v)).unwrap_or_else(|| "-".to_string()), best_obj = self.best_obj, num_filters = self.num_filters, last_filter = self.last_filter
        )
    }
}

#[derive(Clone)]
/// Holds all state updates that can influence the UI
pub enum UIUserMessage {
    LogS( &'static str),
    Log( String),
    TimeLimitReached,
    StartPhase(&'static str, u8),
    ExitUi { root_node : Option<Duration>, lds_root_node_duration : Option<Duration>},
    StartWorker,

    BranchStart(BranchUIState),
    BranchFinish(BranchUIState),
    BranchCut { branch_id : u32},

    LPSolveIterationFinish(LPSolveIterationUIState),
    PricingProblemFinish(PricingProblemUIState),

    NewBest {    obj : f64,   branch_id : u32}
}

#[derive(Clone)]
pub struct PricingProblemUIState {
    pub runtime : f64,
    pub num_columns : usize,
    pub dual_bound : f64
}

impl Display for PricingProblemUIState {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "pricing_runtime=<{:>5.2}> num_cols=<{}> dual_bound=<{}>",  self.runtime , self.num_columns, self.dual_bound )
    }
}

#[derive(Clone)]
pub struct NewBestUIState {

}


#[derive(Clone)]
pub struct LPSolveIterationUIState {
    pub obj : f64,
    pub best_dual_bound : Option<f64>,
    pub lp_runtime : f64,
    pub num_columns : usize,
    pub num_total_pool_columns : usize,
    pub previous_pricing_runtime : f64
}

impl Display for LPSolveIterationUIState {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "obj=<{:>10.8}> bound=<{}>: lp_runtime=<{:>5.2}> prev_pricing_runtime=<{:>5.2}> cols=<{}/{}>", self.obj, self.best_dual_bound.map(|db| format!("{:>10.8}", db)).unwrap_or("-".to_string()), self.lp_runtime, self.previous_pricing_runtime, self.num_columns, self.num_total_pool_columns )
    }
}


#[derive(Clone)]
pub struct  UIMessage {
   pub thread_id : ThreadId,
    pub message : UIUserMessage
}