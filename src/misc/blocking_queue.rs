/////////////////////////////////////////////////////////////
// rust_blocking_queue::lib.rs - BlockingQueue             //
//                                                         //
// Jim Fawcett, https://JimFawcett.github.io, 19 May 2020  //
/////////////////////////////////////////////////////////////
/*
   This is a BlockingQueue abstraction.  To be shared between
   threads, without using unsafe code, any abstraction must
   be composed only of Mutexes and Condvars or a struct or
   tuple with only those members.
   That means that the blocking queue must hold its native
   queue in a Mutex, as shown below.

   There is another alternative, based on Rust channels, which
   are essentially blocking queues.
*/
#![allow(dead_code)]

use binary_heap_plus::{BinaryHeap};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::*;
use crate::branch::Branch;
use crate::{IBranchFilter, UserDualStore};
use compare::Compare;




/// Thread-safe queue that blocks de_q on empty
pub struct BlockingQueue<T, C, DualStore>
where
    T: IBranchFilter,
    DualStore : UserDualStore,
C : Compare<Branch<T,DualStore>>  + Clone

{
    queue: Mutex<BinaryHeap<Branch<T,DualStore>, C>>,
    did_swap_priority: AtomicBool,
    flushing : AtomicBool,
    in_progress: AtomicU8,
    condvar: Condvar,
    compare_bound : C,
    compare_without_bound : C
}

impl<T: IBranchFilter, C : Compare<Branch<T,DualStore >> + Clone, DualStore : UserDualStore> BlockingQueue<T , C, DualStore>

{
    /// Create empty blocking queue
    pub fn new(compare_bound :  C, compare_without_bound : C) -> Self {


        let heap  = BinaryHeap::from_vec_cmp(vec![] as Vec<Branch<T, DualStore>>, compare_without_bound.clone());



        Self {
            queue: Mutex::new(heap),
            in_progress: AtomicU8::new(0),
            condvar: Condvar::new(),
            did_swap_priority: AtomicBool::new(false),
            compare_bound,
            compare_without_bound,
            flushing : false.into()

        }
    }

    pub fn now_has_bound(&self) {

        // only swap if not previously
        if !self.did_swap_priority.load(self::Ordering::Relaxed) {
            self.did_swap_priority.store(true, self::Ordering::SeqCst);
            let mut lq = self.queue.lock().unwrap();
            lq.replace_cmp(self.compare_bound.clone());
        }
    }

    /// push input on back of queue
    /// - unrecoverable if lock fails so just unwrap
    pub fn add_job(&self, t: Branch<T, DualStore>) {

        if self.flushing.load(Ordering::SeqCst) { return;}
        let mut lq = self.queue.lock().unwrap();
        lq.push(t);
        self.condvar.notify_one();
    }
    /// pop element from front of queue
    /// - unrecoverable if lock fails so just unwrap
    /// - same for condition variable
    pub fn get_job(&self) -> Option<Branch<T, DualStore>> {
        let mut lq = self.queue.lock().unwrap();
        // if the queue is empty we wait, but only if there are no running jobs
        while lq.len() == 0 && self.in_progress.load(Ordering::SeqCst) > 0 && !self.flushing.load(Ordering::SeqCst) {
            lq = self.condvar.wait(lq).unwrap();
        }

        if self.flushing.load(Ordering::SeqCst) { return None}

        let job = lq.pop();
        if job.is_some() {
            self.in_progress.fetch_add(1, Ordering::SeqCst);
        }
        job
    }

    pub fn job_done(&self) {
        self.in_progress.fetch_sub(1, Ordering::SeqCst);
        self.condvar.notify_all();
    }

    pub fn flush_and_terminate(&self) {

        self.flushing.store(true, Ordering::SeqCst);
        self.in_progress.store(0, Ordering::SeqCst);
        self.condvar.notify_all();
    }

    pub fn copy_of_queue(&self) -> Vec<Branch<T, DualStore>> {
        self.queue.lock().unwrap().clone().into_vec()
    }


    pub fn lowest_bound(&self) -> Option<f64> {
        self.queue.lock().unwrap().iter().map(|i| i.old_obj_bound).min_by(|a, b| a.partial_cmp(b).unwrap())
    }


    /// return number of elements in queue
    pub fn len(&self) -> usize {
        self.queue.lock().unwrap().len()
    }
}
