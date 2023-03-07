use std::pin::Pin;

use crate::{epics, utils::misc::unfold_variable};
use futures::Stream;

pub enum Debug {}

pub struct DebugHandle {
    pub stats_reset: Pin<Box<dyn Stream<Item = ()> + Send>>,
}

impl Debug {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(epics: epics::Debug) -> DebugHandle {
        DebugHandle {
            stats_reset: Box::pin(unfold_variable(epics.stats_reset, |x| {
                if x != 0 {
                    Some(())
                } else {
                    None
                }
            })),
        }
    }
}
