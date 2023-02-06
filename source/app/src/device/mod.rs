mod adc;
mod dac;

use crate::{
    channel::Channel,
    config,
    epics::Epics,
    proto::{self, AppMsg, McuMsg, McuMsgRef},
};
use async_std::{sync::Arc, task::sleep};
use flatty::prelude::*;
use flatty_io::{AsyncReader as MsgReader, AsyncWriter as MsgWriter};
use futures::{
    executor::ThreadPool,
    future::{join_all, FutureExt},
};

use adc::{Adc, AdcHandle};
use dac::{Dac, DacHandle};

pub struct Device {
    reader: MsgReader<McuMsg, Channel>,
    writer: MsgWriter<AppMsg, Channel>,
    dacs: [Dac; config::DAC_COUNT],
    adcs: [Adc; config::ADC_COUNT],
}

struct MsgDispatcher {
    channel: MsgReader<McuMsg, Channel>,
    dacs: [DacHandle; config::DAC_COUNT],
    adcs: [AdcHandle; config::ADC_COUNT],
}

struct Keepalive {
    channel: MsgWriter<AppMsg, Channel>,
}

impl Device {
    pub fn new(channel: Channel, epics: Epics) -> Self {
        let reader = MsgReader::<McuMsg, _>::new(channel.clone(), config::MAX_MCU_MSG_LEN);
        let writer = MsgWriter::<AppMsg, _>::new(channel, config::MAX_APP_MSG_LEN);
        Self {
            dacs: epics.analog_outputs.map(|epics| Dac {
                channel: writer.clone(),
                epics,
            }),
            adcs: epics.analog_inputs.map(|epics| Adc { epics }),
            reader,
            writer,
        }
    }

    pub async fn run(self, exec: Arc<ThreadPool>) {
        let (dac_loops, dac_handles): (Vec<_>, Vec<_>) = self
            .dacs
            .into_iter()
            .map(|dac| dac.run(exec.clone()))
            .unzip();
        let (adc_loops, adc_handles): (Vec<_>, Vec<_>) =
            self.adcs.into_iter().map(|adc| adc.run()).unzip();

        let dispatcher = MsgDispatcher {
            channel: self.reader,
            dacs: dac_handles.try_into().ok().unwrap(),
            adcs: adc_handles.try_into().ok().unwrap(),
        };
        let keepalive = Keepalive {
            channel: self.writer,
        };

        exec.spawn_ok(join_all(dac_loops).map(|_| ()));
        exec.spawn_ok(join_all(adc_loops).map(|_| ()));
        exec.spawn_ok(dispatcher.run());
        exec.spawn_ok(keepalive.run());
    }
}

impl MsgDispatcher {
    async fn run(self) {
        let mut channel = self.channel;
        let mut adcs = self.adcs;
        loop {
            let msg = channel.read_message().await.unwrap();
            //log::info!("read_msg: {:?}", msg.tag());
            match msg.as_ref() {
                McuMsgRef::Empty(_) => (),
                McuMsgRef::DinUpdate(_) => unimplemented!(),
                McuMsgRef::DacRequest(req) => self.dacs[0].request(req.count.to_native() as usize),
                McuMsgRef::AdcData(data) => {
                    for (index, adc) in adcs.iter_mut().enumerate() {
                        adc.push(data.points_arrays.iter().map(|a| a[index].to_native()))
                            .await
                    }
                }
                McuMsgRef::Error(error) => {
                    panic!(
                        "Error {}: {}",
                        error.code,
                        String::from_utf8_lossy(error.message.as_slice())
                    )
                }
                McuMsgRef::Debug(debug) => {
                    println!(
                        "Debug: {}",
                        String::from_utf8_lossy(debug.message.as_slice())
                    )
                }
            }
        }
    }
}

impl Keepalive {
    async fn run(mut self) {
        {
            self.channel
                .new_message()
                .emplace(proto::AppMsgInitConnect(proto::AppMsgConnect {}))
                .unwrap()
                .write()
                .await
                .unwrap();
        }
        loop {
            sleep(config::KEEP_ALIVE_PERIOD).await;
            self.channel
                .new_message()
                .emplace(proto::AppMsgInitKeepAlive(proto::AppMsgKeepAlive {}))
                .unwrap()
                .write()
                .await
                .unwrap();
        }
    }
}
