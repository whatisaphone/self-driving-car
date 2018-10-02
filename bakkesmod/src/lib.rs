#![cfg_attr(feature = "strict", deny(warnings))]

extern crate crossbeam_channel;
extern crate websocket;

use std::{
    io::{Read, Write},
    thread,
};
use websocket::{
    client::{sync::Client, ClientBuilder},
    Message, OwnedMessage, WebSocketError,
};

pub struct BakkesMod {
    tx: Option<crossbeam_channel::Sender<String>>,
    io_thread: Option<thread::JoinHandle<()>>,
}

impl BakkesMod {
    pub fn connect() -> Result<BakkesMod, WebSocketError> {
        let client = ClientBuilder::new("ws://127.0.0.1:9002")
            .unwrap()
            .connect_insecure()?;

        let (tx, rx) = crossbeam_channel::unbounded();

        Ok(BakkesMod {
            tx: Some(tx),
            io_thread: Some(thread::spawn(|| run(client, rx))),
        })
    }
}

impl Drop for BakkesMod {
    fn drop(&mut self) {
        drop(self.tx.take());
        self.io_thread.take().unwrap().join().ok().unwrap();
    }
}

impl BakkesMod {
    pub fn send(&self, msg: impl Into<String>) {
        self.tx.as_ref().unwrap().send(msg.into());
    }
}

fn run(mut client: Client<impl Read + Write>, rx: crossbeam_channel::Receiver<String>) {
    client
        .send_message(&Message::text("rcon_password password"))
        .unwrap();
    match client.recv_message() {
        Ok(OwnedMessage::Text(ref s)) if s == "authyes" => (),
        _ => panic!("Could not authenticate with BakkesMod"),
    }

    loop {
        match rx.recv() {
            Some(msg) => client.send_message(&Message::text(msg)).unwrap(),
            None => break,
        }
    }
}
