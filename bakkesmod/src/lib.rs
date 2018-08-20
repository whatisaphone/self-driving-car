extern crate crossbeam_channel;
extern crate websocket;

use std::thread;
use websocket::client::sync::Client;
use websocket::client::ClientBuilder;
use websocket::{Message, OwnedMessage};

pub struct BakkesMod {
    tx: Option<crossbeam_channel::Sender<String>>,
    io_thread: Option<thread::JoinHandle<()>>,
}

impl BakkesMod {
    pub fn connect() -> Result<BakkesMod, ()> {
        let client = ClientBuilder::new("ws://127.0.0.1:9002")
            .map_err(|_| ())?
            .connect_insecure()
            .map_err(|_| ())?;

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
    pub fn send<S: Into<String>>(&self, msg: S) {
        self.tx.as_ref().unwrap().send(msg.into());
    }
}

fn run<C>(mut client: Client<C>, rx: crossbeam_channel::Receiver<String>)
where
    C: std::io::Read + std::io::Write,
{
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
