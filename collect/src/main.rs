extern crate bakkesmod;
extern crate csv;
extern crate ratelimit;
extern crate rlbot;

pub use collector::Collector;

mod cli;
mod collector;

fn main() -> Result<(), ()> {
    cli::main()
}
