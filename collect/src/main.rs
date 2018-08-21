extern crate bakkesmod;
extern crate csv;
extern crate rlbot;

mod cli;
mod collector;

fn main() -> Result<(), ()> {
    cli::main()
}
