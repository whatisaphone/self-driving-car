extern crate bakkesmod;
extern crate csv;
extern crate rlbot;

use std::error::Error;

mod cli;
mod collector;

fn main() -> Result<(), Box<Error>> {
    cli::main()
}
