# Formula nOne

A bot that loses at Rocket League using [RLBot].

## Development

### Prerequisites

* Windows

  RLBot only works on Windows, and we inherit this limitation.

* Install [Rust](https://www.rust-lang.org/).

* Install [pre-commit], and run this command:

  ```sh
  pre-commit install
  ```

  This will run tests/lints/etc before every commit. The bot may lose every
  game, but at least the code will be pretty!

* Obtain these files from [RLBot] and copy them into `target/debug` and `target/release`:
  * `RLBot_Core.dll`
  * `RLBot_Core_Interface.dll`
  * `RLBot_Injector.exe`

[pre-commit]: https://pre-commit.com/
[RLBot]: http://www.rlbot.org/

### Watch the bot play

```sh
cargo run -p play
```

### Run the bot with the RLBot framework

```sh
cargo build && python -c "from rlbot import runner; runner.main()"
```

### Run integration tests

The integration tests require a copy of Rocket League running (naturally). There
are a lot of moving parts, so be warned that this is a somewhat finicky
endeavor.

1.  Start BakkesMod.

2.  Start Rocket League. BakkesMod should inject itself successfully.

3.  Run the tests.

    ```sh
    cargo test -p brain -- --test-threads=1 integration_tests
    ```

    To run an individual test, you can replace `integration_tests` with a
    pattern that matches the name of the test.

## Handy Links

* [RLBot wiki – Useful Game Values](https://github.com/RLBot/RLBot/wiki/Useful-Game-Values)
* [Vehicle specifications](https://www.reddit.com/r/RocketLeague/comments/7fotyx/vehicle_specifications_v139_hitboxes_handling/)
* [Sam Mish's RL physics notes](https://samuelpmish.github.io/notes/RocketLeague/)
  * [The author's bot](https://github.com/samuelpmish/Lobot)
* [BakkesMod commands](http://bakkesmod.wikia.com/wiki/Configuration)

## Ideas

* Names
  * the bottmer peak
  * formula none
  * bender
  * RoboCar
  * rosie the rocket maid
  * Rock-E
* Chat
  * misc quickchat
  * they told me they would delete me if i lost
  * wrong name handling
* Play
  * brazil

## Roadmap

### Next

- make it work when orange
- workaround for root capture interruption
- add the offense swing-around stuff (when no good shot angle)
- cache simulations to save on cpu
- stop JumpShot freezing

### Backlog

- shoot: choose bounce shot or ground shot depending on whether ball is bouncing
- [chip recovery](https://pastebin.com/XtFL5JzV)
- finish chip bvh
- generic long-range pathfinding
  - if far away, waiting stationary, and turned the wrong way, fix yaw before sitting still
  - if in TIME CRUNCH MODE, just blitz to destination
  - otherwise: weave for pennies, opportunistic demos, etc
- if inside a goal, never drive in a path that overlaps inner walls of goal
- fix whiffs when retreating redirects to corner
- [half flip](https://discordapp.com/channels/348658686962696195/348661571297214465/489479593632464901)
