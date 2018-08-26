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
* Chat
  * misc quickchat
  * they told me they would delete me if i lost
  * wrong name handling
* Play
  * brazil
