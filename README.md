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

The integration tests require a copy of Rocket League running (naturally).

```sh
cargo test -p brain -- --test-threads=1 integration_tests
```

To run an individual test, you can replace `integration_tests` with a pattern
that matches the name of the test.

### Profiling

Adapt this command to your needs:

```sh
cd brain
vsperf="C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\Team Tools\Performance Tools\VSPerf.exe"
RUST_BACKTRACE=1 \
"$vsperf" \
    -launch:../target/debug/deps/brain-b7338d0ad12bc0e2.exe \
    -args:"--test-threads=1 recording_template --ignored"
```

### Build a tournament package

```sh
scripts/tourney.sh
```

This will package up a release build in `target/tournament/`. It will also
modify the local `rlbot.cfg` for quick testing. You can test the build by
starting Rocket League and then running:

```sh
( cd target/tournament && python -c "from rlbot import runner; runner.main()" )
```

Once you know it works, zip up the directory and send it to the tournament
organizer!

## Handy Links

### Rocket League

* [RLBot wiki – Useful Game Values](https://github.com/RLBot/RLBot/wiki/Useful-Game-Values)
* [Vehicle specifications](https://www.reddit.com/r/RocketLeague/comments/7fotyx/vehicle_specifications_v139_hitboxes_handling/)
* [Sam Mish's RL physics notes](https://samuelpmish.github.io/notes/RocketLeague/)
  * [RLUtilities](https://github.com/samuelpmish/RLUtilities)
* [DomNomNom's bot](https://github.com/DomNomNom/RocketBot)

### Math

* [Biarc interpolation](http://www.ryanjuckett.com/programming/biarc-interpolation/)
* [Rotation converter](https://www.andre-gaschler.com/rotationconverter/)
* [Parabola trajectory math](http://hyperphysics.phy-astr.gsu.edu/hbase/traj.html)

## Ideas

* Names
  * the bottmer peak
  * formula none
  * bender
  * RoboCar
  * rosie the rocket maid
  * Rock-E
  * self-driving car
* Chat
  * misc quickchat
  * they told me they would delete me if i lost
  * wrong name handling
* Play
  * brazil

## Roadmap

### Next

- GetToFlatGround: require stricter roof pointing before jumping (test case
  fast_retreating_save)
- aiming model for most power
- AnticipatingDrive with ability to throttle without boosting
- aerials v2.0
- model who is controlling the ball?
- TepidHit should sent it to the corner if things are dangerous (e.g., we can't
  roll it up the wall and we're facing our own goal and the enemy is closing in)

### Backlog

- driving on walls
- add the offense swing-around stuff (when no good shot angle)
- backwards tepid hits should not dodge? (maybe all tepid hits)
- BounceShot (and related) should verify the hit happened, and Action::Abort if
  not
- don't jump directly after skidding and go the wrong way (probably test yaw vs
  velocity angle and see if match)
- QuickJumpAndDodge should allow choosing the angle at apex instead of only
  ahead of time
- Brake should not get stuck in a loop when moving backwards
- [chip recovery](https://pastebin.com/XtFL5JzV)
- generic long-range pathfinding
  - if far away, waiting stationary, and turned the wrong way, fix yaw before
    sitting still
  - if in TIME CRUNCH MODE, just blitz to destination
  - otherwise: weave for pennies, opportunistic demos, etc
- fix whiffs when retreating redirects to corner
- [half flip](https://discordapp.com/channels/348658686962696195/348661571297214465/489479593632464901)
