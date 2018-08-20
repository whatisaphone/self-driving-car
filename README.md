# Formula nOne

A bot that loses at Rocket League using [RLBot].

[RLBot]: http://www.rlbot.org/

## Development

### Prerequisites

* [Rust](https://www.rust-lang.org/)

### First-time setup

```sh
pre-commit install
```

The bot may always lose, but at the code will be pretty.

### Watch the bot play

```sh
cargo run -p play
```

### Run tests

The tests require a copy of Rocket League running (naturally). There are a lot
of moving parts, so be warned that this is a somewhat finicky endeavor.

1. Start BakkesMod.

2. Start Rocket League. BakkesMod should inject itself successfully.

3. Run the tests.

   ```sh
   cargo test -- --test-threads=1 
   ```

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
* Chat
  * misc quickchat
  * they told me they would delete me if i lost
  * wrong name handling
* Play
  * brazil
