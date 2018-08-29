# rlbot

This library exposes Rust bindings to RLBot's [RLBot_Core_Interface.dll].

[RLBot_Core_Interface.dll]: https://github.com/RLBot/RLBot/tree/master/src/main/cpp/RLBotInterface

## How to generate bindings

Bindings are generated with [rust-bindgen]. Those docs are required reading.

[rust-bindgen]: https://rust-lang-nursery.github.io/rust-bindgen/

After bindgen and its prerequisites are installed and working, run this short
command:

```sh
rlbot=<path-to-rlbot>
bindgen \
    "$rlbot"/src/main/cpp/RLBotInterface/RLBotInterface/Interface.hpp \
    -o src/ffi.rs \
    --disable-name-namespacing \
    --no-layout-tests \
    --default-enum-style rust \
    --with-derive-default \
    --raw-line '#![allow(dead_code, non_snake_case, non_camel_case_types)]' \
    --whitelist-function Interface::IsInitialized \
    --whitelist-function GameFunctions::SetGameState \
    --whitelist-function GameFunctions::StartMatch \
    --whitelist-function GameFunctions::UpdateFieldInfo \
    --whitelist-function GameFunctions::UpdateLiveDataPacket \
    --whitelist-function GameFunctions::SendQuickChat \
    --whitelist-function GameFunctions::SendChat \
    --whitelist-function GameFunctions::UpdatePlayerInput \
    --whitelist-function RenderFunctions::RenderGroup \
    -- \
    -I "$rlbot"/src/main/cpp/RLBotInterface/RLBotMessages
```

It should output errors in white text. Modify RLBot's source to fix the errors
manually. For any problematic references to boost, it will be easier to just
completely purge any mention of boost. Keep running the above command and fixing
errors (good times!). Once you're done, it will run successfully and output more
errors, but in red text this time. As long as the errors are in red, that means
it worked!

Now open the resulting file (`src/ffi.rs`) and remove all the `extern "C" pub
fn` declarations from the end. Since the functions are called using this crate's
`dll` module, there's no sense exposing the non-working ones—it would just lead
to confusion.
