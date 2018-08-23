use ffi;

pub fn match_settings_1v1() -> ffi::MatchSettings {
    let mut match_settings = ffi::MatchSettings {
        NumPlayers: 2,
        SkipReplays: true,
        InstantStart: true,
        MutatorSettings: ffi::MutatorSettings {
            MatchLength: ffi::MatchLength::Unlimited,
            ..Default::default()
        },
        ..Default::default()
    };

    match_settings.PlayerConfiguration[0].Bot = true;
    match_settings.PlayerConfiguration[0].RLBotControlled = true;
    match_settings.PlayerConfiguration[0].set_name("Blue Car");

    match_settings.PlayerConfiguration[1].Bot = true;
    match_settings.PlayerConfiguration[1].BotSkill = 1.0;
    match_settings.PlayerConfiguration[1].set_name("Orange Car");
    match_settings.PlayerConfiguration[1].Team = 1;

    match_settings
}
