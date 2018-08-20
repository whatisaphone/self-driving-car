use std::process::Command;

pub fn inject_dll() -> Result<InjectorCode, ()> {
    let code = Command::new("RLBot_Injector")
        .arg("hidden")
        .status()
        .map_err(|_| ())?
        .code()
        .ok_or(())?;

    if code == InjectorCode::InjectionSuccessful as i32 {
        Ok(InjectorCode::InjectionSuccessful)
    } else if code == InjectorCode::RLBotDLLAlreadyInjected as i32 {
        Ok(InjectorCode::RLBotDLLAlreadyInjected)
    } else {
        Err(())
    }
}

#[allow(dead_code)]
#[derive(Eq, PartialEq)]
pub enum InjectorCode {
    InjectionSuccessful = 0,
    InjectionFailed = 1,
    MultipleRocketLeagueProcessesFound = 2,
    RLBotDLLAlreadyInjected = 3,
    RLBotDLLNotFound = 4,
    MultipleRLBotDLLFilesFound = 5,
}
