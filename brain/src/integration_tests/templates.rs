// `from_recorded_row` and `preview_recording` are both marked as deprecated so
// I don't accidentally copy/paste them and forget to commit the actual harness.
#![allow(deprecated)]

use crate::integration_tests::helpers::{TestRunner, TestScenario};
use nalgebra::Point3;

#[test]
#[ignore]
fn scenario_template() {
    let _test = TestRunner::new()
        .scenario(TestScenario {
            enemy_loc: Point3::new(6000.0, 6000.0, 0.0),
            ..TestScenario::from_recorded_row("../logs/play.csv", 10.0)
        })
        .starting_boost(25.0)
        .soccar()
        .run_for_millis(7000);
    unimplemented!();
}

#[test]
#[ignore]
fn recording_template() {
    let _test = TestRunner::new()
        .preview_recording("../logs/play.csv", 100.0, 2.0, 5.0)
        .starting_boost(25.0)
        .soccar()
        .run_for_millis(7000);
    unimplemented!();
}
