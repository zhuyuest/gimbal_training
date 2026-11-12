#[path = "../src/config.rs"]
mod config;
#[path = "../src/control.rs"]
mod control;
use control::*;

#[test]
fn queued_feedback_age_and_old_frames_do_not_refresh() {
    let mut c = Control::new(rig());
    c.read_feedback_aged(0x205, &feedback(10, 0, 0), 19);
    assert_eq!(c.yaw.feedback_age, 19);
    c.read_feedback_aged(0x205, &feedback(11, 0, 0), 20);
    assert_eq!(c.yaw.motor.encoder_raw, 10);
    c.read_feedback_aged(0x205, &feedback(12, 0, 0), 0);
    c.read_feedback_aged(0x205, &feedback(13, 0, 0), 5);
    assert_eq!(c.yaw.motor.encoder_raw, 12);
    assert_eq!(c.yaw.feedback_age, 0);
}

#[test]
fn no_enable_from_stale_feedback_or_non_neutral_recovery() {
    let mut c = Control::new(rig());
    tick(&mut c, Some(Some(rc(2, 3, 0, 0))), None, None);
    tick(&mut c, Some(Some(rc(3, 3, 0, 0))), None, None);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
    let mut c = running();
    c.yaw.target_angle_rad = 0.5;
    for _ in 0..500 {
        step(&mut c, rc(3, 3, 0, 0), 0, 0);
    }
    step(&mut c, rc(3, 2, 0, 0), 0, 0);
    step(&mut c, rc(3, 1, 660, 0), 0, 0);
    step(&mut c, rc(3, 1, 0, 0), 0, 0);
    assert_eq!(c.state, State::Error);
    step(&mut c, rc(3, 2, 0, 0), 0, 0);
    tick(&mut c, Some(None), None, None);
    step(&mut c, rc(3, 1, 0, 0), 0, 0);
    assert_eq!(c.state, State::Error);
}

#[test]
fn pid_integral_bound_independent_history_and_prepare_limits() {
    let mut a = Pid::new(0.0, 10.0, 0.0, 2.0, 100.0);
    let mut b = a;
    for _ in 0..1000 {
        a.calculate(1.0, 0.0, DT);
    }
    assert_eq!(a.integral, 2.0);
    assert_eq!(b.calculate(0.0, 0.0, DT), 0.0);
    let mut c = Control::new(rig());
    step(&mut c, rc(2, 3, 0, 0), 1000, 1000);
    step(&mut c, rc(3, 3, 0, 0), 1000, 1000);
    step(&mut c, rc(3, 3, 0, 0), 1000, 1000);
    assert_eq!(c.state, State::Prepare);
    assert_eq!(c.yaw.target_rpm, -10.0);
    assert_eq!(c.yaw.motor.command_voltage_raw, -500.0);
}
fn rig() -> RigConfig {
    let mut c = config::CONFIG;
    c.calibrated = true;
    c.yaw.motor_id = 1;
    c.pitch.motor_id = 3;
    for a in [&mut c.yaw, &mut c.pitch] {
        a.zero_raw = Some(0);
        a.min_rad = -1.0;
        a.max_rad = 1.0;
        a.position_pid = Pid::new(20.0, 0.0, 0.0, 0.0, 30.0);
        a.speed_pid = Pid::new(100.0, 0.0, 0.0, 0.0, 1000.0);
    }
    c.settle_ticks = 5;
    c
}
fn rc(r: u8, l: u8, y: i16, p: i16) -> Remote {
    Remote {
        channels: [0, 0, y, p, 0],
        right_switch: r,
        left_switch: l,
    }
}
fn feedback(raw: u16, rpm: i16, current: i16) -> [u8; 8] {
    let mut b = [0; 8];
    b[..2].copy_from_slice(&raw.to_be_bytes());
    b[2..4].copy_from_slice(&rpm.to_be_bytes());
    b[4..6].copy_from_slice(&current.to_be_bytes());
    b
}
fn tick(c: &mut Control, r: Option<Option<Remote>>, y: Option<u16>, p: Option<u16>) {
    c.begin_tick(r);
    if let Some(raw) = y {
        c.read_feedback(0x205, &feedback(raw, 0, 0));
    }
    if let Some(raw) = p {
        c.read_feedback(0x207, &feedback(raw, 0, 0));
    }
    c.update_mode();
    c.update();
}
fn step(c: &mut Control, r: Remote, y: u16, p: u16) {
    tick(c, Some(Some(r)), Some(y), Some(p));
}
fn running() -> Control {
    let mut c = Control::new(rig());
    step(&mut c, rc(2, 3, 0, 0), 0, 0);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    for _ in 0..5 {
        step(&mut c, rc(3, 3, 0, 0), 0, 0);
    }
    assert_eq!(c.state, State::Running);
    c
}
#[test]
fn pid_examples_limits_reset_and_dt() {
    let mut p = Pid::new(2.0, 0.0, 0.0, 10.0, 100.0);
    assert_eq!(p.calculate(10.0, 7.0, DT), 6.0);
    p.maxout = 5.0;
    assert_eq!(p.calculate(10.0, 7.0, DT), 5.0);
    assert_eq!(p.calculate(7.0, 10.0, DT), -5.0);
    p.maxout = 0.0;
    assert_eq!(p.calculate(10.0, 7.0, DT), 0.0);
    p = Pid::new(0.0, 2.0, 0.0, 10.0, 100.0);
    assert_eq!(p.calculate(1.0, 0.0, 0.5), 1.0);
    p.ki = 0.0;
    assert_eq!(p.calculate(1.0, 0.0, DT), 0.0);
    p.reset();
    assert_eq!(p.integral, 0.0);
    assert!(!p.initialized);
    p = Pid::new(0.0, 0.0, 1.0, 0.0, 100.0);
    assert_eq!(p.calculate(2.0, 0.0, DT), 0.0);
    assert!((p.calculate(2.01, 0.0, DT) - 10.0).abs() < 0.001);
}
#[test]
fn pid_antiwindup_nonfinite_and_zero_limit() {
    let mut p = Pid::new(10.0, 1.0, 0.0, 20.0, 1.0);
    for _ in 0..1000 {
        assert_eq!(p.calculate(10.0, 0.0, DT), 1.0);
    }
    assert_eq!(p.integral, 0.0);
    assert_eq!(p.calculate(f32::NAN, 0.0, DT), 0.0);
    assert_eq!(p.calculate(1.0, 0.0, 0.0), 0.0);
    p.maxout = 0.0;
    assert_eq!(p.calculate(-100.0, 0.0, DT), 0.0);
}
#[test]
fn protocol_signed_bounds_unused_slots_and_routing() {
    assert_eq!(MOTOR_COMMAND_ID, 0x1ff);
    let mut data = [0; 8];
    encode_voltage(&mut data, 1, 30000.0, 30000.0);
    encode_voltage(&mut data, 3, -30000.0, 30000.0);
    assert_eq!(data, [0x61, 0xa8, 0, 0, 0x9e, 0x58, 0, 0]);
    encode_voltage(&mut data, 1, f32::NAN, 1000.0);
    assert_eq!(&data[..2], &[0, 0]);
    let mut c = Control::new(rig());
    c.read_feedback(0x205, &feedback(8191, -123, -456));
    assert_eq!(c.yaw.motor.encoder_raw, 8191);
    assert_eq!(c.yaw.motor.speed_rpm, -123.0);
    assert_eq!(c.yaw.motor.current_raw, -456);
    assert!(!c.pitch.feedback_seen);
    assert!(c.yaw.motor.angle_rad.abs() < 0.001);
    c.begin_tick(None);
    c.read_feedback(0x205, &feedback(8192, 0, 0));
    c.read_feedback(0x205, &[0; 7]);
    c.read_feedback(0x206, &feedback(0, 0, 0));
    assert_eq!(c.yaw.feedback_age, 1);
}
#[test]
fn angle_wrap_and_installation_polarity() {
    assert!((relative_angle(1, 8191, 1.0) - 2.0 * core::f32::consts::TAU / 8192.0).abs() < 1e-6);
    assert!(relative_angle(8191, 1, 1.0) < 0.0);
    let mut cfg = rig();
    cfg.yaw.direction = -1.0;
    let mut c = Control::new(cfg);
    c.read_feedback(0x205, &feedback(100, 10, 0));
    assert!(c.yaw.motor.angle_rad < 0.0);
    assert_eq!(c.yaw.motor.speed_rpm, -10.0);
    c.read_feedback(0x207, &feedback(0, 0, 0));
    c.remote = Some(rc(3, 3, 0, 0));
    c.state = State::Running;
    c.yaw.motor.command_voltage_raw = 100.0;
    assert_eq!(&c.command_data()[..2], &(-100i16).to_be_bytes());
}
#[test]
fn unconfigured_and_invalid_configs_never_enable() {
    assert!(!config::CONFIG.valid());
    let mut cfg = rig();
    cfg.calibrated = false;
    let mut c = Control::new(cfg);
    step(&mut c, rc(2, 3, 0, 0), 0, 0);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
    assert_eq!(c.command_data(), [0; 8]);
    cfg = rig();
    cfg.pitch.motor_id = cfg.yaw.motor_id;
    assert!(!cfg.valid());
    cfg = rig();
    cfg.pitch.max_rad = 4.0;
    assert!(!cfg.valid());
}
#[test]
fn startup_mid_and_continuous_settling() {
    let mut c = Control::new(rig());
    for _ in 0..10 {
        step(&mut c, rc(3, 3, 0, 0), 0, 0);
    }
    assert_eq!(c.state, State::Waiting);
    step(&mut c, rc(2, 3, 0, 0), 0, 0);
    step(&mut c, rc(3, 3, 660, 660), 100, 100);
    assert_eq!(c.state, State::Prepare);
    for _ in 0..4 {
        step(&mut c, rc(3, 3, 660, 660), 0, 0);
    }
    assert_eq!(c.state, State::Prepare);
    assert_eq!(c.yaw.target_angle_rad, 0.0);
    step(&mut c, rc(3, 3, 0, 0), 100, 100);
    assert_eq!(c.settled_ticks, 0);
    for _ in 0..5 {
        step(&mut c, rc(3, 3, 0, 0), 0, 0);
    }
    assert_eq!(c.state, State::Running);
}
#[test]
fn stick_integrates_holds_clamps_and_disable_clears() {
    let mut c = running();
    for _ in 0..100 {
        step(&mut c, rc(3, 3, 660, -660), 0, 0);
    }
    assert!((c.yaw.target_angle_rad - 0.04).abs() < 1e-5);
    assert!((c.pitch.target_angle_rad + 0.04).abs() < 1e-5);
    let target = c.yaw.target_angle_rad;
    step(&mut c, rc(3, 3, 10, -10), 0, 0);
    assert_eq!(c.yaw.target_angle_rad, target);
    c.yaw.target_angle_rad = 0.9999;
    step(&mut c, rc(3, 3, 660, 0), 0, 0);
    assert_eq!(c.yaw.target_angle_rad, 1.0);
    step(&mut c, rc(2, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
    assert_eq!(c.command_data(), [0; 8]);
    assert_eq!(c.yaw.motor.command_voltage_raw, 0.0);
}
#[test]
fn loss_invalid_frames_feedback_timeout_and_reconnect() {
    let mut c = running();
    for _ in 0..99 {
        tick(&mut c, None, Some(0), Some(0));
    }
    assert_eq!(c.state, State::Running);
    tick(&mut c, None, Some(0), Some(0));
    assert_eq!(c.state, State::Waiting);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
    step(&mut c, rc(2, 3, 0, 0), 0, 0);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Prepare);
    tick(&mut c, Some(None), Some(0), Some(0));
    assert_eq!(c.state, State::Waiting);
    let mut c = running();
    for _ in 0..20 {
        tick(&mut c, Some(Some(rc(3, 3, 0, 0))), Some(0), None);
    }
    assert_eq!(c.state, State::Waiting);
    assert_eq!(c.command_data(), [0; 8]);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
}
#[test]
fn zero_speed_no_stall_but_demand_stalls_same_tick_stop() {
    let mut c = running();
    c.yaw.position_pid.maxout = 0.0;
    for _ in 0..1000 {
        step(&mut c, rc(3, 3, 660, 0), 0, 0);
    }
    assert_eq!(c.state, State::Running);
    assert_eq!(c.yaw.target_rpm, 0.0);
    c.yaw.position_pid.maxout = 30.0;
    c.yaw.target_angle_rad = 0.5;
    for _ in 0..499 {
        step(&mut c, rc(3, 3, 0, 0), 0, 0);
    }
    assert_eq!(c.state, State::Running);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(
        c.fault,
        Some(Fault {
            axis: AxisName::Yaw,
            reason: FaultReason::Stall
        })
    );
    assert_eq!(c.yaw.motor.command_voltage_raw, 0.0);
    assert_eq!(c.pitch.motor.command_voltage_raw, 0.0);
    assert_eq!(c.command_data(), [0; 8]);
}
#[test]
fn error_latch_and_fresh_manual_recovery() {
    let mut c = running();
    c.yaw.target_angle_rad = 0.5;
    for _ in 0..500 {
        step(&mut c, rc(3, 1, 0, 0), 0, 0);
    }
    assert_eq!(c.state, State::Error);
    tick(&mut c, Some(None), None, None);
    assert_eq!(c.state, State::Error);
    step(&mut c, rc(3, 1, 0, 0), 100, 200);
    assert_eq!(c.state, State::Error);
    step(&mut c, rc(2, 2, 0, 0), 100, 200);
    assert_eq!(c.state, State::Error);
    step(&mut c, rc(3, 1, 0, 0), 100, 200);
    assert_eq!(c.state, State::Error);
    step(&mut c, rc(3, 2, 0, 0), 100, 200);
    step(&mut c, rc(3, 1, 0, 0), 100, 200);
    assert_eq!(c.state, State::Running);
    assert_eq!(c.fault, None);
    assert_eq!(c.yaw.target_angle_rad, c.yaw.motor.angle_rad);
    assert_eq!(c.pitch.target_angle_rad, c.pitch.motor.angle_rad);
    assert_eq!(c.yaw.target_rpm, 0.0);
    assert_eq!(c.yaw.speed_pid.integral, 0.0);
    assert_eq!(c.command_data(), [0; 8]);
    assert_eq!(c.recovery_count, 1);
}
#[test]
fn prepare_timeout_and_range_guard() {
    let mut cfg = rig();
    cfg.prepare_timeout_ticks = 10;
    let mut c = Control::new(cfg);
    step(&mut c, rc(2, 3, 0, 0), 100, 100);
    step(&mut c, rc(3, 3, 0, 0), 100, 100);
    for _ in 0..10 {
        step(&mut c, rc(3, 3, 0, 0), 100, 100);
    }
    assert_eq!(c.fault.unwrap().reason, FaultReason::PrepareTimeout);
    let mut c = running();
    step(&mut c, rc(3, 3, 0, 0), 2000, 0);
    assert_eq!(c.fault.unwrap().reason, FaultReason::OutsideRange);
    assert_eq!(c.command_data(), [0; 8]);
}
#[test]
fn full_cycle_frequency_rejects_noise_and_slow_motion() {
    let cfg = rig();
    for (hz, amplitude, should_trip) in
        [(10.0, 0.08, true), (3.0, 0.08, false), (20.0, 0.005, false)]
    {
        let mut d = Oscillation::default();
        let mut tripped = false;
        for t in 1..3000 {
            let angle = amplitude * (core::f32::consts::TAU * hz * t as f32 * DT).sin();
            tripped |= d.update(angle, t, &cfg);
        }
        assert_eq!(tripped, should_trip, "hz={hz} amplitude={amplitude}");
        if should_trip {
            assert!((d.frequency_hz - hz).abs() < 0.1);
        }
    }
    let mut d = Oscillation::default();
    for t in 1..300 {
        assert!(!d.update(0.08 * (0.02 * t as f32).sin(), t, &cfg));
    }
    for t in 300..2000 {
        assert!(!d.update(0.0, t, &cfg));
    }
    assert_eq!(d.high_ticks, 0);
}
#[test]
fn pitch_oscillation_and_timing_gap() {
    let mut c = running();
    for t in 1..2000 {
        let a = 0.08 * (core::f32::consts::TAU * 10.0 * t as f32 * DT).sin();
        let raw = ((a * 8192.0 / core::f32::consts::TAU) as i32).rem_euclid(8192) as u16;
        step(&mut c, rc(3, 3, 0, 0), 0, raw);
        if c.state == State::Error {
            break;
        }
    }
    assert_eq!(
        c.fault,
        Some(Fault {
            axis: AxisName::Pitch,
            reason: FaultReason::Oscillation
        })
    );
    c.timing_gap();
    assert_eq!(c.state, State::Error);
    assert_eq!(c.command_data(), [0; 8]);
    let mut c = running();
    c.timing_gap();
    assert_eq!(c.state, State::Waiting);
    step(&mut c, rc(3, 3, 0, 0), 0, 0);
    assert_eq!(c.state, State::Waiting);
}
