//! Hardware-independent 1 kHz control logic, also compiled by tests/host.rs.
use core::f32::consts::{PI, TAU};
pub const DT: f32 = 0.001;
pub const MOTOR_COMMAND_ID: u16 = 0x1ff;
pub const REMOTE_SILENT_TICKS: u16 = 100;

#[derive(Clone, Copy, Debug)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Limit on integral CONTRIBUTION, in output units.
    pub integral_limit: f32,
    pub maxout: f32,
    pub integral: f32,
    pub previous_error: f32,
    pub initialized: bool,
}
impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, maxout: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            maxout,
            integral: 0.0,
            previous_error: 0.0,
            initialized: false,
        }
    }
    pub fn valid(&self) -> bool {
        [self.kp, self.ki, self.kd, self.integral_limit, self.maxout]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0)
    }
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.initialized = false;
    }
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        self.calculate_limited(target, measured, dt_s, self.maxout)
    }
    pub fn calculate_limited(&mut self, target: f32, measured: f32, dt_s: f32, limit: f32) -> f32 {
        if !self.valid()
            || !target.is_finite()
            || !measured.is_finite()
            || !dt_s.is_finite()
            || dt_s <= 0.0
            || !limit.is_finite()
            || limit < 0.0
        {
            self.reset();
            return 0.0;
        }
        let cap = self.maxout.min(limit);
        if cap == 0.0 {
            self.reset();
            return 0.0;
        }
        let error = target - measured;
        let derivative = if self.initialized {
            (error - self.previous_error) / dt_s
        } else {
            0.0
        };
        self.previous_error = error;
        self.initialized = true;
        if self.ki == 0.0 {
            self.integral = 0.0;
        }
        let candidate = (self.integral + self.ki * error * dt_s)
            .clamp(-self.integral_limit, self.integral_limit);
        let pd = self.kp * error + self.kd * derivative;
        let trial = pd + candidate;
        // Conditional integration: allow unwinding, but not further windup into saturation.
        if (trial <= cap || error < 0.0) && (trial >= -cap || error > 0.0) {
            self.integral = candidate;
        }
        let output = pd + self.integral;
        if !output.is_finite() {
            self.reset();
            return 0.0;
        }
        output.clamp(-cap, cap)
    }
}

#[derive(Clone, Copy)]
pub struct AxisConfig {
    pub motor_id: u8,
    pub zero_raw: Option<u16>,
    pub direction: f32,
    pub stick_direction: f32,
    pub min_rad: f32,
    pub max_rad: f32,
    pub position_pid: Pid,
    pub speed_pid: Pid,
}
impl AxisConfig {
    fn valid(&self) -> bool {
        (1..=4).contains(&self.motor_id)
            && self.zero_raw.is_some_and(|v| v < 8192)
            && self.direction.abs() == 1.0
            && self.stick_direction.abs() == 1.0
            && self.min_rad.is_finite()
            && self.max_rad.is_finite()
            && self.min_rad > -PI
            && self.min_rad < 0.0
            && self.max_rad > 0.0
            && self.max_rad < PI
            && self.position_pid.valid()
            && self.speed_pid.valid()
    }
}
#[derive(Clone, Copy)]
pub struct RigConfig {
    pub calibrated: bool,
    pub yaw: AxisConfig,
    pub pitch: AxisConfig,
    pub voltage_limit: f32,
    pub prepare_voltage_limit: f32,
    pub prepare_rpm_limit: f32,
    pub stick_rate_rad_s: f32,
    pub deadzone: f32,
    pub feedback_timeout_ticks: u16,
    pub settle_angle_rad: f32,
    pub settle_speed_rpm: f32,
    pub settle_ticks: u32,
    pub prepare_timeout_ticks: u32,
    pub stall_demand_rpm: f32,
    pub stall_output_raw: f32,
    pub stall_speed_rpm: f32,
    pub stall_ticks: u32,
    pub oscillation_amplitude_rad: f32,
    pub oscillation_hz: f32,
    pub oscillation_ticks: u32,
}
impl RigConfig {
    pub fn valid(&self) -> bool {
        self.calibrated
            && self.yaw.valid()
            && self.pitch.valid()
            && self.yaw.motor_id != self.pitch.motor_id
            && [
                self.voltage_limit,
                self.prepare_voltage_limit,
                self.prepare_rpm_limit,
                self.stick_rate_rad_s,
                self.settle_angle_rad,
                self.settle_speed_rpm,
                self.stall_demand_rpm,
                self.stall_output_raw,
                self.stall_speed_rpm,
                self.oscillation_amplitude_rad,
                self.oscillation_hz,
            ]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0)
            && self.voltage_limit <= 25000.0
            && self.prepare_voltage_limit <= self.voltage_limit
            && self.deadzone.is_finite()
            && (0.0..1.0).contains(&self.deadzone)
            && self.feedback_timeout_ticks > 0
            && self.settle_ticks > 0
            && self.prepare_timeout_ticks > self.settle_ticks
            && self.stall_ticks > 0
            && self.oscillation_ticks > 0
            && self.oscillation_hz < 500.0
    }
}

#[derive(Default)]
pub struct Motor {
    pub encoder_raw: u16,
    pub shaft_angle_rad: f32,
    /// Calibrated direct-drive joint angle; targets use this same coordinate system.
    pub angle_rad: f32,
    /// Motor rpm in the positive joint direction.
    pub speed_rpm: f32,
    pub current_raw: i16,
    /// Logical joint direction. The encoder applies installation polarity once.
    pub command_voltage_raw: f32,
}
impl Motor {
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }
    pub fn read(&mut self, data: &[u8; 8]) -> bool {
        let raw = u16::from_be_bytes([data[0], data[1]]);
        if raw >= 8192 {
            return false;
        }
        self.encoder_raw = raw;
        self.shaft_angle_rad = raw as f32 * TAU / 8192.0;
        self.angle_rad = self.shaft_angle_rad;
        self.speed_rpm = i16::from_be_bytes([data[2], data[3]]) as f32;
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
        true
    }
}
pub fn relative_angle(raw: u16, zero: u16, direction: f32) -> f32 {
    let mut diff = raw as i32 - zero as i32;
    if diff >= 4096 {
        diff -= 8192;
    }
    if diff < -4096 {
        diff += 8192;
    }
    diff as f32 * TAU / 8192.0 * direction
}
pub fn encode_voltage(data: &mut [u8; 8], id: u8, voltage: f32, limit: f32) {
    if !(1..=4).contains(&id) {
        return;
    }
    let v = if voltage.is_finite() && limit.is_finite() && limit >= 0.0 {
        voltage.clamp(-limit.min(25000.0), limit.min(25000.0)) as i16
    } else {
        0
    };
    let slot = (id as usize - 1) * 2;
    data[slot..slot + 2].copy_from_slice(&v.to_be_bytes());
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Waiting,
    Prepare,
    Running,
    Error,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultReason {
    Stall,
    Oscillation,
    PrepareTimeout,
    OutsideRange,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisName {
    Yaw,
    Pitch,
    Both,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fault {
    pub axis: AxisName,
    pub reason: FaultReason,
}

/// Hysteretic turning-point detector. Only excursions >= amplitude register a reversal.
/// Two consecutive reversals form one full period; estimates overlap, elapsed persistence
/// counts only the newest half-period, so frequency/persistence are not doubled.
#[derive(Default)]
pub struct Oscillation {
    initialized: bool,
    direction: i8,
    low: f32,
    high: f32,
    extreme_tick: u32,
    last_turn: Option<u32>,
    previous_turn: Option<u32>,
    pub frequency_hz: f32,
    pub high_ticks: u32,
}
impl Oscillation {
    pub fn update(&mut self, angle: f32, tick: u32, cfg: &RigConfig) -> bool {
        if !self.initialized {
            self.initialized = true;
            self.low = angle;
            self.high = angle;
            self.extreme_tick = tick;
            return false;
        }
        // With no turn for a whole threshold period it cannot be continuously high-frequency.
        if self
            .last_turn
            .is_some_and(|t| (tick - t) as f32 * DT > 1.0 / cfg.oscillation_hz)
        {
            self.high_ticks = 0;
            self.frequency_hz = 0.0;
            self.previous_turn = None;
            self.last_turn = None;
        }
        let a = cfg.oscillation_amplitude_rad;
        let mut turn = None;
        match self.direction {
            0 => {
                self.low = self.low.min(angle);
                self.high = self.high.max(angle);
                if angle - self.low >= a {
                    self.direction = 1;
                    self.high = angle;
                    self.extreme_tick = tick;
                } else if self.high - angle >= a {
                    self.direction = -1;
                    self.low = angle;
                    self.extreme_tick = tick;
                }
            }
            1 => {
                if angle >= self.high {
                    self.high = angle;
                    self.extreme_tick = tick;
                } else if self.high - angle >= a {
                    turn = Some(self.extreme_tick);
                    self.direction = -1;
                    self.low = angle;
                    self.extreme_tick = tick;
                }
            }
            _ => {
                if angle <= self.low {
                    self.low = angle;
                    self.extreme_tick = tick;
                } else if angle - self.low >= a {
                    turn = Some(self.extreme_tick);
                    self.direction = 1;
                    self.high = angle;
                    self.extreme_tick = tick;
                }
            }
        }
        if let Some(t) = turn {
            if let (Some(prev), Some(last)) = (self.previous_turn, self.last_turn) {
                let period = t - prev;
                if period > 0 {
                    self.frequency_hz = 1.0 / (period as f32 * DT);
                    if self.frequency_hz > cfg.oscillation_hz {
                        self.high_ticks = self.high_ticks.saturating_add(t - last);
                    } else {
                        self.high_ticks = 0;
                    }
                }
            }
            self.previous_turn = self.last_turn;
            self.last_turn = Some(t);
        }
        self.high_ticks >= cfg.oscillation_ticks
    }
}

pub struct Axis {
    pub motor: Motor,
    pub target_angle_rad: f32,
    pub target_rpm: f32,
    pub position_pid: Pid,
    pub speed_pid: Pid,
    pub feedback_age: u16,
    pub feedback_seen: bool,
    pub stall_ticks: u32,
    pub oscillation: Oscillation,
    pub config: AxisConfig,
}
impl Axis {
    fn new(config: AxisConfig) -> Self {
        Self {
            motor: Motor::default(),
            target_angle_rad: 0.0,
            target_rpm: 0.0,
            position_pid: config.position_pid,
            speed_pid: config.speed_pid,
            feedback_age: u16::MAX,
            feedback_seen: false,
            stall_ticks: 0,
            oscillation: Oscillation::default(),
            config,
        }
    }
    fn accept(&mut self, data: &[u8; 8], age: u16) {
        if self.feedback_seen && age > self.feedback_age {
            return;
        }
        if self.motor.read(data) {
            if let Some(zero) = self.config.zero_raw {
                self.motor.angle_rad =
                    relative_angle(self.motor.encoder_raw, zero, self.config.direction);
            }
            self.motor.speed_rpm *= self.config.direction;
            self.feedback_age = age;
            self.feedback_seen = true;
        }
    }
    fn fresh(&self, cfg: &RigConfig) -> bool {
        self.feedback_seen && self.feedback_age < cfg.feedback_timeout_ticks
    }
    fn in_range(&self) -> bool {
        self.motor.angle_rad.is_finite()
            && self.motor.angle_rad >= self.config.min_rad
            && self.motor.angle_rad <= self.config.max_rad
    }
    fn reset(&mut self, target: f32) {
        self.target_angle_rad = target;
        self.target_rpm = 0.0;
        self.position_pid.reset();
        self.speed_pid.reset();
        self.motor.disable();
        self.stall_ticks = 0;
        self.oscillation = Oscillation::default();
    }
    fn run(&mut self, prepare: bool, cfg: &RigConfig) {
        let output_limit = if prepare {
            cfg.prepare_voltage_limit
        } else {
            cfg.voltage_limit
        };
        // Bounded joint coordinates: never blindly wrap target error through a hard stop.
        self.target_rpm = if prepare {
            self.position_pid.calculate_limited(
                self.target_angle_rad,
                self.motor.angle_rad,
                DT,
                cfg.prepare_rpm_limit,
            )
        } else {
            self.position_pid
                .calculate(self.target_angle_rad, self.motor.angle_rad, DT)
        };
        self.motor.command_voltage_raw = self.speed_pid.calculate_limited(
            self.target_rpm,
            self.motor.speed_rpm,
            DT,
            output_limit,
        );
    }
    fn detect(&mut self, tick: u32, cfg: &RigConfig) -> Option<FaultReason> {
        if self.target_rpm.abs() >= cfg.stall_demand_rpm
            && self.motor.command_voltage_raw.abs() >= cfg.stall_output_raw
            && self.motor.speed_rpm.abs() <= cfg.stall_speed_rpm
        {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
        } else {
            self.stall_ticks = 0;
        }
        if self.stall_ticks >= cfg.stall_ticks {
            return Some(FaultReason::Stall);
        }
        if self.oscillation.update(self.motor.angle_rad, tick, cfg) {
            return Some(FaultReason::Oscillation);
        }
        None
    }
}

#[derive(Clone, Copy)]
pub struct Remote {
    pub channels: [i16; 5],
    pub right_switch: u8,
    pub left_switch: u8,
}
impl Remote {
    fn valid(&self) -> bool {
        self.channels.iter().all(|v| (-660..=660).contains(v))
            && (1..=3).contains(&self.right_switch)
            && (1..=3).contains(&self.left_switch)
    }
}
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<Remote>,
    pub remote_silent_ticks: u16,
    pub fault: Option<Fault>,
    pub prepare_ticks: u32,
    pub settled_ticks: u32,
    pub recovery_count: u32,
    pub config: RigConfig,
    pub tick_count: u32,
    enable_armed: bool,
    recovery_armed: bool,
    previous_right: u8,
    previous_left: u8,
    just_entered: bool,
    /// Zero-output discovery works even before motor IDs/zero offsets are configured.
    pub discovered_raw: [u16; 7],
    pub discovered_mask: u8,
}
impl Control {
    pub fn new(config: RigConfig) -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis::new(config.yaw),
            pitch: Axis::new(config.pitch),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            fault: None,
            prepare_ticks: 0,
            settled_ticks: 0,
            recovery_count: 0,
            config,
            tick_count: 0,
            enable_armed: false,
            recovery_armed: false,
            previous_right: 0,
            previous_left: 0,
            just_entered: false,
            discovered_raw: [0; 7],
            discovered_mask: 0,
        }
    }
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }
    fn cancel_gestures(&mut self) {
        self.enable_armed = false;
        self.recovery_armed = false;
        self.previous_right = 0;
        self.previous_left = 0;
    }
    fn waiting(&mut self) {
        self.state = State::Waiting;
        self.yaw.reset(self.yaw.motor.angle_rad);
        self.pitch.reset(self.pitch.motor.angle_rad);
        self.prepare_ticks = 0;
        self.settled_ticks = 0;
    }
    fn latch(&mut self, axis: AxisName, reason: FaultReason) {
        self.fault = Some(Fault { axis, reason });
        self.state = State::Error;
        self.cancel_gestures();
        self.disable_all();
    }
    /// Call before ingesting this cycle's CAN frames. None = no new UART frame;
    /// Some(None) = explicit invalid frame / serial error, invalidated immediately.
    pub fn begin_tick(&mut self, incoming: Option<Option<Remote>>) {
        self.tick_count = self.tick_count.wrapping_add(1);
        // Keep turn timestamps far from wrap without affecting the safety state.
        if self.tick_count == 0 {
            self.yaw.oscillation = Oscillation::default();
            self.pitch.oscillation = Oscillation::default();
        }
        self.yaw.feedback_age = self.yaw.feedback_age.saturating_add(1);
        self.pitch.feedback_age = self.pitch.feedback_age.saturating_add(1);
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = incoming {
            self.remote = remote.filter(Remote::valid);
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            self.cancel_gestures();
            if self.state != State::Error {
                self.waiting();
            }
        }
    }
    #[cfg(test)]
    pub fn read_feedback(&mut self, standard_id: u16, data: &[u8]) {
        self.read_feedback_aged(standard_id, data, 0);
    }
    /// Include time already spent in the CAN queue; dequeue time is not reception time.
    pub fn read_feedback_aged(&mut self, standard_id: u16, data: &[u8], age: u16) {
        if age >= self.config.feedback_timeout_ticks {
            return;
        }
        let Ok(data) = <&[u8; 8]>::try_from(data) else {
            return;
        };
        if (0x205..=0x20b).contains(&standard_id) {
            let raw = u16::from_be_bytes([data[0], data[1]]);
            if raw < 8192 {
                let index = (standard_id - 0x205) as usize;
                self.discovered_raw[index] = raw;
                self.discovered_mask |= 1 << index;
            }
        }
        if (1..=4).contains(&self.yaw.config.motor_id)
            && standard_id == 0x204 + self.yaw.config.motor_id as u16
        {
            self.yaw.accept(data, age);
        }
        if (1..=4).contains(&self.pitch.config.motor_id)
            && standard_id == 0x204 + self.pitch.config.motor_id as u16
        {
            self.pitch.accept(data, age);
        }
    }
    pub fn update_mode(&mut self) {
        self.just_entered = false;
        let Some(remote) = self.remote else {
            return;
        };
        let fresh = self.yaw.fresh(&self.config) && self.pitch.fresh(&self.config);
        let ready = fresh && self.config.valid();
        let right = remote.right_switch;
        let left = remote.left_switch;
        let new_enable = self.enable_armed && self.previous_right == 2 && right == 3;
        let neutral =
            self.stick(remote.channels[2]) == 0.0 && self.stick(remote.channels[3]) == 0.0;
        let recover =
            self.recovery_armed && self.previous_left != 1 && left == 1 && right == 3 && neutral;
        self.previous_right = right;
        self.previous_left = left;
        if self.state == State::Error {
            self.disable_all();
            if !ready || right != 3 || !neutral {
                self.recovery_armed = false;
            } else if left == 2 {
                self.recovery_armed = true;
            }
            if recover {
                self.recovery_armed = false; // consume even if feedback/range conditions fail
                if ready && self.yaw.in_range() && self.pitch.in_range() {
                    self.yaw.reset(self.yaw.motor.angle_rad);
                    self.pitch.reset(self.pitch.motor.angle_rad);
                    self.fault = None;
                    self.state = State::Running;
                    self.just_entered = true;
                    self.recovery_count = self.recovery_count.wrapping_add(1);
                }
            }
            return;
        }
        if !ready {
            self.waiting();
            self.enable_armed = false;
            return;
        }
        if right != 3 {
            self.waiting();
            self.enable_armed = right == 2;
            return;
        }
        if self.state == State::Waiting {
            self.enable_armed = false;
            if new_enable && self.yaw.in_range() && self.pitch.in_range() {
                self.yaw.reset(0.0);
                self.pitch.reset(0.0);
                self.prepare_ticks = 0;
                self.settled_ticks = 0;
                self.state = State::Prepare;
                self.just_entered = true;
            }
            return;
        }
        if !self.yaw.in_range() {
            self.latch(AxisName::Yaw, FaultReason::OutsideRange);
            return;
        }
        if !self.pitch.in_range() {
            self.latch(AxisName::Pitch, FaultReason::OutsideRange);
            return;
        }
        if self.state == State::Prepare {
            self.prepare_ticks += 1;
            let settled = [&self.yaw, &self.pitch].iter().all(|a| {
                (a.target_angle_rad - a.motor.angle_rad).abs() <= self.config.settle_angle_rad
                    && a.motor.speed_rpm.abs() <= self.config.settle_speed_rpm
            });
            self.settled_ticks = if settled { self.settled_ticks + 1 } else { 0 };
            if self.settled_ticks >= self.config.settle_ticks {
                self.state = State::Running;
                self.just_entered = true;
                // reset at same reference target, avoid derivative/integral carryover
                self.yaw.reset(0.0);
                self.pitch.reset(0.0);
            } else if self.prepare_ticks >= self.config.prepare_timeout_ticks {
                self.latch(AxisName::Both, FaultReason::PrepareTimeout);
            }
        }
    }
    fn stick(&self, channel: i16) -> f32 {
        let v = channel as f32 / 660.0;
        if v.abs() <= self.config.deadzone {
            0.0
        } else {
            v.signum() * (v.abs() - self.config.deadzone) / (1.0 - self.config.deadzone)
        }
    }
    pub fn update(&mut self) {
        if !matches!(self.state, State::Prepare | State::Running) {
            self.disable_all();
            return;
        }
        // Recovery/entry has an explicit zero-output tick, preserving logged reset values.
        if self.just_entered {
            self.disable_all();
            return;
        }
        if self.state == State::Running {
            if let Some(remote) = self.remote {
                let yaw_step = self.stick(remote.channels[2]) * self.config.stick_rate_rad_s * DT;
                let pitch_step = self.stick(remote.channels[3]) * self.config.stick_rate_rad_s * DT;
                self.yaw.target_angle_rad = (self.yaw.target_angle_rad
                    + yaw_step * self.yaw.config.stick_direction)
                    .clamp(self.yaw.config.min_rad, self.yaw.config.max_rad);
                self.pitch.target_angle_rad = (self.pitch.target_angle_rad
                    + pitch_step * self.pitch.config.stick_direction)
                    .clamp(self.pitch.config.min_rad, self.pitch.config.max_rad);
            }
        }
        self.yaw.run(self.state == State::Prepare, &self.config);
        self.pitch.run(self.state == State::Prepare, &self.config);
        if let Some(reason) = self.yaw.detect(self.tick_count, &self.config) {
            self.latch(AxisName::Yaw, reason);
        } else if let Some(reason) = self.pitch.detect(self.tick_count, &self.config) {
            self.latch(AxisName::Pitch, reason);
        }
    }
    pub fn command_data(&self) -> [u8; 8] {
        let mut data = [0; 8];
        // Defense in depth: no stale command may escape a disabled or invalid state.
        if self.config.valid()
            && self.remote.is_some()
            && self.yaw.fresh(&self.config)
            && self.pitch.fresh(&self.config)
            && matches!(self.state, State::Prepare | State::Running)
        {
            encode_voltage(
                &mut data,
                self.yaw.config.motor_id,
                self.yaw.motor.command_voltage_raw * self.yaw.config.direction,
                self.config.voltage_limit,
            );
            encode_voltage(
                &mut data,
                self.pitch.config.motor_id,
                self.pitch.motor.command_voltage_raw * self.pitch.config.direction,
                self.config.voltage_limit,
            );
        }
        data
    }
    /// A missed control deadline invalidates remote/feedback and requires a new gesture.
    pub fn timing_gap(&mut self) {
        self.remote = None;
        self.remote_silent_ticks = REMOTE_SILENT_TICKS;
        self.yaw.feedback_age = u16::MAX;
        self.pitch.feedback_age = u16::MAX;
        self.cancel_gestures();
        self.disable_all();
        if self.state != State::Error {
            self.waiting();
        }
    }
}
