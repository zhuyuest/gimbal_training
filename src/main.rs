#![no_std]
#![no_main]
#![forbid(unsafe_code)]
mod config;
mod control;
mod dm_mc02;
use control::{Control, MOTOR_COMMAND_ID, Remote};
use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::{Frame as _, Id};
const TICK: Duration = Duration::from_millis(1);
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);
impl Control {
    pub fn tick(&mut self, can: &mut Can1) {
        let old_state = self.state;
        let recovery_count = self.recovery_count;
        let incoming = dm_mc02::REMOTE.try_take().map(|r| {
            r.map(|r| Remote {
                channels: r.channels,
                right_switch: r.right_switch,
                left_switch: r.left_switch,
            })
        });
        self.begin_tick(incoming);
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let age = Instant::now()
                        .saturating_duration_since(envelope.ts)
                        .as_millis()
                        .min(u16::MAX as u64) as u16;
                    let frame = envelope.frame;
                    if !frame.is_remote_frame() {
                        if let Id::Standard(id) = frame.id() {
                            self.read_feedback_aged(id.as_raw(), frame.data(), age);
                        }
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
        self.update_mode();
        self.update();
        self.transmit(can);
        if old_state != self.state {
            rtt_target::rprintln!(
                "state {:?}->{:?} fault={:?}",
                old_state,
                self.state,
                self.fault
            );
        }
        if self.recovery_count != recovery_count {
            rtt_target::rprintln!(
                "RECOVER reset PID+detectors: yaw current={:.4} target={:.4}; pitch current={:.4} target={:.4}; rpm targets=0 output=0",
                self.yaw.motor.angle_rad,
                self.yaw.target_angle_rad,
                self.pitch.motor.angle_rad,
                self.pitch.target_angle_rad
            );
        }
    }
    pub fn transmit(&mut self, can: &mut Can1) {
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &self.command_data()).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 4096);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!(
        "Gimbal control started; config_ready={} (false => zero output only)",
        config::CONFIG.valid()
    );
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::new(config::CONFIG);
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
            control.timing_gap();
            rtt_target::rprintln!(
                "Control timing gap: output stopped, fresh enable required; state={:?}",
                control.state
            );
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
        log_ticks += 1;
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} ready={} prepare={} stable={} tx_fail={}",
                control.state,
                control.config.valid(),
                control.prepare_ticks,
                control.settled_ticks,
                TX_FAILED.load(Ordering::Relaxed)
            );
            for (name, axis) in [("yaw", &control.yaw), ("pitch", &control.pitch)] {
                rtt_target::rprintln!(
                    "{} raw={} angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0} age={} stall={} hz={:.1} high_ms={}",
                    name,
                    axis.motor.encoder_raw,
                    axis.motor.angle_rad,
                    axis.target_angle_rad,
                    axis.motor.speed_rpm,
                    axis.target_rpm,
                    axis.motor.command_voltage_raw,
                    axis.feedback_age,
                    axis.stall_ticks,
                    axis.oscillation.frequency_hz,
                    axis.oscillation.high_ticks
                );
            }
            rtt_target::rprintln!(
                "CAN {:03X} data={:02X?}; seen_mask={:02X} raw_ID1..7={:?}",
                MOTOR_COMMAND_ID,
                control.command_data(),
                control.discovered_mask,
                control.discovered_raw
            );
        }
    }
}
