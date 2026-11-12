//! 根据真实云台填写配置，并与助教确认后，再将 calibrated 设置为 true。
//! 当前实现仅支持直驱，且相对标定零位的允许角度范围必须严格位于 (-π, π) 内。
//! 若存在减速传动或多圈运动，需要另行确认并实现可靠的位置确定方法。
use crate::control::{AxisConfig, Pid, RigConfig};

pub const CONFIG: RigConfig = RigConfig {
    calibrated: false,
    yaw: AxisConfig {
        motor_id: 0,          // 实际电机 ID 待确认；本实现支持 1～4
        zero_raw: None,       // 正前方姿态对应的编码器读数；None 表示尚未标定
        direction: 1.0,       // 电机与关节的方向转换，只能为 1.0 或 -1.0，需现场确认
        stick_direction: 1.0, // 摇杆与目标角度的方向转换，需确认左右操作方向
        min_rad: 0.0,         // 最小允许角度，单位 rad；当前为未配置占位值
        max_rad: 0.0,
        position_pid: Pid::new(0.0, 0.0, 0.0, 0.0, 0.0),
        speed_pid: Pid::new(0.0, 0.0, 0.0, 0.0, 0.0),
    },
    pitch: AxisConfig {
        motor_id: 0,
        zero_raw: None,
        direction: 1.0,
        stick_direction: 1.0,
        min_rad: 0.0,
        max_rad: 0.0,
        position_pid: Pid::new(0.0, 0.0, 0.0, 0.0, 0.0),
        speed_pid: Pid::new(0.0, 0.0, 0.0, 0.0, 0.0),
    },
    // 以下数值为初始软件设置，尚未经过真实教具验证。
    // 设置 calibrated 为 true 前需现场核对；PID 自身的输出限幅也可能更小。
    voltage_limit: 1000.0, // 电压指令原始值上限，不是伏特；协议绝对值上限为 25000
    prepare_voltage_limit: 500.0,
    prepare_rpm_limit: 10.0,
    stick_rate_rad_s: 0.4,
    deadzone: 0.05,
    feedback_timeout_ticks: 20, // 反馈超时阈值；1 kHz 控制下，20 个周期约为 20 ms
    settle_angle_rad: 0.03,
    settle_speed_rpm: 1.0,
    settle_ticks: 500,
    prepare_timeout_ticks: 15000,
    stall_demand_rpm: 3.0,
    stall_output_raw: 400.0,
    stall_speed_rpm: 0.5,
    stall_ticks: 500,
    oscillation_amplitude_rad: 0.02,  // 确认反向转折所需的最小角度回摆量，单位 rad
    oscillation_hz: 8.0,
    oscillation_ticks: 1000, // 持续超频时间阈值；1000 个控制周期约为 1 s
}；
