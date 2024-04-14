use safe_drive::{
    context::Context,
    error::DynError,
    msg::common_interfaces::{sensor_msgs, nav_msgs, geometry_msgs},
};

use async_std;
use imu_localizer::axis_9_imu_localizer_task;

use ros2_rust_util::{get_str_parameter, get_i64_parameter};

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("axis9_imu_localizer", None, Default::default())?;

    let sub_imu = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;
    let sub_mag = node.create_subscriber::<geometry_msgs::msg::Vector3>("/mag", None)?;
    let pub_odom = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;
    let pub_euler = node.create_publisher::<geometry_msgs::msg::Vector3>("/euler", None)?;

    let delta_time = get_i64_parameter(node.get_name().unwrap().as_str(), "delta_milli", 1) as u64;
    let odom_frame_id = get_str_parameter(node.get_name().unwrap().as_str(), "odom_frame_id", "odom");


    let task = async_std::task::spawn(axis_9_imu_localizer_task(
        sub_imu,
        sub_mag,
        pub_odom,
        pub_euler,
        delta_time,
        odom_frame_id
    ));

    task.await?;

    Ok(())
}