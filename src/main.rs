use safe_drive::{
    context::Context,
    error::DynError,
    msg::common_interfaces::{sensor_msgs, nav_msgs, geometry_msgs},
};
use async_std;
use imu_localizer::imu_localizer_task;

use ros2_rust_util::{get_str_parameter, get_i64_parameter};

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("imu_localizer", None, Default::default())?;

    let sub_imu = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;
    let pub_odom = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;
    let pub_euler = node.create_publisher::<geometry_msgs::msg::Vector3>("/euler", None)?;

    let delta_time = get_i64_parameter(node.get_name(), "delta_milli", 10) as u64;
    let odom_frame_id = get_str_parameter(node.get_name(), "odom_frame_id", "odom");


    let task = async_std::task::spawn(imu_localizer_task(sub_imu, pub_odom, pub_euler, delta_time, odom_frame_id));

    task.await?;

    Ok(())
}
