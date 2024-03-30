use safe_drive::{
    context::Context,
    error::DynError,
    msg::common_interfaces::{sensor_msgs, nav_msgs},
};
use async_std;
use imu_localizer::imu_localizer_task;


#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("imu_localizer", None, Default::default())?;

    let sub_imu = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;
    let pub_odom = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let task = async_std::task::spawn(imu_localizer_task(sub_imu, pub_odom, 10, "odom"));

    task.await?;

    Ok(())
}