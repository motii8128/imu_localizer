use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::{sensor_msgs, nav_msgs},
    topic::{publisher::Publisher, subscriber::Subscriber},
    msg::RosString
};
use async_std;
use rust_imu_utils;


#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("imu_localizer", None, Default::default())?;

    let sub_imu = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;
    let pub_odom = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let localization_task = async_std::task::spawn(localizer_task(sub_imu, pub_odom, 10));

    localization_task.await?;

    Ok(())
}

async fn localizer_task(
    mut sub_imu:Subscriber<sensor_msgs::msg::Imu>,
    pub_odom:Publisher<nav_msgs::msg::Odometry>,
    delta_milli:u64
)->Result<(), DynError>
{
    let log = Logger::new("IMULocalizer");

    let mut odom = nav_msgs::msg::Odometry::new().unwrap();
    odom.header.frame_id = RosString::new("odom").unwrap();

    let delta_time = delta_milli as f32 * 10e-4;

    pr_info!(log, "Init EKF");
    let mut ekf = rust_imu_utils::ekf::Axis6EKF::new(delta_time);

    pr_info!(log, "Start IMULocalizer:interval:{}ms pub:{} sub:{}", delta_milli, pub_odom.get_topic_name(), sub_imu.get_topic_name());
    loop {
        let get_imu = sub_imu.recv().await.unwrap();

        let gyro_velocity = rust_imu_utils::convert_to_vector(
            get_imu.angular_velocity.x.to_radians() as f32, 
            get_imu.angular_velocity.y.to_radians() as f32, 
            get_imu.angular_velocity.z.to_radians() as f32);

        let linear_accel = rust_imu_utils::convert_to_vector(
            get_imu.linear_acceleration.x as f32, 
            get_imu.linear_acceleration.y as f32, 
            get_imu.linear_acceleration.z as f32);

        let result = ekf.run_ekf(gyro_velocity, linear_accel, delta_time);
        let q = rust_imu_utils::xyz_to_quaternion(result);

        let (x, y, z) = rust_imu_utils::remove_gravity_from_euler(linear_accel,result, 0.981);

        
        pr_info!(log, "value:{}", fixed(x));

        odom.pose.pose.position.x += odom.twist.twist.linear.x * delta_time as f64 + 0.5*(fixed(x)*delta_time.powi(2)) as f64;
        odom.pose.pose.position.y += odom.twist.twist.linear.y * delta_time as f64 + 0.5*(fixed(y)*delta_time.powi(2)) as f64;
        odom.pose.pose.position.z += 0.0;
        odom.twist.twist.linear.x += (fixed(x)*delta_time) as f64;
        odom.twist.twist.linear.y += (fixed(y)*delta_time) as f64;
        odom.twist.twist.linear.z += (fixed(z)*delta_time) as f64;
        odom.pose.pose.orientation.w = q.w as f64;
        odom.pose.pose.orientation.x = q.i as f64;
        odom.pose.pose.orientation.y = q.j as f64;
        odom.pose.pose.orientation.z = q.k as f64;

        let _ = pub_odom.send(&odom);

        std::thread::sleep(std::time::Duration::from_millis(delta_milli));
    }
}

fn fixed(value:f32)->f32
{
    if value.abs() < 0.3
    {
        0.0
    }
    else {
        value
    }
}