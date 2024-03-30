use safe_drive::{
    error::DynError, 
    logger::Logger,
    msg::common_interfaces::{nav_msgs, sensor_msgs}, 
    topic::{publisher::Publisher, subscriber::Subscriber},
    msg::RosString,
    pr_info
};
use rust_imu_utils;

pub async fn imu_localizer_task(
    mut sub_imu:Subscriber<sensor_msgs::msg::Imu>,
    pub_odom:Publisher<nav_msgs::msg::Odometry>,
    delta_milli:u64,
    odom_frame_id:&str,
)->Result<(), DynError>
{
    let log = Logger::new("ImuLocalizer");

    let delta_time = delta_milli as f64 * 10e-4;

    let mut prev_velocity = rust_imu_utils::convert_to_vector(0.0, 0.0, 0.0);
    let mut ekf = rust_imu_utils::ekf::Axis6EKF::new(delta_time);
    let mut odom = nav_msgs::msg::Odometry::new().unwrap();
    odom.header.frame_id = RosString::new(odom_frame_id).unwrap();

    pr_info!(log, "Start ImuLocalizer. set interval {}s", delta_time);
    loop {
        let imu = sub_imu.recv().await.unwrap();

        let get_linear_accel = rust_imu_utils::convert_to_vector(
            imu.linear_acceleration.x, 
            imu.linear_acceleration.y, 
            imu.linear_acceleration.z);

        let get_angular_velocity = rust_imu_utils::convert_to_vector(
            imu.angular_velocity.x.to_radians(), 
            imu.angular_velocity.y.to_radians(), 
            imu.angular_velocity.z.to_radians());

        let estimated_posture = ekf.run_ekf(get_angular_velocity, get_linear_accel, delta_time);
        let estimated_quaternion = rust_imu_utils::xyz_to_quaternion(estimated_posture);

        let mut gravity_removed = rust_imu_utils::remove_gravity_from_posture(get_angular_velocity, estimated_posture, 0.981);
        gravity_removed.x = noise_filter(gravity_removed.x, 0.1);
        gravity_removed.y = noise_filter(gravity_removed.y, 0.1);
        gravity_removed.z = noise_filter(gravity_removed.z, 0.1);

        odom.pose.pose.position.x += prev_velocity.y * delta_time + 0.5 * gravity_removed.y * delta_time.powi(2);
        odom.pose.pose.position.y += prev_velocity.x * delta_time + 0.5 * gravity_removed.x * delta_time.powi(2);
        odom.pose.pose.position.z += 0.0;

        pr_info!(log, "{}", odom.pose.pose.position.y);

        prev_velocity = gravity_removed * delta_time;

        odom.pose.pose.orientation.w = estimated_quaternion.w;
        odom.pose.pose.orientation.x = estimated_quaternion.i;
        odom.pose.pose.orientation.y = estimated_quaternion.j;
        odom.pose.pose.orientation.z = estimated_quaternion.k;

        let _ = pub_odom.send(&odom).unwrap();

         

        std::thread::sleep(std::time::Duration::from_millis(delta_milli));
    }
}

fn noise_filter(value:f64, threshold:f64)->f64
{
    if value.abs() > threshold
    {
        value * 1000.0
    }
    else {
        0.0_f64
    }
}