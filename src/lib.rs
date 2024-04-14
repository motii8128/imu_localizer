use safe_drive::{
    error::DynError, 
    logger::Logger,
    msg::common_interfaces::{nav_msgs, sensor_msgs, geometry_msgs}, 
    topic::{publisher::Publisher, subscriber::Subscriber},
    msg::RosString,
    pr_info
};
use rust_imu_utils;

pub async fn axis_6_imu_localizer_task(
    mut sub_imu:Subscriber<sensor_msgs::msg::Imu>,
    pub_odom:Publisher<nav_msgs::msg::Odometry>,
    pub_euler:Publisher<geometry_msgs::msg::Vector3>,
    delta_milli:u64,
    odom_frame_id:String,
)->Result<(), DynError>
{
    let log = Logger::new("ImuLocalizer");

    let delta_time = delta_milli as f64 * 10e-4;

    let mut prev_velocity = rust_imu_utils::convert_to_vector(0.0, 0.0, 0.0);
    let mut prev_accel = rust_imu_utils::convert_to_vector(0.0, 0.0, 0.0);
    let mut ekf = rust_imu_utils::ekf::Axis6EKF::new(delta_time);
    let mut odom = nav_msgs::msg::Odometry::new().unwrap();
    odom.header.frame_id = RosString::new(odom_frame_id.as_str()).unwrap();

    pr_info!(log, "Start ImuLocalizer. Param delta time:{}, frame_id:{}", delta_time, odom_frame_id);
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
        let estimated_quaternion = rust_imu_utils::quaternion_utils::xyz_to_quaternion(estimated_posture);

        let mut euler_msg = geometry_msgs::msg::Vector3::new().unwrap();
        euler_msg.x = estimated_posture.x / 2.0;
        euler_msg.y = estimated_posture.y / 2.0;
        euler_msg.z = estimated_posture.z / 2.0;

        if euler_msg.x > 3.14
        {
            euler_msg.x = -6.28 + euler_msg.x;
        }
        if euler_msg.y > 3.14
        {
            euler_msg.y = -6.28 + euler_msg.y;
        }
        if euler_msg.z > 3.14
        {
            euler_msg.z = -6.28 + euler_msg.z;
        }

        let _ = pub_euler.send(&euler_msg);

        let mut gravity_removed = rust_imu_utils::remove_gravity_from_posture(get_angular_velocity, estimated_posture, 0.981);
        gravity_removed.x = noise_filter(gravity_removed.x, 0.1);
        gravity_removed.y = noise_filter(gravity_removed.y, 0.1);
        gravity_removed.z = noise_filter(gravity_removed.z, 0.1);

        
        odom.twist.twist.linear.x = (gravity_removed.x + prev_accel.x)* delta_time * 0.5;
        odom.twist.twist.linear.y = (gravity_removed.y + prev_accel.y)* delta_time * 0.5;
        odom.twist.twist.linear.z = (gravity_removed.z + prev_accel.z)* delta_time * 0.5;

        // odom.pose.pose.position.x += (odom.twist.twist.linear.y + prev_velocity.y) * delta_time * 0.5;
        // odom.pose.pose.position.y -= (odom.twist.twist.linear.x + prev_velocity.x) * delta_time * 0.5;
        // odom.pose.pose.position.z = 0.0;

        // pr_info!(log, "{}", odom.pose.pose.position.y);

        odom.pose.pose.orientation.w = estimated_quaternion.w;
        odom.pose.pose.orientation.x = estimated_quaternion.i;
        odom.pose.pose.orientation.y = estimated_quaternion.j;
        odom.pose.pose.orientation.z = estimated_quaternion.k;

        let _ = pub_odom.send(&odom).unwrap();

        prev_velocity.x = odom.twist.twist.linear.x;
        prev_velocity.y = odom.twist.twist.linear.y;
        prev_velocity.z = odom.twist.twist.linear.z;
        prev_accel.x = gravity_removed.x;
        prev_accel.y = gravity_removed.y;
        prev_accel.z = gravity_removed.z;

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

pub async fn axis_9_imu_localizer_task(
    mut sub_imu:Subscriber<sensor_msgs::msg::Imu>,
    mut sub_magnetic:Subscriber<geometry_msgs::msg::Vector3>,
    pub_odom:Publisher<nav_msgs::msg::Odometry>,
    pub_euler:Publisher<geometry_msgs::msg::Vector3>,
    delta_milli:u64,
    odom_frame_id:String,
)->Result<(), DynError>
{
    let log = Logger::new("Axis9IMULocakuzer");

    let delta_time = delta_milli as f64 * 10e-4;

    let mut odom = nav_msgs::msg::Odometry::new().unwrap();
    odom.header.frame_id = RosString::new(odom_frame_id.as_str()).unwrap();

    let mut ekf9 = rust_imu_utils::ekf::Axis9EKF::new(delta_time);
    let mut ekf6 = rust_imu_utils::ekf::Axis6EKF::new(delta_time);

    pr_info!(log, "Start ImuLocalizer. Param delta time:{}, frame_id:{}", delta_time, odom_frame_id);
    loop {
        let get_imu = sub_imu.recv().await.unwrap();
        let get_magnet_field = sub_magnetic.recv().await.unwrap();

        let get_linear_accel = rust_imu_utils::convert_to_vector(
            get_imu.linear_acceleration.x, 
            get_imu.linear_acceleration.y, 
            get_imu.linear_acceleration.z);

        let get_angular_velocity = rust_imu_utils::convert_to_vector(
            get_imu.angular_velocity.x.to_radians(),
            get_imu.angular_velocity.y.to_radians(),
            get_imu.angular_velocity.z.to_radians());

        let get_magnetic = rust_imu_utils::convert_to_vector(
            get_magnet_field.x, 
            get_magnet_field.y, 
            get_magnet_field.z);

        let nine_estimated = ekf9.run_ekf(get_angular_velocity, get_linear_accel, get_magnetic, delta_time);
        let six_estimated = ekf6.run_ekf(get_angular_velocity, get_linear_accel, delta_time);
        let estimated = rust_imu_utils::convert_to_vector(
            six_estimated.x, 
            nine_estimated.y, 
            six_estimated.z);
        let q = rust_imu_utils::quaternion_utils::xyz_to_quaternion(estimated);
        
        odom.pose.pose.orientation.w = q.w;
        odom.pose.pose.orientation.x = q.i;
        odom.pose.pose.orientation.y = q.j;
        odom.pose.pose.orientation.z = q.k;

        let mut euler = geometry_msgs::msg::Vector3::new().unwrap();
        euler.x = estimated.x;
        euler.y = estimated.y;
        euler.z = estimated.z;

    
        let _ = pub_euler.send(&euler);
        let _ = pub_odom.send(&odom);

        std::thread::sleep(std::time::Duration::from_millis(delta_milli));
    }
}

