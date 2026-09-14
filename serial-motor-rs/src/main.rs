use futures::{
    executor::LocalPool, executor::LocalSpawner, future, stream::StreamExt, task::LocalSpawnExt,
};
use libm::{cosf, sinf};
use nalgebra::UnitQuaternion;
use r2r::{
    Context, Node, Publisher, QosProfile,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{
        Point, Pose, PoseWithCovariance, Quaternion, Transform, TransformStamped, Twist,
        TwistWithCovariance, Vector3,
    },
    nav_msgs::msg::Odometry,
    std_msgs::msg::Header,
    tf2_msgs::msg::TFMessage,
};
use serial2_tokio::SerialPort;
use std::f32::consts::PI;
use std::thread;
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

pub struct EncoderVariables {
    last_read_time: u128,
    last_left_enc: i32,
    last_right_enc: i32,
    left_speed: f32,
    right_speed: f32,
}

impl EncoderVariables {
    pub fn new(
        last_read_time: u128,
        last_left_enc: i32,
        last_right_enc: i32,
        left_speed: f32,
        right_speed: f32,
    ) -> Self {
        Self {
            last_read_time,
            last_left_enc,
            last_right_enc,
            left_speed,
            right_speed,
        }
    }
}

pub struct PoseVariables {
    pose_x: f64,
    pose_y: f64,
    pose_th: f64,
}

impl PoseVariables {
    pub fn new(pose_x: f64, pose_y: f64, pose_th: f64) -> Self {
        Self {
            pose_x,
            pose_y,
            pose_th,
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // set up the serial port
    // /dev/ttyACM0 are the wheels
    // /dev/ttyACM1 is the IMU
    let port_name = "/dev/ttyACM0";
    let baud_rate = 57600;

    let Ok(port) = SerialPort::open(port_name, baud_rate) else {
        panic!("Error opening serial port at {}", port_name);
    };

    let ctx = match Context::create() {
        Ok(ctx) => ctx,
        Err(_) => panic!("couldn't start node context"),
    };

    let mut node = match Node::create(ctx, "serial_motor", "") {
        Ok(node) => node,
        Err(e) => panic!("couldn't create node!"),
    };

    // create new encoder variables object to keep track of values
    let now = SystemTime::now();
    let last_read_time = now.duration_since(UNIX_EPOCH).expect("Invalid time");
    let mut enc_variables = EncoderVariables::new(last_read_time.as_millis(), 0, 0, 0.0, 0.0);

    // create new Pose variables object to keep track of the robot's Pose
    // default everything to the starting point (0.0, 0.0, 0.0)
    let mut pose_variables = PoseVariables::new(0.0, 0.0, 0.0);

    // create subscriber to /cmd_vel and publisher to /odom
    let mut subscriber = node.subscribe::<Twist>("/cmd_vel", QosProfile::default())?;

    let publisher: Publisher<Odometry> =
        node.create_publisher::<Odometry>("/odom", QosProfile::default())?;

    let tf_publisher: Publisher<TFMessage> =
        node.create_publisher::<TFMessage>("/tf", QosProfile::default())?;

    let publisher_clone = publisher.clone();
    tokio::spawn(async move {
        let Ok(port_clone) = SerialPort::open(port_name, baud_rate) else {
            panic!("Error opening clone serial port at {}", port_name);
        };

        while let Some(msg) = subscriber.next().await {
            // we calculate the max possible speed values to move the left/right wheel
            // based on the linear and angular velocity provided by /cmd_vel
            let dist_between_wheels = 0.5;
            let max_robot_speed = 1.0;

            let left_wheel_vel = &msg.linear.x - (&msg.angular.z * dist_between_wheels / 2.0);
            let right_wheel_vel = &msg.linear.x + (&msg.angular.z * dist_between_wheels / 2.0);

            // convert to 0 to 255
            let left_converted = (left_wheel_vel / max_robot_speed) * 255.0;
            let right_converted = (right_wheel_vel / max_robot_speed) * 255.0;

            // clamp between -255 to 255
            let left_clamped = left_converted.clamp(-255.0, 255.0);
            let right_clamped = right_converted.clamp(-255.0, 255.0);

            let command = format!("o {} {}>", left_clamped as i32, right_clamped as i32);

            send_encoder_vel_command(command.to_string(), &port_clone, port_name).await;
        }
    });

    loop {
        let _ = node.spin_once(Duration::from_millis(100));
        tokio::task::yield_now().await;

        let _ = check_encoders(
            &port,
            port_name,
            &publisher,
            &tf_publisher,
            &mut enc_variables,
            &mut pose_variables,
        )
        .await;
    }

    Ok(())
}

async fn check_encoders(
    port: &SerialPort,
    port_name: &str,
    // node: &Node,
    publisher: &Publisher<Odometry>,
    tf_publisher: &Publisher<TFMessage>,
    enc_variables: &mut EncoderVariables,
    pose_variables: &mut PoseVariables,
) {
    let mut command = String::from("e");
    let check_response = send_encoder_read_command(command.to_string(), port, port_name).await;

    if check_response.len() == 2 {
        // get new current time
        let now = SystemTime::now();
        let last_read_time = now.duration_since(UNIX_EPOCH).expect("Invalid time");
        // time difference in millseconds
        let time_diff = last_read_time.as_millis() - enc_variables.last_read_time;

        // get different for left/right encoder values
        let left_m_diff = check_response[0] - enc_variables.last_left_enc;
        let right_m_diff = check_response[1] - enc_variables.last_right_enc;

        // update left/right encoder values
        enc_variables.last_left_enc = check_response[0];
        enc_variables.last_left_enc = check_response[1];

        // calculate the speed for left and right motors
        let encoder_cpr = 3000.0;
        let rads_per_ct = (2.0 * PI) / encoder_cpr;

        // update the speed for left and right motors
        enc_variables.left_speed = ((left_m_diff as f32) * rads_per_ct) / (time_diff as f32);
        enc_variables.right_speed = ((right_m_diff as f32) * rads_per_ct) / (time_diff as f32);

        // create a header message
        let time_stamp = Time {
            sec: last_read_time.as_secs() as i32,
            nanosec: last_read_time.as_nanos() as u32,
        };
        let header = Header {
            stamp: time_stamp.clone(),
            frame_id: "odom".to_string(),
        };

        // compute odometry in a typical way given the velocities of the roboti
        let dt = (time_diff as f32) / 1000.0; // time difference in seconds
        let vy = 0.0; // velocity y
        let vth = enc_variables.left_speed; // set vth same as x velocity
        let delta_x = (enc_variables.left_speed * cosf(pose_variables.pose_th as f32)
            - vy * sinf(pose_variables.pose_th as f32))
            * dt;
        let delta_y = (enc_variables.left_speed * sinf(pose_variables.pose_th as f32)
            + vy * cosf(pose_variables.pose_th as f32))
            * dt;
        let delta_th = vth * dt;

        pose_variables.pose_x += delta_x as f64;
        pose_variables.pose_y += delta_y as f64;
        pose_variables.pose_th += delta_th as f64;

        // since all odometry is 6DOF we will need a quaternion created from yaw
        let unit_quat = UnitQuaternion::from_euler_angles(0.0, 0.0, pose_variables.pose_th);
        let odom_quat = unit_quat.quaternion();

        // update last read time
        enc_variables.last_read_time = last_read_time.as_millis();

        // create the pose
        let pose = Pose {
            position: Point {
                x: pose_variables.pose_x,
                y: pose_variables.pose_y,
                z: 0.0,
            },
            orientation: Quaternion {
                x: odom_quat.i,
                y: odom_quat.j,
                z: odom_quat.k,
                w: odom_quat.w,
            },
        };

        // create the transform that we will broadcast over the /tf channel
        let header_tf = Header {
            stamp: time_stamp,
            frame_id: "odom".to_string(),
        };
        let tf_stamped = TransformStamped {
            header: header_tf,
            child_frame_id: "base_link".to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: pose_variables.pose_x,
                    y: pose_variables.pose_y,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: odom_quat.i,
                    y: odom_quat.j,
                    z: odom_quat.k,
                    w: odom_quat.w,
                },
            },
        };
        // broadcast it
        let tf_message = TFMessage {
            transforms: vec![tf_stamped],
        };
        tf_publisher.publish(&tf_message);

        let linear_vel = (enc_variables.left_speed + enc_variables.right_speed) / 2.0;

        // wheel separation is the distance between the 2 wheels
        let wheel_separation = 0.2;
        let angular_vel = (enc_variables.right_speed - enc_variables.left_speed) / wheel_separation;

        // create the twist for velocity
        let twist = Twist {
            linear: Vector3 {
                x: linear_vel as f64,
                y: vy as f64,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: angular_vel as f64,
            },
        };

        // create the odom message
        let odom_msg = Odometry {
            header: header,
            child_frame_id: "base_footprint".to_string(),
            pose: PoseWithCovariance {
                pose: pose,
                covariance: vec![
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                ],
            },
            twist: TwistWithCovariance {
                twist: twist,
                covariance: vec![
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                ],
            },
        };

        // publish the odom message
        publisher.publish(&odom_msg);
    }
    // Ok(())
}

async fn send_encoder_vel_command(command: String, port: &SerialPort, port_name: &str) {
    let _ = send_write_command(port, command, port_name).await;
}

async fn send_encoder_read_command(
    command: String,
    port: &SerialPort,
    port_name: &str,
) -> Vec<i32> {
    let response = send_command(port, command, port_name).await;
    match response {
        Some(resp) => {
            let mut if_error = false;

            let split_numbers: Vec<i32> = resp
                .as_str()
                .split(' ')
                .map(|each| match each.parse::<i32>() {
                    Ok(num) => num,
                    Err(_) => {
                        if_error = true;
                        0
                    }
                })
                .collect();

            if if_error {
                return vec![];
            }

            return split_numbers;
        }
        None => {
            vec![]
        }
    }
}

async fn send_write_command(port: &SerialPort, mut cmd: String, port_name: &str) -> Option<String> {
    let mut buffer = cmd.into_bytes();

    match port.write_all(&mut buffer).await {
        Ok(_) => {
            println!("successful");
            Some(String::from("Write successful"))
        }
        Err(_) => {
            eprintln!("error sending command");
            None
        }
    }
}

async fn send_command(port: &SerialPort, mut cmd: String, port_name: &str) -> Option<String> {
    cmd.push_str(">");

    // send the command
    let mut buffer = cmd.into_bytes();

    match port.write_all(&mut buffer).await {
        Ok(_) => {
            // if no errors, we start reading the output
            let mut response = String::new();
            let mut current = "";

            let mut buffer = [0; 512];
            match port.read(&mut buffer).await {
                Ok(n) => {
                    let output = &buffer[..n];

                    let convert_results = str::from_utf8(output);
                    match convert_results {
                        Ok(ok_out) => response.push_str(ok_out),
                        Err(_) => {
                            return None;
                        }
                    }
                }
                Err(ref e) => return None,
                Err(e) => {
                    eprintln!("error: failed to read from {}:{}", port_name, e);
                    return None;
                }
            }

            response = response.replace("\r", "");
            response = response.replace("\n", "");
            return Some(response);
        }
        Err(_e) => {
            eprintln!("error sending command");
            None
        }
    }
}
