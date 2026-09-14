use futures::{executor::LocalPool, future, stream::StreamExt, task::LocalSpawnExt};
use r2r::{
    Context, Node, Publisher, QosProfile,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
    },
    sensor_msgs::msg::Imu,
    std_msgs::msg::Header,
};
use serial2_tokio::SerialPort;
use std::f32::consts::PI;
use std::thread;
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // set up the serial port
    // ttyACM0 are the wheels
    // ttyACM1 is the IMU
    let port_name = "/dev/ttyACM0";
    let baud_rate = 57600;

    let Ok(port) = SerialPort::open(port_name, baud_rate) else {
        panic!("Error opening serial port at {}", port_name);
    };

    let ctx = match Context::create() {
        Ok(ctx) => ctx,
        Err(_) => panic!("couldn't start ROS2 node context"),
    };

    let mut node = match Node::create(ctx, "wrx_node_imu", "") {
        Ok(node) => node,
        Err(_) => panic!("couldn't create ROS2 node!"),
    };

    // create publisher to /imu
    let publisher: Publisher<Imu> = node.create_publisher("/imu", QosProfile::default())?;

    // start spinning the ROS2 node
    loop {
        node.spin_once(Duration::from_millis(100));
        let _ = process_readings_from_imu(&port, port_name, &publisher).await;
    }

    Ok(())
}

async fn process_readings_from_imu(
    port: &SerialPort,
    port_name: &str,
    publisher: &Publisher<Imu>,
) -> Result<(), ()> {
    let response = get_readings_from_imu(port, port_name).await;
    match response {
        Some(resp) => {
            // we split the line reading into the rotation quaternion and angular velocity
            if let Some((quat, angular_vel)) = resp.split_once("|ang_vel:") {
                // we do a further split on both the rotation quat and angular velocity
                let split_quat: Vec<&str> = quat.split(" ").collect();
                let parse_result: Result<f64, _> = split_quat[0].parse();
                match parse_result {
                    Ok(x) => {
                        // we used split_quat[0] to verify that parsing it will not have any issues (see 3 lines above).
                        // hence we can unwrap safely here
                        let y: f64 = split_quat[1].parse().unwrap();
                        let z: f64 = split_quat[2].parse().unwrap();

                        // get the angular velocity
                        let split_ang_vel: Vec<&str> = angular_vel.split(" ").collect();
                        let parse_ang_vel_result: Result<f64, _> = split_ang_vel[0].parse();

                        match parse_ang_vel_result {
                            Ok(ang_vel_x) => {
                                // we used split_ang_vel[0] to verify that parsing it will not have any issues (see 3 lines above).
                                // hence we can unwrap safely here
                                let ang_vel_y: f64 = split_ang_vel[1].parse().unwrap();
                                let ang_vel_z: f64 = split_ang_vel[2].parse().unwrap();

                                // get current timestamp
                                let now = SystemTime::now();
                                let duration =
                                    now.duration_since(UNIX_EPOCH).expect("Time is wrong");

                                // we create an Imu message here
                                let imu_msg = Imu {
                                    header: Header {
                                        stamp: Time {
                                            sec: duration.as_secs() as i32,
                                            nanosec: duration.as_nanos() as u32,
                                        },
                                        frame_id: format!(
                                            "{}-{}",
                                            duration.as_secs(),
                                            duration.as_nanos()
                                        ),
                                    },
                                    orientation: Quaternion {
                                        x: x,
                                        y: y,
                                        z: z,
                                        w: 0.0,
                                    },
                                    orientation_covariance: vec![
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    ],
                                    angular_velocity: Vector3 {
                                        x: ang_vel_x,
                                        y: ang_vel_y,
                                        z: ang_vel_z,
                                    },

                                    angular_velocity_covariance: vec![
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    ],
                                    linear_acceleration: Vector3 {
                                        x: 0.0,
                                        y: 0.0,
                                        z: 0.0,
                                    },
                                    linear_acceleration_covariance: vec![
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    ],
                                };

                                match publisher.publish(&imu_msg) {
                                    Ok(_) => {
                                        println!("published IMU successfully!");
                                    }
                                    Err(_) => {
                                        println!("error with publishing IMU msg!");
                                    }
                                }
                            }
                            Err(_) => {
                                println!("angular velocity not ready yet!");
                            }
                        }
                    }
                    Err(_) => {
                        println!("error parsing str to float");
                    }
                }
            }
        }
        None => {
            eprintln!("zero response from IMU!");
        }
    }

    Ok(())
}

async fn get_readings_from_imu(port: &SerialPort, port_name: &str) -> Option<String> {
    let mut buffer = [0; 1];
    let mut current = "";
    let mut response = String::new();

    // keep on reading until hit special break character >
    loop {
        // reset buffer
        buffer = [0; 1];

        match port.read(&mut buffer).await {
            Ok(n) => {
                // get the output from the buffer
                let output = &buffer[..n];

                // convert it to a &str slice
                current = match str::from_utf8(output) {
                    Ok(ok_output) => ok_output,
                    Err(e) => {
                        eprintln!("error at from_utf8");
                        return None;
                    }
                };

                // check if it's the special break character >
                if current == ">" {
                    break;
                }

                // if not, we add to the string per line
                response.push_str(current);
            }
            Err(e) => {
                eprintln!("error: failed to read from {}: {}", port_name, e);
                return None;
            }
        }
    }
    response = response.replace("\n", "");

    Some(response)
}
