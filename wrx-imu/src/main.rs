#![no_std]
#![no_main]

use panic_halt as _;

use lexical_core::BUFFER_SIZE;

use arduino_hal::prelude::*;

use bno080::{
    interface::I2cInterface,
    wrapper::{WrapperError, BNO080},
};

use embedded_hal::blocking::delay::DelayMs;

use ufloat::Uf32;

use mint::{Quaternion, Vector3};

// constants
pub const BNO085_ADDR: u8 = 0x4b;

// memory address for reading quaternion for IMU orientation
pub const BNO085_QUA_DATA_W_LSB: u8 = 0x4a;

// memory address for reading Euler angles representation of heading (roll, pitch, yaw)
pub const BNO085_EUL_HEADING_LSB: u8 = 0x1A;

// memory address for reading linear acceleration vector
pub const BNO085_LTA_DATA_X_LSB: u8 = 0x28;

// memory address for reading gravity vector
pub const BNO085_GRV_DATA_X_LSB: u8 = 0x2E;

// memory address for reading accelerometer
pub const BNO085_ACC_DATA_X_LSB: u8 = 0x08;

fn get_angular_velocity_from_quat(
    previous_quat: Quaternion<f32>,
    current_quat: Quaternion<f32>,
    dt: f32,
) -> Vector3<f32> {
    // conjugate previous quaternion
    let previous_w = previous_quat.s;
    let previous_x = -previous_quat.v.x;
    let previous_y = -previous_quat.v.y;
    let previous_z = -previous_quat.v.z;

    // multiple current quat by the conjugate of previous quaternion
    let diff_x =
        current_quat.s * previous_x + current_quat.v.x * previous_w + current_quat.v.y * previous_z
            - current_quat.v.z * previous_y;
    let diff_y = current_quat.s * previous_y - current_quat.v.x * previous_z
        + current_quat.v.y * previous_w
        + current_quat.v.z * previous_x;
    let diff_z = current_quat.s * previous_z + current_quat.v.x * previous_y
        - current_quat.v.y * previous_x
        + current_quat.v.z * previous_w;

    // scale by 2 / dt
    let scale = 2.0 / dt;
    let vel_x = diff_x * scale;
    let vel_y = diff_y * scale;
    let vel_z = diff_z * scale;

    Vector3::<f32> {
        x: vel_x,
        y: vel_y,
        z: vel_z,
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let Some(dp) = arduino_hal::Peripherals::take() else {
        ufmt::uwriteln!(&mut serial, "error reading peripherals at the start!").unwrap();
    };

    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let pin_a4 = pins.a4.into_pull_up_input();
    let pin_a5 = pins.a5.into_pull_up_input();

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI, pin_a4, pin_a5, 100_000, // 100khz
    );

    let iface = bno080::interface::I2cInterface::default(i2c);
    let mut imu = BNO080::new_with_interface(iface);

    ufmt::uwriteln!(&mut serial, "Initializing BNO085...").unwrap();

    let mut delay = arduino_hal::Delay::new();

    // delay needed for BNO085 to boot
    arduino_hal::delay_ms(1000);

    match imu.init(&mut delay) {
        Ok(_) => ufmt::uwriteln!(&mut serial, "Initialization successful!").unwrap(),
        Err(_) => ufmt::uwriteln!(&mut serial, "Init failed!").unwrap(),
    }

    imu.enable_rotation_vector(50).unwrap();

    let mut current_quat: Quaternion<f32> = Quaternion {
        v: Vector3::<f32> {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        s: 0.0,
    };
    let mut previous_quat: Quaternion<f32> = Quaternion {
        v: Vector3::<f32> {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        s: 0.0,
    };

    // simple counter to ensure that we only start calculating from the 2nd loop
    // as we use the first loop to get the quat and use it as the "1st previous" quat
    let mut count = 0;
    loop {
        let msg_count = imu.handle_all_messages(&mut delay, 1u8);

        if msg_count > 0 {
            let read_result = imu.rotation_quaternion();
            match read_result {
                Ok(quat) => {
                    let (i, j, k, real) = (quat[0], quat[1], quat[2], quat[3]);

                    if count > 0 {
                        current_quat.v = Vector3::<f32> { x: i, y: j, z: k };
                        current_quat.s = real;

                        let angular_vel =
                            get_angular_velocity_from_quat(previous_quat, current_quat, 0.01);

                        // broadcast the angular velocity
                        ufmt::uwriteln!(
                            &mut serial,
                            "ang_vel: x={} y={} z={}",
                            Uf32(angular_vel.x, 3),
                            Uf32(angular_vel.y, 3),
                            Uf32(angular_vel.z, 3)
                        )
                        .unwrap();
                    }

                    // broadcast the orientation quaternion
                    ufmt::uwriteln!(
                        &mut serial,
                        "Q: x={} y={} z={} r={}",
                        Uf32(i, 3),
                        Uf32(j, 3),
                        Uf32(k, 3),
                        Uf32(real, 3)
                    )
                    .unwrap();

                    // update the previous quat with the new readings to prepare for the next loop
                    previous_quat.v = Vector3::<f32> { x: i, y: j, z: k };
                    previous_quat.s = real;
                }
                Err(e) => {
                    ufmt::uwriteln!(&mut serial, "error reading quat!").unwrap();
                }
            }
        }

        arduino_hal::delay_ms(10);

        count += 1;
    }
}
