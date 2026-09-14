#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::port::{self, mode, Pin};
use arduino_hal::prelude::*;
use arduino_hal::simple_pwm::*;
use nb::block;
use panic_halt as _;
use ufmt::uwriteln;

use rotary_encoder_hal::{Direction, Rotary};

mod timer;

fn flush(mut index: usize, mut buffer: [u8; 256]) {
    index = 0;
    buffer = [0; 256];
}

#[arduino_hal::entry]
fn main() -> ! {
    let take_result = arduino_hal::Peripherals::take();
    let Some(dp) = take_result else {
        panic!("error: no connected microcontroller!");
    };
    let pins = arduino_hal::pins!(dp);

    let pin_d2 = pins.d2.into_pull_up_input();
    let pin_d3 = pins.d3.into_pull_up_input();
    let mut left_encoder_val: i32 = 0;
    let mut last_left_high = pin_d2.is_high();
    let mut enc = Rotary::new(pin_d2, pin_d3);

    // for right motor digital pins
    let pin_d4 = pins.d4.into_pull_up_input();
    let pin_d7 = pins.d7.into_pull_up_input();
    let mut right_encoder_val: i32 = 0;
    let mut last_right_high = pin_d4.is_high();
    let mut enc_right = Rotary::new(pin_d4, pin_d7);

    // add tracking of when the last command was given
    // in order to auto-stop the motors if it has been some time since
    // the last command was given (e.g 2 seconds)
    let mut last_command: u32 = 0;

    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);

    let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);

    // d5 is right motor backward
    // d9 is right motor forward
    let mut pwm_pin_d5 = pins.d5.into_output().into_pwm(&timer0);
    pwm_pin_d5.enable();

    let mut pwm_pin_d9 = pins.d9.into_output().into_pwm(&timer1);
    pwm_pin_d9.enable();

    // d6 is left motor backwards
    // d10 is left motor forward
    let mut pwm_pin_d6 = pins.d6.into_output().into_pwm(&timer0);
    pwm_pin_d6.enable();

    let mut pwm_pin_d10 = pins.d10.into_output().into_pwm(&timer1);
    pwm_pin_d10.enable();

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // for reading serial input until we get an '/r'
    let mut buffer: [u8; 256] = [0; 256];
    let mut index: usize = 0;

    ufmt::uwriteln!(&mut serial, "Ready!");

    let mut elapsed_now = 0;
    loop {
        elapsed_now += 1;
        match block!(serial.read()) {
            Ok(b) => {
                let x = b as char;

                if x == '>' {
                    // if just read finish a command fully (> is end of command line)

                    if index > 0 {
                        let received_result = core::str::from_utf8(&buffer[..index]);
                        let Ok(received_line) = received_result else {
                            continue;
                        };

                        // split the recieved command line by the delimiter " "
                        let mut split_str = received_line.split(' ');
                        let split_result = split_str.next();
                        let Some(command_char) = split_result else {
                            continue;
                        };

                        match command_char {
                            "o" => {
                                last_command = elapsed_now;

                                // setting left motor value
                                let left_motor_result = split_str.next();
                                let Some(left_motor_value) = left_motor_result else {
                                    continue;
                                };

                                let left_value_result = left_motor_value.parse::<i8>();
                                let Ok(left_value) = left_value_result else {
                                    continue;
                                };

                                // if positive value (i.e move this motor forward)
                                if left_value > 0 {
                                    pwm_pin_d10.set_duty(left_value as u8);
                                    pwm_pin_d6.set_duty(0);
                                } else {
                                    // if negative
                                    let positive = left_value * -1;
                                    pwm_pin_d6.set_duty(positive as u8);
                                    pwm_pin_d10.set_duty(0);
                                }

                                // // setting right motor value
                                let right_motor_result = split_str.next();
                                let Some(right_motor_value) = right_motor_result else {
                                    continue;
                                };

                                let right_value_result = right_motor_value.parse::<i8>();
                                let Ok(right_value) = right_value_result else {
                                    continue;
                                };

                                if right_value > 0 {
                                    pwm_pin_d9.set_duty(right_value as u8);
                                    pwm_pin_d5.set_duty(0);
                                } else {
                                    let positive = right_value * -1;
                                    pwm_pin_d5.set_duty(positive as u8);
                                    pwm_pin_d9.set_duty(0);
                                }

                                ufmt::uwriteln!(
                                    &mut serial,
                                    "{} {}",
                                    left_encoder_val,
                                    right_encoder_val
                                );
                            }
                            "e" => {
                                // if the odom/wheel node wants to read encoder values

                                // calculate left motor encoder value
                                let enc_update_results = enc.update();
                                match enc_update_results {
                                    Ok(enc_direction) => match enc_direction {
                                        Direction::Clockwise => {
                                            left_encoder_val += 1;
                                        }
                                        Direction::CounterClockwise => {
                                            left_encoder_val -= 1;
                                        }
                                        Direction::None => {}
                                    },
                                    Err(_) => {}
                                }

                                // right motor encoder value
                                let enc_update_right_results = enc_right.update();
                                match enc_update_right_results {
                                    Ok(enc_direction) => match enc_direction {
                                        Direction::Clockwise => {
                                            right_encoder_val -= 1;
                                        }
                                        Direction::CounterClockwise => {
                                            right_encoder_val += 1;
                                        }
                                        Direction::None => {}
                                    },
                                    Err(_) => {}
                                }

                                ufmt::uwriteln!(
                                    &mut serial,
                                    "{} {}",
                                    left_encoder_val,
                                    right_encoder_val
                                );
                            }
                            _ => {
                                pwm_pin_d5.set_duty(0);
                                pwm_pin_d9.set_duty(0);
                                pwm_pin_d6.set_duty(0);
                                pwm_pin_d10.set_duty(0);
                            }
                        }

                        // clear the buffer and reset the index

                        index = 0;
                        buffer = [0; 256];
                    }
                } else if index < buffer.len() - 1 {
                    buffer[index] = b;
                    index += 1;
                }
            }
            Err(_e) => {
                // stop the pins when error
                pwm_pin_d5.set_duty(0);
                pwm_pin_d9.set_duty(0);
                pwm_pin_d6.set_duty(0);
                pwm_pin_d10.set_duty(0);
                ufmt::uwriteln!(&mut serial, "reading error o command!\n");
            }
        }
    }
}
