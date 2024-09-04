#![no_std]
#![no_main]

use embedded_hal::digital::{OutputPin, StatefulOutputPin};
// use panic_halt as _;

use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{bank0, FunctionSio, Pin, PinState, Pins, PullDown, SioOutput},
    pac::Peripherals,
    usb::UsbBus,
    Sio, Timer, Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{embedded_io::Write, SerialPort};

use core::fmt::Write as OtherWrite;
use core::panic::PanicInfo;
use heapless::String;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const EXTERN_FREQ_HZ: u32 = 12_000_000u32;

static mut TIMER: Option<Timer> = None;
static mut LED_RED: Option<Pin<bank0::Gpio25, FunctionSio<SioOutput>, PullDown>> = None;
static mut LED_GREEN: Option<Pin<bank0::Gpio5, FunctionSio<SioOutput>, PullDown>> = None;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe { LED_GREEN.as_mut().unwrap() }.set_high().unwrap();
    let mut red_led_counter = unsafe { TIMER.unwrap().get_counter() };
    loop {
        let current_count = unsafe { TIMER.unwrap().get_counter() };
        if (current_count - red_led_counter).to_millis() > 1000 {
            red_led_counter = current_count;
            unsafe { LED_RED.as_mut().unwrap() }.toggle().unwrap();
        }
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        EXTERN_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let _timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    unsafe {
        TIMER = Some(_timer);
    }

    // The single-cycle I/O block controls our GPIO pins
    let sio = Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _dir = pins.gpio0.into_push_pull_output();
    let _step = pins.gpio1.into_push_pull_output();
    let _mode1 = pins.gpio2.into_push_pull_output();
    let _mode0 = pins.gpio3.into_push_pull_output();
    let _standby = pins.gpio4.into_push_pull_output();
    let led_green = pins.gpio5.into_push_pull_output();
    let _switch_ino = pins.gpio6.into_push_pull_output();
    let _t_ready = pins.gpio7.into_push_pull_output();
    let _eeprom_scl = pins.gpio8.into_push_pull_output();
    let _enable = pins.gpio9.into_push_pull_output();
    let _error = pins.gpio10.into_push_pull_output();
    let _switch_in1 = pins.gpio11.into_push_pull_output();
    let _hv_oe = pins.gpio12.into_push_pull_output();
    let _hv_nle = pins.gpio13.into_push_pull_output();
    let _hv_clk = pins.gpio14.into_push_pull_output();
    let _hv_npol = pins.gpio15.into_push_pull_output();
    let _hv_nbl = pins.gpio16.into_push_pull_output();
    let _hv_dir = pins.gpio17.into_push_pull_output();
    let _hv_din = pins.gpio18.into_push_pull_output();
    let _t_scl = pins.gpio19.into_push_pull_output();
    let _t_sda = pins.gpio20.into_push_pull_output();
    let _eeprom_sda = pins.gpio21.into_push_pull_output();

    // Pull down HV_PWM to stop the HV Supply from pulling a large current
    let _hv_pwm = pins.gpio22.into_push_pull_output_in_state(PinState::Low);

    let _rh_pwm = pins.gpio23.into_push_pull_output();
    let _tec_pwm = pins.gpio24.into_push_pull_output();
    let led_red = pins.gpio25.into_push_pull_output();
    let _c_test = pins.gpio26.into_push_pull_output();
    let _dis_6 = pins.gpio27.into_push_pull_output();
    let _dis_5 = pins.gpio28.into_push_pull_output();
    let _hv_feed = pins.gpio29.into_push_pull_output();

    unsafe {
        LED_GREEN = Some(led_green);
        LED_RED = Some(led_red);
    }

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2)
        .build();

    enum Command {
        Get,
        Set(usize),
    }

    let mut dummy_user_controlled_value = 42;

    const COMMAND_BUFFER_CAPACITY: usize = 32;
    let mut command_buffer = [0u8; COMMAND_BUFFER_CAPACITY];
    let mut command_buffer_len = 0;
    let mut input_buffer = [0u8; 32];
    let mut green_led_counter = unsafe { TIMER.unwrap().get_counter() };

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            match serial.read(&mut input_buffer) {
                Err(_) | Ok(0) => (),
                Ok(count) => {
                    for byte in input_buffer.iter().take(count) {
                        if command_buffer_len == COMMAND_BUFFER_CAPACITY {
                            _ = serial.write_all(&input_buffer);
                            _ = serial.write_all(
                                b"\r\nCommand buffer is full, clearing command buffer!\r\n",
                            );
                            command_buffer_len = 0;
                            break;
                        }

                        command_buffer[command_buffer_len] = *byte;
                        command_buffer_len += 1;

                        if *byte == b'\r' {
                            _ = serial.write(b"\\r\r\n");
                            command_buffer_len -= 1;
                            break;
                        }
                        _ = serial.write_all(&[*byte]);
                    }
                }
            }
        }

        let mut command: Option<Command> = None;
        if command_buffer[command_buffer_len] == b'\r' {
            match command_buffer[0..command_buffer_len] {
                [b'G', b'e', b't'] => {
                    command = Some(Command::Get);
                }
                [b'S', b'e', b't', b' ', ..] => {
                    let mut number: String<32> = String::new();
                    for byte in command_buffer[4..command_buffer_len].iter() {
                        number.push(*byte as char).unwrap();
                    }
                    if let Ok(n) = number.parse::<usize>() {
                        command = Some(Command::Set(n));
                    } else {
                        _ = serial.write_all(b"failed parsing number to set");
                    }
                }
                _ => {
                    _ = serial.write_all(b"unrecognized command");
                    command_buffer_len = 0;
                }
            }
        }
        match command {
            Some(Command::Get) => {
                let mut text: String<32> = String::new();
                writeln!(&mut text, "current value: {}", dummy_user_controlled_value).unwrap();
            }
            Some(Command::Set(n)) => {
                dummy_user_controlled_value = n;
            }
            None => (),
        }

        let current_count = unsafe { TIMER.unwrap().get_counter() };
        if (current_count - green_led_counter).to_millis() > 1000 {
            green_led_counter = current_count;
            unsafe { LED_GREEN.as_mut().unwrap() }.toggle().unwrap();
        }
    }

    //     let mut said_hello = false;
    //     loop {
    //         // A welcome message at the beginning
    //         if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
    //             said_hello = true;
    //             let _ = serial.write(b"Hello, World!\r\n");

    //            let time = timer.get_counter().ticks();
    //            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();
    //             // the serial port is smaller than the buffers available to the USB
    //             // peripheral. In general, the return value should be handled, so that
    //             // bytes not transferred yet don't get lost.
    //            let _ = serial.write(text.as_bytes());

    //        // Check for new data
    //             let mut buf = [0u8; 64];
    //             match serial.read(&mut buf) {
    //                 Err(_e) => {
    //                     // Do nothing
    //                 }
    //                Ok(0) => {
    //                 }
    //                 Ok(count) => {
    //                     // Convert to upper case
    //                     buf.iter_mut().take(count).for_each(|b| {
    //                         b.make_ascii_uppercase();
    //                     });
    //                     // Send back to the host
    //                     let mut wr_ptr = &buf[..count];
    //                     while !wr_ptr.is_empty() {
    //                         match serial.write(wr_ptr) {
    //                             Ok(len) => wr_ptr = &wr_ptr[len..],
    //                             // On error, just drop unwritten data.
    //                             // One possible error is Err(WouldBlock), meaning the USB
    //                             // write buffer is full.
    //                             Err(_) => break,
    //                         };
    //                     }
    //                 }
    //             }
    //         }
    //     }
}
