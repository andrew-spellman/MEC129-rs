#![no_std]
#![no_main]

use embedded_hal::{adc::OneShot, digital::v2::OutputPin, PwmPin};

use rp2040_hal::{
    adc::{Adc, AdcPin},
    clocks::init_clocks_and_plls,
    gpio::Pins,
    pac::Peripherals,
    pwm::{FreeRunning, Slices},
    usb::UsbBus,
    Sio, Timer, Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{embedded_io::Write, SerialPort};

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

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut pac = unsafe { Peripherals::steal() };
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

    // TODO: fix not printing here
    _ = writeln!(serial, "{}", info);

    let mut dummy_buffer = [0; 8];
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            _ = serial.read(&mut dummy_buffer);
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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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
    let mut led_green = pins.gpio5.into_push_pull_output();
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
    // let _hv_pwm = pins.gpio22.into_push_pull_output_in_state(PinState::Low);

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // 8 slices, each with 2 channels
    // each of the 30 gpio pins is associated with one of these 16 channels
    // see 4.5.2 page 525 for which channel to use
    // wanting to use channel 3A here for hv_pwm on gpio 22
    let mut pwm = pwm_slices.pwm3;

    // we want 100khz which is 1/1250th of the 125Mhz clk_sys
    // for this, for every output clock cycle we will:
    //     count once per clock cycle (int = 1, frac = 0)
    //     up to top = 1249 (there's an implicit +1)
    pwm.set_div_int(1u8);
    pwm.set_div_frac(0u8);
    // TODO: this " / 2" is temporary for testing
    pwm.set_top(1249u16 / 2);
    pwm.enable();

    // counter is free-running, and will count continuously whenever the slice is enabled
    let pwm = pwm.into_mode::<FreeRunning>();

    let mut channel_hv_pwm = pwm.channel_a;
    let _hv_pwm = channel_hv_pwm.output_to(pins.gpio22);
    channel_hv_pwm.set_duty(0);

    let _rh_pwm = pins.gpio23.into_push_pull_output();
    let _tec_pwm = pins.gpio24.into_push_pull_output();
    let _led_red = pins.gpio25.into_push_pull_output();
    let _c_test = pins.gpio26.into_push_pull_output();
    let _dis_6 = pins.gpio27.into_push_pull_output();
    let _dis_5 = pins.gpio28.into_push_pull_output();

    //  hal docs use floating input for adc
    let hv_feed = pins.gpio29.into_floating_input();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_hv_feed = AdcPin::new(hv_feed).unwrap();

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
        GetHVFeed,
        SetHVPWM(usize),
    }

    let mut dummy_user_controlled_value = 42;

    const INPUT_BUFFER_CAPACITY: usize = 32;
    let mut input_buffer = [0u8; INPUT_BUFFER_CAPACITY];
    let mut input_buffer_len = 0;
    let mut green_led_counter = timer.get_counter();

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            match serial.read(&mut input_buffer[input_buffer_len..]) {
                Err(_) | Ok(0) => (),
                Ok(count) => {
                    // TODO: Potentially make all lowercase
                    for byte in input_buffer.iter().skip(input_buffer_len).take(count) {
                        if *byte == b'\r' {
                            _ = serial.write_all(b"\\r\r\n");
                        } else {
                            _ = serial.write_all(&[*byte]);
                        }
                    }
                    input_buffer_len += count;
                    if input_buffer_len == INPUT_BUFFER_CAPACITY {
                        _ = serial.write_all(
                            b"\r\ninput buffer is full, no commands are this long, clearing it!\r\n",
                        );
                        input_buffer_len = 0;
                    }
                }
            }
        }

        let mut command: Option<Command> = None;
        if input_buffer_len != 0 && input_buffer[input_buffer_len - 1] == b'\r' {
            match input_buffer[0..input_buffer_len] {
                // get hv feed*
                [b'g', b'e', b't', b' ', b'h', b'v', b' ', b'f', b'e', b'e', b'd', ..] => {
                    command = Some(Command::GetHVFeed);
                }
                // set duty *
                [b's', b'e', b't', b' ', b'd', b'u', b't', b'y', ..] => {
                    if let Some(n) = usize_from_bytes(&input_buffer[9..input_buffer_len - 1]) {
                        if n < 1024 {
                            command = Some(Command::SetHVPWM(n));
                        } else {
                            _ = serial.write_all(b"duty must be in range 0..1024\r\n");
                        }
                    } else {
                        _ = serial.write_all(b"failed parsing number to set\r\n");
                    }
                }
                // get*
                [b'g', b'e', b't', ..] => {
                    command = Some(Command::Get);
                }
                // set *
                [b's', b'e', b't', b' ', ..] => {
                    if let Some(n) = usize_from_bytes(&input_buffer[4..input_buffer_len - 1]) {
                        command = Some(Command::Set(n));
                    } else {
                        _ = serial.write_all(b"failed parsing number to set\r\n");
                    }
                }
                _ => {
                    _ = serial.write_all(b"unrecognized command\r\n");
                }
            }
            input_buffer_len = 0;
        }
        match command {
            Some(Command::Get) => {
                write!(serial, "current value: {}\r\n", dummy_user_controlled_value).unwrap();
            }
            Some(Command::Set(n)) => {
                dummy_user_controlled_value = n;
            }
            Some(Command::GetHVFeed) => {
                let adc_counts_hv_feed: u16 = adc.read(&mut adc_hv_feed).unwrap();
                write!(serial, "adc_counts_hv_feed: {}\r\n", adc_counts_hv_feed).unwrap();
                let counts_max: f32 = 4095.0;
                let divided_volts_hv_feed = (adc_counts_hv_feed as f32) / counts_max * 3.3;
                write!(
                    serial,
                    "divided_volts_hv_feed : {}\r\n",
                    divided_volts_hv_feed
                )
                .unwrap();
                let volts_hv_feed = divided_volts_hv_feed * (6.49f32 + 1000f32) / 6.49f32;
                write!(serial, "volts_hv_feed : {}\r\n", volts_hv_feed).unwrap();
            }
            Some(Command::SetHVPWM(n)) => {
                channel_hv_pwm.set_duty(n as u16);
            }
            None => (),
        }

        let current_count = timer.get_counter();
        if (current_count - green_led_counter).to_millis() > 1000 {
            if (current_count - green_led_counter).to_millis() > 2000 {
                led_green.set_high().unwrap();
                green_led_counter = current_count;
            } else {
                led_green.set_low().unwrap();
            }
        }
    }
}

fn usize_from_bytes(input_slice: &[u8]) -> Option<usize> {
    let mut number: String<32> = String::new();
    for byte in input_slice.iter() {
        number.push(*byte as char).unwrap();
    }
    if let Ok(n) = number.parse::<usize>() {
        Some(n)
    } else {
        None
    }
}
