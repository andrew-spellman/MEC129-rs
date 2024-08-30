#![no_std]
#![no_main]

use panic_halt as _;

use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{PinState, Pins},
    pac::Peripherals,
    usb::UsbBus,
    Sio, {Timer, Watchdog},
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use core::fmt::Write;
use heapless::String;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const EXTERN_FREQ_HZ: u32 = 12_000_000u32;

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
    let _led_green = pins.gpio5.into_push_pull_output();
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
    let _led_red = pins.gpio25.into_push_pull_output();
    let _c_test = pins.gpio26.into_push_pull_output();
    let _dis_6 = pins.gpio27.into_push_pull_output();
    let _dis_5 = pins.gpio28.into_push_pull_output();
    let _hv_feed = pins.gpio29.into_push_pull_output();

    // TODO: If using RP2040B0 or B1 investigate the issue referenced here
    // https://docs.rs/rp2040-hal/latest/rp2040_hal/usb/index.html
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}
