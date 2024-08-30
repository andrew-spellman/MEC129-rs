#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;

use hal::gpio::PinState;
use hal::pac;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
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
    let _hv_pwm = pins.gpio22.into_push_pull_output_in_state(PinState::Low);

    let _rh_pwm = pins.gpio23.into_push_pull_output();
    let _tec_pwm = pins.gpio24.into_push_pull_output();
    let _led_red = pins.gpio25.into_push_pull_output();
    let _c_test = pins.gpio26.into_push_pull_output();
    let _dis_6 = pins.gpio27.into_push_pull_output();
    let _dis_5 = pins.gpio28.into_push_pull_output();
    let _hv_feed = pins.gpio29.into_push_pull_output();

    loop {
        led_green.set_high().unwrap();
        timer.delay_ms(500);
        led_green.set_low().unwrap();
        timer.delay_ms(500);
    }
}
