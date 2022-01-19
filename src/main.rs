//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
// use defmt::*;
// use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use embedded_time::{duration::Extensions, fixed_point::FixedPoint};
// use panic_probe as _;
use panic_halt as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
};

#[entry]
fn main() -> ! {
    // info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // Set the LED high for 2 seconds so we know when we're about to start the watchdog
    led_pin.set_high().unwrap();
    delay.delay_ms(2000);

    // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
    watchdog.start(1_050_000u32.microseconds());

    // Blink once a second for 5 seconds, refreshing the watchdog timer once a second to avoid a reset
    for _ in 1..=5 {
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        watchdog.feed();
    }

    // Blink 10 times per second, not feeding the watchdog.
    // The processor should reset in 1.05 seconds, or 5 blinks time
    loop {
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
    }
}

// End of file
