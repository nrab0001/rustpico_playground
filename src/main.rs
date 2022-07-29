//! # Pico PWM Blink Example
//!
//! Fades the LED on a Pico board using the PWM peripheral.
//!
//! This will fade in/out the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rp2040_hal::gpio::DynPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;


/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let mut led=pins.led.into_push_pull_output();

    let mut segment_pins: [DynPin; 8] =
        [pins.gpio10.into(),
        pins.gpio6.into(),
        pins.gpio3.into(),
        pins.gpio1.into(),
        pins.gpio0.into(),
        pins.gpio9.into(),
        pins.gpio4.into(),
        pins.gpio2.into()];

    let mut display_select_pins: [DynPin; 4] =
        [pins.gpio11.into(),
        pins.gpio8.into(),
        pins.gpio7.into(),
        pins.gpio5.into(),
        ];

    // Turn pins into push pull outputs
    for pin in segment_pins.iter_mut() {
        pin.into_push_pull_output();
    }
    led.set_high();

    for pin in display_select_pins.iter_mut() {
        pin.into_push_pull_output();
    }

    //set all display to on
    for pin in display_select_pins.iter_mut() {
        pin.set_low().unwrap();
    }

    display_select_pins[0].set_high().unwrap();

    // Infinite loop, fading LED up and down
    loop {
        display_select_pins[0].set_low().unwrap();
        for pin in segment_pins.iter_mut() {
            pin.set_high().unwrap();
        }
        delay.delay_ms(1000);
        for pin in segment_pins.iter_mut(){
            pin.set_low().unwrap();
        }
        delay.delay_ms(1000)
    }
}

// End of file
