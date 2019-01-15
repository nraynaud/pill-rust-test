//! Blinks an LED

#![deny(unsafe_code)]
#![no_std]
#![no_main]
//MCU: STM32F103C8T699
extern crate panic_halt;

use rtfm::{app, Duration, Instant, U32Ext};
use stm32f103xx_hal::{
    gpio::gpioc::PC13,
    gpio::Output,
    gpio::PushPull,
    prelude::*,
    time::Hertz,
};

fn hertz_to_cycles(sysclock: Hertz, hertz: Hertz) -> Duration {
    (sysclock.0 / hertz.0).cycles()
}

#[app(device = stm32f103xx)]
const APP: () = {
    static mut LED_GLOBAL: PC13<Output<PushPull>> = ();
    static mut PERIOD: Duration = ();

    #[init(schedule = [toggle])]
    unsafe fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let sysclock = clocks.sysclk();
        let period = hertz_to_cycles(sysclock, 10.hz());
        schedule.toggle(Instant::now() + period).unwrap();
        PERIOD = period;
        LED_GLOBAL = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    }

    #[task(schedule = [toggle], resources = [LED_GLOBAL, PERIOD])]
    fn toggle() {
        resources.LED_GLOBAL.toggle();
        schedule.toggle(scheduled + *resources.PERIOD).unwrap();
    }

    extern "C" {
        fn USART1();
    }
};