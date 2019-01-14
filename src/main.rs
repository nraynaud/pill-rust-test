//! Blinks an LED

#![deny(unsafe_code)]
#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f103xx_hal::{
    device,
    prelude::*,
};

#[entry]
fn main() -> ! {
    let _cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    gpioa.pa8.into_alternate_open_drain(&mut gpioa.crh);
    // prevent use
    let _led = gpioc.pc13.into_floating_input(&mut gpioc.crh);
    let mut pwm = dp.TIM2.pwm(gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl), &mut afio.mapr, 2.hz(), clocks, &mut rcc.apb1);
    pwm.enable();
    pwm.set_duty(pwm.get_max_duty() / 2);
    loop {
        asm::wfi();
    }
}