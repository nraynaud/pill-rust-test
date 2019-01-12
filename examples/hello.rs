//! Prints "Hello, world!" on the host console using semihosting

#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f103xx;

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!").unwrap();
    hprintln!("Hello2, world!").unwrap();


    loop {}
}
