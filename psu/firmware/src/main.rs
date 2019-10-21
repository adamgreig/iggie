#![no_std]
#![no_main]

extern crate panic_halt;
use cortex_m_rt::entry;

use stm32f3::stm32f3x4;

#[entry]
fn start() -> ! {
    let dp = stm32f3x4::Peripherals::take().unwrap();

    let rcc = &dp.RCC;
    let gpioa = &dp.GPIOA;
    let gpiob = &dp.GPIOB;

    rcc.ahbenr.modify(|_, w| w.iopaen().enabled().iopben().enabled());
    gpioa.moder.modify(|_, w| w.moder8().output());
    gpioa.odr.write(|w| w.odr8().clear_bit());
    gpiob.moder.modify(|_, w| w.moder3().output().moder4().output());
    gpiob.odr.write(|w| w.odr3().clear_bit().odr4().clear_bit());

    loop {
        cortex_m::asm::nop();
    }
}
