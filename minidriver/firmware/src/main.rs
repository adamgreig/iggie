#![no_std]
#![no_main]

extern crate panic_halt;
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

pub mod hal;
pub mod app;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let rcc = hal::rcc::RCC::new(stm32ral::rcc::RCC::take().unwrap());
    let gpioa = hal::gpio::GPIO::new(stm32ral::gpio::GPIOA::take().unwrap());
    let gpiob = hal::gpio::GPIO::new(stm32ral::gpio::GPIOB::take().unwrap());
    let tim1 = hal::tim::TIM::new(stm32ral::tim1::TIM1::take().unwrap());

    let pins = hal::gpio::Pins {
        gk1: gpiob.pin(5),
        gk2: gpiob.pin(6),
        gk3: gpiob.pin(7),
        gk4: gpiob.pin(8),
        a0: gpioa.pin(2),
        a1: gpioa.pin(1),
        a2: gpioa.pin(4),
        a3: gpioa.pin(5),
        le: gpioa.pin(3),
        en: gpioa.pin(8),
        led: gpiob.pin(1),
        psu_run: gpioa.pin(7),
    };

    rcc.setup();
    pins.setup();
    tim1.setup();

    let mut ctr = 0;
    let mut idx = 0;

    let cols = [10, 11, 12, 13, 13, 12, 11, 10, 10, 11, 12, 13, 13, 12, 11, 10, 10, 11, 12, 13, 13, 12, 11, 10];
    let rows = [1, 1, 1, 1, 2, 2, 2, 2, 4, 4, 4, 4, 8, 8, 8, 8, 4, 4, 4, 4, 2, 2, 2, 2];

    loop {
        if tim1.cc1if_set() {
            tim1.clear_sr();

            pins.set_rows(rows[idx]);
            pins.set_col(cols[idx]);

            ctr += 1;
            if ctr == 100 {
                ctr = 0;
                idx += 1;
                if idx == cols.len() {
                    idx = 0;
                }

                pins.led.toggle();
            }
        }
    }
}
