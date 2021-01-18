#![no_std]
#![no_main]

extern crate panic_halt;
use cortex_m_rt::entry;

pub mod hal;
pub mod app;

#[entry]
fn main() -> ! {
    let flash = stm32ral::flash::FLASH::take().unwrap();
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

    rcc.setup(&flash);
    pins.setup();
    tim1.setup();

    let mut ctr = 0;
    let f1 = [
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [1, 0, 1, 0],
        [0, 1, 0, 1],
    ];
    let f2 = [
        [0, 1, 0, 1],
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [1, 0, 1, 0],
    ];
    let mut fbuf: &[[u8; 4]; 4] = &f1;

    loop {
        for row in 1..=64 {
            pins.set_row(row);
            for col in 0..64 {
                // Set up next pulse
                pins.set_col(col);
                if (10..=13).contains(&col) &&
                   ( 1..=4).contains(&row) &&
                    fbuf[col as usize - 10][row as usize - 1] == 1
                {
                    tim1.output_enable();
                } else {
                    tim1.output_disable();
                }

                // Toggle LED slowly
                ctr += 1;
                if ctr == 16000 {
                    ctr = 0;
                    pins.led.toggle();
                    if fbuf == &f1 {
                        fbuf = &f2;
                    } else {
                        fbuf = &f1;
                    }
                }

                tim1.wait_pulse();
            }
        }
    }
}
