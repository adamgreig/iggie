#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_semihosting::hprintln;
use rtfm::cyccnt::U32Ext;

use stm32ral::{read_reg, write_reg, modify_reg};

#[rtfm::app(device=stm32ral::stm32f3::stm32f3x4, monotonic=rtfm::cyccnt::CYCCNT, peripherals=true)]
const APP: () = {
    struct Resources {
        gpiob: stm32ral::gpio::Instance,
    }

    #[init(spawn=[foo])]
    fn init(mut cx: init::Context) -> init::LateResources {
        hprintln!("init").unwrap();

        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let rcc = &cx.device.RCC;
        let gpioa = &cx.device.GPIOA;
        let gpiob = &cx.device.GPIOB;

        modify_reg!(stm32ral::rcc, rcc, AHBENR, IOPAEN: Enabled, IOPBEN: Enabled);
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Output);
        modify_reg!(stm32ral::gpio, gpioa, ODR, ODR8: 0);
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER3: Output, MODER4: Output);
        modify_reg!(stm32ral::gpio, gpiob, ODR, ODR3: 0, ODR4: 0);

        cx.spawn.foo().unwrap();

        init::LateResources { gpiob: cx.device.GPIOB }
    }

    #[task(resources=[gpiob], schedule=[foo])]
    fn foo(cx: foo::Context) {
        if read_reg!(stm32ral::gpio, cx.resources.gpiob, ODR, ODR3 == 0) {
            write_reg!(stm32ral::gpio, cx.resources.gpiob, BSRR, BS3: Set);
        } else {
            write_reg!(stm32ral::gpio, cx.resources.gpiob, BSRR, BR3: Reset);
        }
        cx.schedule.foo(cx.scheduled + 1_000_000.cycles()).unwrap();
    }

    extern "C" {
        fn FLASH();
    }
};
