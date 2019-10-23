#![no_std]
#![no_main]

use panic_halt as _;
use rtfm::cyccnt::U32Ext;

use stm32ral::{write_reg, modify_reg};

pub mod hal;

#[rtfm::app(device=stm32ral::stm32f3::stm32f3x4, monotonic=rtfm::cyccnt::CYCCNT, peripherals=true)]
const APP: () = {
    struct Resources {
        gpiob: stm32ral::gpio::Instance,
    }

    #[init(spawn=[heartbeat])]
    fn init(mut cx: init::Context) -> init::LateResources {
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Initialise device clocks
        let rcc = hal::rcc::RCC::new(cx.device.RCC, cx.device.Flash);
        rcc.setup();

        // Initialise GPIOs
        let gpioa = &cx.device.GPIOA;
        let gpiob = &cx.device.GPIOB;
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Output);
        modify_reg!(stm32ral::gpio, gpioa, ODR, ODR8: 0);
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER3: Output, MODER4: Output);
        modify_reg!(stm32ral::gpio, gpiob, ODR, ODR3: 0, ODR4: 0);

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        init::LateResources { gpiob: cx.device.GPIOB }
    }

    // Heartbeat task runs 10 times a second and toggles an LED
    #[task(resources=[gpiob], schedule=[heartbeat])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut STATE: bool = false;
        *STATE = !*STATE;
        if *STATE {
            write_reg!(stm32ral::gpio, cx.resources.gpiob, BSRR, BS3: Set);
        } else {
            write_reg!(stm32ral::gpio, cx.resources.gpiob, BSRR, BR3: Reset);
        }
        cx.schedule.heartbeat(cx.scheduled + 7_000_000.cycles()).unwrap();
    }

    // Manually define an idle task with just NOPs to prevent the microcontroller
    // entering sleep mode, which makes attaching the debugger more annoying but
    // has no benefit in this mains-powered application.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};
