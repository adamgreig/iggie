#![no_std]
#![no_main]

use panic_halt as _;
use rtfm::cyccnt::U32Ext;

use stm32ral::{write_reg, modify_reg};

pub mod hal;
pub mod app;

use app::ToBytes;

#[rtfm::app(device=stm32ral::stm32f3::stm32f3x4, monotonic=rtfm::cyccnt::CYCCNT, peripherals=true)]
const APP: () = {
    struct Resources {
        // GPIOB is used by heartbeat thread to flash LED
        gpiob: stm32ral::gpio::Instance,
        // USART is used by telem thread to send telemetry
        usart1: hal::usart::USART,
        // DMA is used by USART and ADCs
        dma1: hal::dma::DMA,

        #[init(app::Telem::new())]
        telem: app::Telem,
    }

    #[init(spawn=[heartbeat, send_telem])]
    fn init(mut cx: init::Context) -> init::LateResources {
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Initialise device clocks
        let rcc = hal::rcc::RCC::new(cx.device.RCC, cx.device.Flash);
        rcc.setup();

        // Initialise USART for telemetry
        let usart = hal::usart::USART::new(cx.device.USART1);
        usart.setup();

        // Initialise DMA controller
        let dma = hal::dma::DMA::new(cx.device.DMA1);
        dma.setup();

        // Initialise GPIOs
        let gpioa = &cx.device.GPIOA;
        let gpiob = &cx.device.GPIOB;
        // Set PA8 to 0V otherwise it floats high
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Output);
        modify_reg!(stm32ral::gpio, gpioa, ODR, ODR8: 0);
        modify_reg!(stm32ral::gpio, gpioa, OSPEEDR, OSPEEDR8: HighSpeed);
        // Set both LEDs to outputs
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER3: Output, MODER4: Output);
        modify_reg!(stm32ral::gpio, gpiob, ODR, ODR3: 0, ODR4: 0);
        // Set PB6 to USART Tx (AF7)
        modify_reg!(stm32ral::gpio, gpiob, AFRL, AFRL6: AF7);
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER6: Alternate);

        // Prototyping: Set up TIM1 to generate very occasional short pulses on CH1
        let tim1 = &cx.device.TIM1;
        modify_reg!(stm32ral::tim1, tim1, CCMR1, CC1S: Output, OC1M: PwmMode1);
        modify_reg!(stm32ral::tim1, tim1, CCER, CC1P: 0, CC1E: 1);
        modify_reg!(stm32ral::tim1, tim1, BDTR, MOE: Enabled);
        write_reg!(stm32ral::tim1, tim1, ARR, 0x00F0);
        write_reg!(stm32ral::tim1, tim1, PSC, 8);
        write_reg!(stm32ral::tim1, tim1, CCR1, 10);
        //modify_reg!(stm32ral::tim1, tim1, CR1, CEN: Enabled);
        // Prototyping: connect GD to TIM1 CH1 (AF6)
        //modify_reg!(stm32ral::gpio, gpioa, AFRH, AFRH8: AF6);
        //modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Alternate);

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        // Start telem sender
        cx.spawn.send_telem().unwrap();

        // Release peripherals as late resources for use by other tasks
        init::LateResources { gpiob: cx.device.GPIOB, usart1: usart, dma1: dma }
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

    #[task(resources=[telem, usart1, dma1], schedule=[send_telem])]
    fn send_telem(cx: send_telem::Context) {
        let telem = cx.resources.telem.to_bytes();
        let dma = cx.resources.dma1;
        cx.resources.usart1.transmit(dma, &telem);
        cx.schedule.send_telem(cx.scheduled + 7_000_000.cycles()).unwrap();
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

    #[task(binds=USART1_EXTI25, resources=[usart1, dma1])]
    fn usart1(cx: usart1::Context) {
        cx.resources.usart1.isr(cx.resources.dma1);
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};
