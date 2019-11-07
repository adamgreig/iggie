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
        // ADC is used to run control loops and update telem
        adc: hal::adc::ADC,

        #[init([0; 4])]
        adc_buf1: [u16; 4],
        #[init([0; 2])]
        adc_buf2: [u16; 2],
        #[init(app::Telem::new())]
        telem: app::Telem,
    }

    #[init(spawn=[heartbeat, send_telem], resources=[adc_buf1, adc_buf2])]
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

        // Initialise ADCs
        let mut adc = hal::adc::ADC::new(cx.device.ADC1, cx.device.ADC2);
        adc.setup();

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
        // Set PA0, 1, 2, 3, 6, 7 to analogue input
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER0: Analog, MODER1: Analog, MODER2: Analog,
                                                  MODER3: Analog, MODER6: Analog, MODER7: Analog);

        // Prototyping: Set up TIM1 to generate very occasional short pulses on CH1
        let tim1 = &cx.device.TIM1;
        modify_reg!(stm32ral::tim1, tim1, CCMR1, CC1S: Output, OC1M: PwmMode1);
        modify_reg!(stm32ral::tim1, tim1, CCER, CC1P: 0, CC1E: 1);
        modify_reg!(stm32ral::tim1, tim1, BDTR, MOE: Enabled);
        write_reg!(stm32ral::tim1, tim1, ARR, 0x0FF0);
        write_reg!(stm32ral::tim1, tim1, PSC, 8);
        write_reg!(stm32ral::tim1, tim1, CCR1, 10);
        //modify_reg!(stm32ral::tim1, tim1, CR1, CEN: Enabled);
        // Prototyping: connect GD to TIM1 CH1 (AF6)
        //modify_reg!(stm32ral::gpio, gpioa, AFRH, AFRH8: AF6);
        //modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Alternate);

        // Start ADC conversion
        adc.start(&dma, &mut cx.resources.adc_buf1, &mut cx.resources.adc_buf2);

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        // Start telem sender
        cx.spawn.send_telem().unwrap();

        // Release peripherals as late resources for use by other tasks
        init::LateResources { gpiob: cx.device.GPIOB, usart1: usart, dma1: dma, adc: adc }
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

    #[task(resources=[telem, usart1, dma1, adc_buf1, adc_buf2], schedule=[send_telem])]
    fn send_telem(cx: send_telem::Context) {
        let telem = cx.resources.telem;
        let dma = cx.resources.dma1;
        telem.update_adc(*cx.resources.adc_buf1, *cx.resources.adc_buf2);
        cx.resources.usart1.transmit(dma, &telem.to_bytes());
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

    #[task(binds=ADC1_2, resources=[adc])]
    fn adc1_2(cx: adc1_2::Context) {
        cx.resources.adc.isr();
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};
