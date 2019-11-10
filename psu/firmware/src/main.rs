#![no_std]
#![no_main]

use panic_halt as _;
use rtfm::cyccnt::U32Ext;

pub mod hal;
pub mod app;

use app::ToBytes;

#[rtfm::app(device=stm32ral::stm32f3::stm32f3x4, monotonic=rtfm::cyccnt::CYCCNT, peripherals=true)]
const APP: () = {
    struct Resources {
        // GPIO is used by heartbeat thread to flash LED
        gpio: hal::gpio::GPIO,
        // USART is used by telem thread to send telemetry
        usart1: hal::usart::USART,
        // DMA is used by USART and ADCs
        dma1: hal::dma::DMA,
        // ADC is used to run control loops and update telem
        adc: hal::adc::ADC,
        // DAC is used to adjust comparator thresholds
        dac: hal::dac::DAC,
        // HRTIM
        hrtim: hal::hrtim::HRTIM,

        #[init([0; 4])]
        adc_buf1: [u16; 4],
        #[init([0; 2])]
        adc_buf2: [u16; 2],
        #[init(app::Telem::new())]
        telem: app::Telem,
    }

    #[init(spawn=[heartbeat, send_telem, ramp_dac], resources=[adc_buf1, adc_buf2])]
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

        // Initialise comparators
        let comp = hal::comp::Comp::new(cx.device.COMP);
        comp.setup();

        // Initialise DAC
        let dac = hal::dac::DAC::new(cx.device.DAC1);
        dac.setup();

        // Initialise HRTIM
        let hrtim = hal::hrtim::HRTIM::new(
            cx.device.HRTIM_Master, cx.device.HRTIM_TIMA, cx.device.HRTIM_Common);
        hrtim.setup();

        // Initialise GPIOs
        let gpio = hal::gpio::GPIO::new(cx.device.GPIOA, cx.device.GPIOB);
        gpio.setup();

        // Prototyping: Set up TIM1 to generate very occasional short pulses on CH1
        /*
        let tim1 = &cx.device.TIM1;
        modify_reg!(stm32ral::tim1, tim1, CCMR1, CC1S: Output, OC1M: PwmMode1);
        modify_reg!(stm32ral::tim1, tim1, CCER, CC1P: 0, CC1E: 1);
        modify_reg!(stm32ral::tim1, tim1, BDTR, MOE: Enabled);
        write_reg!(stm32ral::tim1, tim1, ARR, 0x4FF0);
        write_reg!(stm32ral::tim1, tim1, PSC, 8);
        write_reg!(stm32ral::tim1, tim1, CCR1, 10);
        modify_reg!(stm32ral::tim1, tim1, CR1, CEN: Enabled);
        */

        // Set a dummy DAC level for prototyping
        dac.set_ch1(2200);

        // Start ADC conversion
        adc.start(&dma, &mut cx.resources.adc_buf1, &mut cx.resources.adc_buf2);

        // Start HRTIM
        hrtim.start();

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        // Start telem sender
        cx.spawn.send_telem().unwrap();

        cx.spawn.ramp_dac().unwrap();

        // Release peripherals as late resources for use by other tasks
        init::LateResources { usart1: usart, dma1: dma, adc, gpio, dac, hrtim }
    }

    // Heartbeat task runs 10 times a second and toggles an LED
    #[task(resources=[gpio], schedule=[heartbeat])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut STATE: bool = false;
        *STATE = !*STATE;
        cx.resources.gpio.set_400v_led(*STATE);
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

    #[task(resources=[dac, telem], schedule=[ramp_dac])]
    fn ramp_dac(cx: ramp_dac::Context) {
        static mut LEVEL: u16 = 0;
        const SETPOINT: f32 = 80.0;
        let err = SETPOINT - cx.resources.telem.v_out;
        if err > 0.0 {
            if *LEVEL < 3400 {
                *LEVEL += 1;
            }
        } else {
            if *LEVEL > 1000 {
                *LEVEL -= 1;
            }
        }
        cx.resources.dac.set_ch1(*LEVEL);
        cx.resources.telem.ref_i_q = *LEVEL;
        cx.schedule.ramp_dac(cx.scheduled + 80_000.cycles()).unwrap();
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
