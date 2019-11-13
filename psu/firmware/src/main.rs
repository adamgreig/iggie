#![no_std]
#![no_main]

const V_SET: f32 = 250.0;
const V_LIM: f32 = 420.0;
const I_LIM: f32 = 0.100;

use panic_halt as _;
use rtfm::cyccnt::U32Ext;

pub mod hal;
pub mod telem;

use telem::ToBytes;

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
        #[init(telem::Telem::new())]
        telem: telem::Telem,
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

        // Set initial DAC level for current feedback
        dac.set_ch1(0);

        // Set initial DAC level for DCM detection
        // Measured 1.28V at Vq(adc) node in midpoint of falling edge at DCM
        // 1.28/3.30 * 4096 = 1589
        dac.set_ch2(1589);

        // Start ADC conversion
        adc.start(&dma, &mut cx.resources.adc_buf1, &mut cx.resources.adc_buf2);

        // Start HRTIM
        hrtim.enable();

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

    #[task(resources=[telem, usart1, dma1], schedule=[send_telem])]
    fn send_telem(cx: send_telem::Context) {
        let telem = cx.resources.telem;
        let dma = cx.resources.dma1;
        cx.resources.usart1.transmit(dma, &telem.to_bytes());
        cx.schedule.send_telem(cx.scheduled + 7_000_000.cycles()).unwrap();
    }

    #[task(resources=[dac, telem], schedule=[ramp_dac])]
    fn ramp_dac(cx: ramp_dac::Context) {
        static mut LEVEL: u16 = 0;
        let err = V_SET - cx.resources.telem.v_out;
        if err > 0.0 {
            if *LEVEL < 3400 {
                *LEVEL += 1;
            }
        } else {
            if *LEVEL > 10 {
                *LEVEL -= 1;
            }
        }
        cx.resources.dac.set_ch1(*LEVEL);
        cx.resources.telem.update_ref_i_q(*LEVEL);
        cx.schedule.ramp_dac(cx.scheduled + 80_000.cycles()).unwrap();
    }

    // Define an idle task with just NOPs to prevent the microcontroller entering sleep mode,
    // which would make attaching the debugger more annoying and has no benefit in this
    // mains-powered application.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // Run USART1 ISR to handle disabling DMA at end of transfer
    #[task(binds=USART1_EXTI25, resources=[usart1, dma1])]
    fn usart1(cx: usart1::Context) {
        cx.resources.usart1.isr(cx.resources.dma1);
    }

    // Run ADC ISR to handle new ADC data.
    #[task(binds=ADC1_2, resources=[adc, hrtim, gpio, telem, adc_buf1, adc_buf2])]
    fn adc1_2(cx: adc1_2::Context) {
        cx.resources.adc.isr();

        let telem = cx.resources.telem;
        telem.update_adc(*cx.resources.adc_buf1, *cx.resources.adc_buf2);

        // Check output voltage and current against limits.
        if telem.v_out >= V_LIM || telem.i_out >= I_LIM {
            if telem.v_out >= V_LIM {
                telem.set_fault(telem::FaultCode::VLim);
            } else if telem.i_out >= I_LIM {
                telem.set_fault(telem::FaultCode::ILim);
            }
            cx.resources.hrtim.disable();
            cx.resources.gpio.set_err_led(true);
        }
    }

    // Handle HRTIM fault: caused by SYSFLT or FLT2 (our nRUN input)
    #[task(binds=HRTIM_FLT, resources=[hrtim, gpio, telem])]
    fn hrtim_flt(cx: hrtim_flt::Context) {
        cx.resources.hrtim.flt_isr();
        cx.resources.telem.set_fault(telem::FaultCode::NoRun);
        cx.resources.gpio.set_err_led(true);
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};
