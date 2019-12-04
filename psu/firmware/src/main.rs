#![no_std]
#![no_main]

// Control loop parameters

/// Setpoint voltage. Typically 400V.
const V_SET: f32 = 400.0;
/// Overvoltage limit before a fault is triggered.
/// This has some filtering. Typically 440V.
const V_LIM: f32 = 440.0;
/// Overcurrent limit before a fault is triggered.
/// This is slightly filtered. Typically 0.100A.
const I_LIM: f32 = 0.100;

/// Maximum control signal.
const IREF_MAX: i16 = 3800;

/// Proportional gain.
/// We know that roughly 3000 counts on the output gives 400V for a moderate load,
/// suggesting K_P near 3000/400. Round down somewhat to reduce overshoot.
const K_P: f32 = 5.0;
/// Integral gain.
/// We expect this to make up a good proportion of the final control signal,
/// so set to near K_P.
const K_I: f32 = 4.0;
/// Derivative gain.
/// TBC.
const K_D: f32 = 1.0;
/// Limits on integral gain.
/// Since we expect the final control signal to be significantly integral based,
/// set a high limit sufficient to reach the maximum control value.
const I_MIN: f32 = 0.0;
const I_MAX: f32 = (IREF_MAX as f32) / K_I;
const I_THR: f32 = 100.0;

use panic_halt as _;
use rtfm::cyccnt::U32Ext;

pub mod hal;
pub mod state;
pub mod pid;
pub mod kalman;

use state::ToBytes;

#[rtfm::app(device=stm32ral::stm32f3::stm32f3x4, monotonic=rtfm::cyccnt::CYCCNT, peripherals=true)]
const APP: () = {
    struct Resources {
        // GPIO is used by heartbeat thread to flash LED
        gpio: hal::gpio::GPIO,
        // USART is used by telem thread to send telemetry
        usart1: hal::usart::USART,
        // DMA is used by USART and ADCs
        dma1: hal::dma::DMA,
        // ADC is used to update feedback values and update telem
        adc: hal::adc::ADC,
        // DAC is used to adjust comparator thresholds
        dac: hal::dac::DAC,
        // HRTIM runs the switch output
        hrtim: hal::hrtim::HRTIM,
        // TIM2 generates periodic interrupts for control loop operation
        tim2: hal::tim2::TIM2,

        #[init([0; 4])]
        adc_buf1: [u16; 4],
        #[init([0; 2])]
        adc_buf2: [u16; 2],
        #[init(state::State::new())]
        state: state::State,

        ctrl_pid: pid::PID,
        vout_kal: kalman::Kalman,
        iout_kal: kalman::Kalman,
    }

    #[init(spawn=[heartbeat, send_telem], resources=[adc_buf1, adc_buf2])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Enable DWT to allow use for software tasks
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Set up Kalman filters for Vout and Iout.
        // At 236.5kS/s, dt=4.2286µs
        let vout_kal = kalman::Kalman::new(2.0, 0.5, 4.2286e-6, 0.0);
        let iout_kal = kalman::Kalman::new(0.001, 0.001, 4.2286e-6, 0.0);

        // Set up PID control loop.
        // We run PID off TIM2 at 10kHz so dt=1/10e3
        let ctrl_pid = pid::PID::new(1.0/10e3, K_P, K_I, K_D, I_MIN, I_MAX, I_THR);

        // Initialise device clocks
        let rcc = hal::rcc::RCC::new(cx.device.RCC, cx.device.Flash);
        rcc.setup();

        // Initialise USART for telemetry
        let usart1 = hal::usart::USART::new(cx.device.USART1);
        usart1.setup();

        // Initialise DMA controller
        let dma1 = hal::dma::DMA::new(cx.device.DMA1);
        dma1.setup();

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

        // Initialise TIM2
        let tim2 = hal::tim2::TIM2::new(cx.device.TIM2);
        tim2.setup();

        // Set initial DAC level for current feedback
        dac.set_ch1(0);

        // Set initial DAC level for DCM detection
        // Measured 1.28V at Vq(adc) node in midpoint of falling edge at DCM
        // 1.28/3.30 * 4096 = 1589
        dac.set_ch2(1589);

        // Start ADC conversion
        adc.start(&dma1, &mut cx.resources.adc_buf1, &mut cx.resources.adc_buf2);

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        // Start telem sender
        cx.spawn.send_telem().unwrap();

        // Start periodic control loop interrupts
        tim2.start();

        // Start HRTIM
        hrtim.enable();

        // Release peripherals as late resources for use by other tasks
        init::LateResources {
            ctrl_pid, vout_kal, iout_kal, usart1, dma1, adc, gpio, dac, hrtim, tim2
        }
    }

    // Heartbeat task runs 10 times a second and toggles an LED
    #[task(resources=[gpio], schedule=[heartbeat])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut STATE: bool = false;
        *STATE = !*STATE;
        cx.resources.gpio.set_400v_led(*STATE);
        cx.schedule.heartbeat(cx.scheduled + 7_000_000.cycles()).unwrap();
    }

    #[task(resources=[state, usart1, dma1], schedule=[send_telem])]
    fn send_telem(cx: send_telem::Context) {
        let state = cx.resources.state;
        let dma = cx.resources.dma1;
        cx.resources.usart1.transmit(dma, &state.to_bytes());
        cx.schedule.send_telem(cx.scheduled + 7_000_000.cycles()).unwrap();
    }

    // Run control loop at fixed frequency on TIM2
    #[task(binds=TIM2, resources=[tim2, dac, state])]
    fn ctrl_loop(cx: ctrl_loop::Context) {
        // Output variable is IREF, from 0 to 4096, sets the current limit reference.
        // 4096 corresponds to 3.3V at the comparator which corresponds to 6.47A.
        // We limit to IREF_MAX=3800 -> 3.06V -> 6.0A, our design point peak current.
        static mut IREF: i16 = 0;

        // Determine error from constant setpoint compared to latest ADC reading of output.
        let err = V_SET - cx.resources.state.v_out;

        // Find proportional correction
        const K_P: f32 = 0.001;
        let k = K_P * err;
        let k = k as i16;
        *IREF += k;

        // Saturate IREF at limits
        if *IREF > IREF_MAX {
            *IREF = IREF_MAX;
        } else if *IREF < 0 {
            *IREF = 0;
        }

        // Update DAC and telemetry
        cx.resources.dac.set_ch1(*IREF as u16);
        cx.resources.state.update_ref_i_q(*IREF as u16);

        // Clear interrupt pending flag
        cx.resources.tim2.isr();
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
    #[task(binds=ADC1_2, resources=[adc, hrtim, gpio, state, adc_buf1, adc_buf2])]
    fn adc1_2(cx: adc1_2::Context) {
        cx.resources.adc.isr();

        let state = cx.resources.state;
        state.update_adc(*cx.resources.adc_buf1, *cx.resources.adc_buf2);

        // Check output voltage and current against limits.
        if state.v_out >= V_LIM || state.i_out >= I_LIM {
            if state.v_out >= V_LIM {
                state.set_fault(state::FaultCode::VLim);
            } else if state.i_out >= I_LIM {
                state.set_fault(state::FaultCode::ILim);
            }
            cx.resources.hrtim.disable();
            cx.resources.gpio.set_err_led(true);
        }
    }

    // Handle HRTIM fault: caused by SYSFLT or FLT2 (our nRUN input)
    #[task(binds=HRTIM_FLT, resources=[hrtim, gpio, state])]
    fn hrtim_flt(cx: hrtim_flt::Context) {
        cx.resources.hrtim.flt_isr();
        cx.resources.state.set_fault(state::FaultCode::NoRun);
        cx.resources.gpio.set_err_led(true);
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};
