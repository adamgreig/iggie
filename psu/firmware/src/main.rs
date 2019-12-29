#![no_std]
#![no_main]

// Control loop parameters

/// Setpoint voltage (V).
/// Typically 400V.
const V_SET: f32 = 370.0;

/// Overvoltage limit before a fault is triggered (V).
/// This has some filtering.
const V_LIM: f32 = 400.0;

/// Overcurrent limit before a fault is triggered (A).
/// This is slightly filtered.
const I_LIM: f32 = 0.100;

/// Minimum output voltage before a fault is triggered after timeout (V).
const V_MIN: f32 = 340.0;

/// Timeout after which VOut must be at least V_MIN (cycles).
/// Typically 380V.
const V_TIMEOUT: u32 = 500_000_000;

/// Minimum permitted input voltage (V).
const VIN_MIN: f32 = 20.0;

/// Maximum permitted input voltage (V).
const VIN_MAX: f32 = 30.0;

/// Maximum permitted input current (A).
const IIN_MAX: f32 = 3.0;

/// Maximum control signal. Absolute maximum is 4095.
const IREF_MAX: i16 = 3800;

/// Proportional gain
const K_P: f32 = 20.0;

/// Integral gain
const K_I: f32 = 30.0;

/// Derivative gain
const K_D: f32 = 8.0;

/// Limits on integral gain.
/// Since we expect the final control signal to be significantly integral based,
/// set a high limit sufficient to reach the maximum control value.
const I_MAX: f32 = (IREF_MAX as f32) / K_I;
const I_MIN: f32 = -I_MAX;

use core::panic::PanicInfo;
use cortex_m_rt::exception;
use rtfm::cyccnt::{Instant, Duration, U32Ext};

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
        #[init(false)]
        start_elapsed: bool,

        ctrl_pid: pid::PID,
        vout_kal: kalman::Kalman,
        iout_kal: kalman::Kalman,
        start_time: Instant,
    }

    #[init(spawn=[heartbeat, send_telem], resources=[adc_buf1, adc_buf2])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Enable DWT to allow use for software tasks
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Set up Kalman filters for Vout and Iout.
        // At 90.2kS/s, dt=4.2286µs
        let vout_kal = kalman::Kalman::new(80.0, 0.001, 1.10857e-6, 0.0);
        let iout_kal = kalman::Kalman::new(0.01, 0.002, 1.10857e-6, 0.0);

        // Set up PID control loop.
        // We run PID off TIM2 at 10kHz so dt=1/10e3
        let ctrl_pid = pid::PID::new(1.0/10e3, K_P, K_I, K_D, I_MIN, I_MAX);

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

        // Start telem sender
        cx.spawn.send_telem().unwrap();

        // Start periodic control loop interrupts
        tim2.start();

        // Start heartbeat task
        cx.spawn.heartbeat().unwrap();

        // Dummy start time since we can't const initialise it
        let start_time = Instant::now();

        // Release peripherals as late resources for use by other tasks
        init::LateResources {
            ctrl_pid, vout_kal, iout_kal, usart1, dma1, adc, gpio, dac, hrtim, tim2, start_time,
        }
    }

    // Heartbeat task runs 50 times a second.
    // Sets status LEDs and checks for nRUN.
    #[task(resources=[gpio, state, hrtim, start_time, start_elapsed], schedule=[heartbeat])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut LED_STATE: bool = false;
        *LED_STATE = !*LED_STATE;

        match cx.resources.state.fault_state {
            state::FaultState::Stopped => {
                cx.resources.gpio.set_400v_led(false);
                cx.resources.gpio.set_err_led(false);
                if cx.resources.gpio.get_run() {
                    *cx.resources.start_time = Instant::now();
                    *cx.resources.start_elapsed = false;
                    cx.resources.state.set_fault(state::FaultCode::NoFault);
                    cx.resources.state.set_state_running();
                    cx.resources.hrtim.enable();
                }
            },
            state::FaultState::Fault => {
                cx.resources.gpio.set_400v_led(false);
                cx.resources.gpio.set_err_led(true);
            },
            state::FaultState::Running => {
                if !*cx.resources.start_elapsed {
                    // Calling elapsed() after more than 2^31 cycles have passed (~30s)
                    // causes an unavoidable panic, so try to avoid that (!).
                    let timeout = Duration::from_cycles(V_TIMEOUT);
                    if cx.resources.start_time.elapsed() > timeout {
                        *cx.resources.start_elapsed = true;
                    }
                }
                cx.resources.gpio.set_400v_led(*LED_STATE);
                cx.resources.gpio.set_err_led(false);
            },
        }

        cx.schedule.heartbeat(cx.scheduled + 1_400_000.cycles()).unwrap();
    }

    #[task(resources=[state, usart1, dma1], schedule=[send_telem])]
    fn send_telem(cx: send_telem::Context) {
        let state = cx.resources.state;
        let dma = cx.resources.dma1;
        cx.resources.usart1.transmit(dma, &state.to_bytes());
        cx.schedule.send_telem(cx.scheduled + 7_000_000.cycles()).unwrap();
    }

    // Run control loop at fixed frequency on TIM2
    #[task(binds=TIM2, resources=[tim2, dac, state, ctrl_pid, vout_kal])]
    fn ctrl_loop(cx: ctrl_loop::Context) {
        // Control loop output ranges from 0 to 4096 and sets the I_Q limit reference.
        // 4096 corresponds to 3.3V at the comparator which corresponds to 6.47A.
        // We limit to IREF_MAX=3800 -> 3.06V -> 6.0A, our design point peak current.

        let (vout, dvout) = cx.resources.vout_kal.get();

        let pid = cx.resources.ctrl_pid;

        match cx.resources.state.fault_state {
            state::FaultState::Running => {
                // When running, compute PID update
                let action = pid.control_step(V_SET, vout, dvout) as i16;
                // Clamp action to bounds
                let action = if action < 0 { 0 }
                             else if action > IREF_MAX { IREF_MAX }
                             else { action };
                // Update DAC and telemetry
                cx.resources.dac.set_ch1(action as u16);
                cx.resources.state.update_ref_i_q(action as u16);
            },

            state::FaultState::Stopped | state::FaultState::Fault => {
                // When stopped or faulted, reset the controller and clear the DAC.
                pid.zero();
                cx.resources.dac.set_ch1(0);
                cx.resources.state.update_ref_i_q(0);
            },
        }

        // Update integrator in state
        cx.resources.state.update_pid_i(pid.get_i());

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
    #[task(binds=ADC1_2, resources=[adc, hrtim, state, adc_buf1, adc_buf2,
                                    vout_kal, iout_kal, start_elapsed])]
    fn adc1_2(cx: adc1_2::Context) {
        cx.resources.adc.isr();

        let state = cx.resources.state;
        state.update_adc(*cx.resources.adc_buf1, *cx.resources.adc_buf2);

        // Update Kalman filters
        cx.resources.vout_kal.predict();
        cx.resources.vout_kal.update(state.v_out);
        cx.resources.iout_kal.predict();
        cx.resources.iout_kal.update(state.i_out);

        // Check output voltage and current against limits.
        let (vout, _) = cx.resources.vout_kal.get();
        let (iout, _) = cx.resources.iout_kal.get();

        state.v_out = vout;
        state.i_out = iout;

        if state.fault_state == state::FaultState::Running {
            let mut fault = false;
            if vout >= V_LIM {
                state.set_fault(state::FaultCode::VLim);
                fault = true;
            }
            if iout >= I_LIM {
                state.set_fault(state::FaultCode::ILim);
                fault = true;
            }
            if state.v_in <= VIN_MIN {
                state.set_fault(state::FaultCode::VInLow);
                fault = true;
            } else if state.v_in >= VIN_MAX {
                state.set_fault(state::FaultCode::VInHigh);
                fault = true;
            }
            if state.i_in >= IIN_MAX {
                state.set_fault(state::FaultCode::IInHigh);
                fault = true;
            }
            if state.fault_state == state::FaultState::Running {
                if *cx.resources.start_elapsed && state.v_out <= V_MIN {
                    state.set_fault(state::FaultCode::NoVOut);
                    fault = true;
                }
            }
            if fault {
                state.set_state_fault();
                cx.resources.hrtim.disable();
            }
        }
    }

    // Handle HRTIM fault: caused by SYSFLT or FLT2 (our nRUN input)
    #[task(binds=HRTIM_FLT, resources=[hrtim, state])]
    fn hrtim_flt(cx: hrtim_flt::Context) {
        cx.resources.hrtim.flt_isr();
        cx.resources.state.set_fault(state::FaultCode::NoRun);
        cx.resources.state.set_state_stopped();
    }

    // We require at least one interrupt vector defined here per software task
    // priority level in use, for RTFM to co-opt for software task execution.
    extern "C" {
        fn FLASH();
    }
};

#[panic_handler]
unsafe fn panic(_info: &PanicInfo) -> ! {
    // On panic, manually trigger fault and hard loop.
    hal::hrtim::HRTIM::global_disable();
    hal::gpio::GPIO::global_set_err_led();
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // On hard fault, manually trigger fault and hard loop.
    hal::hrtim::HRTIM::global_disable();
    hal::gpio::GPIO::global_set_err_led();
    loop {
        cortex_m::asm::nop();
    }
}
