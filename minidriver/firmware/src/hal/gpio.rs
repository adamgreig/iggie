// Copyright 2019 Adam Greig
// Dual licensed under the Apache 2.0 and MIT licenses.

use stm32ral::gpio;
use stm32ral::{read_reg, write_reg, modify_reg};
use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u8)]
pub enum PinState {
    Low = 0,
    High = 1,
}

pub struct GPIO {
    p: gpio::Instance,
}

pub struct Pin<'a> {
    n: u8,
    mask: u32,
    port: &'a GPIO,
    instance: &'a gpio::Instance,
}

pub struct Pins<'a> {
    pub gk1: Pin<'a>,
    pub gk2: Pin<'a>,
    pub gk3: Pin<'a>,
    pub gk4: Pin<'a>,
    pub a0: Pin<'a>,
    pub a1: Pin<'a>,
    pub a2: Pin<'a>,
    pub a3: Pin<'a>,
    pub le: Pin<'a>,
    pub en: Pin<'a>,
    pub led: Pin<'a>,
    pub psu_run: Pin<'a>,
}

impl<'a> GPIO {
    pub fn new(p: gpio::Instance) -> Self {
        GPIO { p }
    }

    pub fn pin(&'a self, n: u8) -> Pin<'a> {
        assert!(n < 16);
        Pin { n, mask: 1 << n, port: self, instance: &self.p }
    }

    pub fn instance(&self) -> &gpio::Instance {
        &self.p
    }

    pub fn set_high(&'a self, n: u8) -> &Self {
        write_reg!(gpio, self.p, BSRR, 1 << n);
        self
    }

    pub fn set_low(&'a self, n: u8) -> &Self {
        write_reg!(gpio, self.p, BRR, 1 << n);
        self
    }

    pub fn toggle(&'a self, n: u8) -> &Self {
        let pin = (read_reg!(gpio, self.p, IDR) >> n) & 1;
        if pin == 1 {
            self.set_low(n)
        } else {
            self.set_high(n)
        }
    }

    pub fn set_mode(&'a self, n: u8, mode: u32) -> &Self {
        let offset = n * 2;
        let mask = 0b11 << offset;
        let val = (mode << offset) & mask;
        modify_reg!(gpio, self.p, MODER, |r| (r & !mask) | val);
        self
    }

    pub fn set_mode_input(&'a self, n: u8) -> &Self {
        self.set_mode(n, 0)
    }

    pub fn set_mode_output(&'a self, n: u8) -> &Self {
        self.set_mode(n, 1)
    }

    pub fn set_mode_alternate(&'a self, n: u8) -> &Self {
        self.set_mode(n, 2)
    }

    pub fn set_mode_analog(&'a self, n: u8) -> &Self {
        self.set_mode(n, 3)
    }

    pub fn set_otype(&'a self, n: u8, otype: u32) -> &Self {
        let offset = n;
        let mask = 0b1 << offset;
        let val = (otype << offset) & mask;
        modify_reg!(gpio, self.p, OTYPER, |r| (r & !mask) | val);
        self
    }

    pub fn set_otype_opendrain(&'a self, n: u8) -> &Self {
        self.set_otype(n, 1)
    }

    pub fn set_otype_pushpull(&'a self, n: u8) -> &Self {
        self.set_otype(n, 0)
    }

    pub fn set_ospeed(&'a self, n: u8, ospeed: u32) -> &Self {
        let offset = n * 2;
        let mask = 0b11 << offset;
        let val = (ospeed << offset) & mask;
        modify_reg!(gpio, self.p, OSPEEDR, |r| (r & !mask) | val);
        self
    }

    pub fn set_ospeed_verylow(&'a self, n: u8) -> &Self {
        self.set_ospeed(n, 0)
    }

    pub fn set_ospeed_low(&'a self, n: u8) -> &Self {
        self.set_ospeed(n, 1)
    }

    pub fn set_ospeed_high(&'a self, n: u8) -> &Self {
        self.set_ospeed(n, 2)
    }

    pub fn set_ospeed_veryhigh(&'a self, n: u8) -> &Self {
        self.set_ospeed(n, 3)
    }

    pub fn set_af(&'a self, n: u8, af: u32) -> &Self {
        if n < 8 {
            let offset = n * 4;
            let mask = 0b1111 << offset;
            let val = (af << offset) & mask;
            modify_reg!(gpio, self.p, AFRL, |r| (r & !mask) | val);
        } else {
            let offset = (n - 8) * 4;
            let mask = 0b1111 << offset;
            let val = (af << offset) & mask;
            modify_reg!(gpio, self.p, AFRH, |r| (r & !mask) | val);
        }
        self
    }

    pub fn set_pull(&'a self, n: u8, pull: u32) -> &Self {
        let offset = n * 2;
        let mask = 0b11 << offset;
        let val = (pull << offset) & mask;
        modify_reg!(gpio, self.p, PUPDR, |r| (r & !mask) | val);
        self
    }

    pub fn set_pull_floating(&'a self, n: u8) -> &Self {
        self.set_pull(n, 0)
    }

    pub fn set_pull_up(&'a self, n: u8) -> &Self {
        self.set_pull(n, 1)
    }

    pub fn set_pull_down(&'a self, n: u8) -> &Self {
        self.set_pull(n, 2)
    }

    pub fn get_idr(&'a self) -> u32 {
        read_reg!(gpio, self.p, IDR)
    }

    pub fn get_pin_idr(&'a self, n: u8) -> u32 {
        (self.get_idr() & (1 << n)) >> n
    }
}

impl<'a> Pin<'a> {
    pub fn pin_n(&self) -> u8 {
        self.n
    }

    pub fn instance(&self) -> &gpio::Instance {
        self.instance
    }

    pub fn set_high(&self) -> &Self {
        write_reg!(gpio, self.instance, BSRR, self.mask);
        self
    }

    pub fn set_low(&self) -> &Self {
        write_reg!(gpio, self.instance, BRR, self.mask);
        self
    }

    pub fn set_state(&self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        };
    }

    pub fn set_bool(&self, state: bool) {
        if state {
            self.set_high();
        } else {
            self.set_low();
        }
    }

    pub fn get_state(&self) -> PinState {
        match (read_reg!(gpio, self.instance, IDR) & self.mask) {
            0 => PinState::Low,
            _ => PinState::High,
        }
    }

    pub fn is_high(&self) -> bool {
        match self.get_state() {
            PinState::High => true,
            PinState::Low => false,
        }
    }

    pub fn is_low(&self) -> bool {
        match self.get_state() {
            PinState::Low => true,
            PinState::High => false,
        }
    }

    pub fn toggle(&'a self) -> &Self {
        self.port.toggle(self.n);
        self
    }

    pub fn set_mode_input(&'a self) -> &Self {
        self.port.set_mode_input(self.n);
        self
    }

    pub fn set_mode_output(&'a self) -> &Self {
        self.port.set_mode_output(self.n);
        self
    }

    pub fn set_mode_alternate(&'a self) -> &Self {
        self.port.set_mode_alternate(self.n);
        self
    }

    pub fn set_mode_analog(&'a self) -> &Self {
        self.port.set_mode_analog(self.n);
        self
    }

    pub fn set_otype_opendrain(&'a self) -> &Self {
        self.port.set_otype_opendrain(self.n);
        self
    }

    pub fn set_otype_pushpull(&'a self) -> &Self {
        self.port.set_otype_pushpull(self.n);
        self
    }

    pub fn set_ospeed_verylow(&'a self) -> &Self {
        self.port.set_ospeed_verylow(self.n);
        self
    }

    pub fn set_ospeed_low(&'a self) -> &Self {
        self.port.set_ospeed_low(self.n);
        self
    }

    pub fn set_ospeed_high(&'a self) -> &Self {
        self.port.set_ospeed_high(self.n);
        self
    }

    pub fn set_ospeed_veryhigh(&'a self) -> &Self {
        self.port.set_ospeed_veryhigh(self.n);
        self
    }

    pub fn set_af(&'a self, af: u32) -> &Self {
        self.port.set_af(self.n, af);
        self
    }

    pub fn set_pull_floating(&'a self) -> &Self {
        self.port.set_pull_floating(self.n);
        self
    }

    pub fn set_pull_up(&'a self) -> &Self {
        self.port.set_pull_up(self.n);
        self
    }

    pub fn set_pull_down(&'a self) -> &Self {
        self.port.set_pull_down(self.n);
        self
    }
}

impl<'a> Pins<'a> {
    /// Configure I/O pins
    pub fn setup(&self) {
        // Push-pull output on GK1..4 and A0..3.
        self.gk1.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.gk2.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.gk3.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.gk4.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.a0.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.a1.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.a2.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();
        self.a3.set_low().set_otype_pushpull().set_ospeed_veryhigh().set_mode_output();

        // Leave LE floating.
        self.le.set_mode_analog();

        // Push-pull output to LED, active high.
        self.led.set_low().set_otype_pushpull().set_ospeed_verylow().set_mode_output();

        // Push-pull output to PSU RUN line, active low.
        self.psu_run.set_high().set_otype_pushpull().set_ospeed_verylow().set_mode_output();

        // TIM1 AF output on EN.
        self.en.set_otype_pushpull().set_ospeed_veryhigh().set_af(2).set_mode_alternate();
    }

    pub fn set_col(&self, addr: u8) {
        self.a0.set_bool((addr & 0b0001) != 0);
        self.a1.set_bool((addr & 0b0010) != 0);
        self.a2.set_bool((addr & 0b0100) != 0);
        self.a3.set_bool((addr & 0b1000) != 0);
    }

    pub fn set_row(&self, row: u8) {
        self.gk1.set_bool(row == 1);
        self.gk2.set_bool(row == 2);
        self.gk3.set_bool(row == 3);
        self.gk4.set_bool(row == 4);
    }
}
