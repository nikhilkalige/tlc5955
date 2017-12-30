//! no_std driver for TLC5955
//! A 48 channel PWM driver

#![allow(dead_code)]
#![deny(missing_docs)]
//#![deny(warnings)]
#![feature(unsize)]
#![feature(conservative_impl_trait)]
#![feature(iterator_step_by)]
#![no_std]

extern crate embedded_hal as hal;

use core::marker::Unsize;
use core::marker::PhantomData;

use hal::spi::{self, DmaRead, DmaReadWrite, DmaWrite, FullDuplex, Mode, Phase, Polarity};
use hal::dma::Transfer;
use hal::blocking::spi::transfer;
use hal::digital::OutputPin;
use hal::Pwm;


/// TLC5955 Driver
/// Needs constant generics to fix the number of chipsets
pub struct Tlc5955<B, SPI, LATCH, PWM> {
    spi: SPI,
    latch: LATCH,
    gsclk: PWM,
    count: u8,
    chipsets: [Chipset; 4],
    data: PhantomData<B>,
}

const CHIP_REGISTER_LENGTH: u16 = 769;
const CHANNELS_PER_CHIP: u8 = 48;

const DC_BITS: u8 = 7;
const MC_BITS: u8 = 3;
const BC_BITS: u8 = 7;
const FUNCTION_BITS: u8 = 5;
const CONTROL_MODE: u8 = 0x96;

const MC_REGISTER_OFFSET: u16 = 23;
const BC_REGISTER_OFFSET: u16 = 23;
const CONTROL_MODE_OFFSET: u16 = 760;

// Sane defaults
/// asdf
pub const DC_DATA: u8 = 0x6A;
/// asdf
pub const MC_DATA: u8 = 5;
/// asdf
pub const BC_DATA: u8 = 110;

/// Contents of each chip
#[derive(Copy, Clone)]
struct Chipset {
    function: u8,
    brightness: [u8; 3],
    max_current: [u8; 3],
}

impl Default for Chipset {
    fn default() -> Chipset {
        Chipset {
            function: 0x0B,
            brightness: [BC_DATA; 3],
            max_current: [MC_DATA; 3],
        }
    }
}

impl<B, SPI, LATCH, PWM> Tlc5955<B, SPI, LATCH, PWM>
where
    SPI: FullDuplex<u8> + DmaWrite<B, u8> + DmaRead<u8> + DmaReadWrite<u8>,
    LATCH: OutputPin,
    PWM: Pwm,
    B: Unsize<[u8]> + 'static
{
    /// Creates a new driver for `4` chips in series
    pub fn new(spi: SPI, latch: LATCH, gsclk: PWM, _count: u8) -> Self {
        let mut tlc = Tlc5955 {
            spi,
            latch,
            gsclk,
            count: 4,
            chipsets: [Default::default(); 4],
            data: PhantomData,
        };

        const BYTES: u16 = 4 * CHIP_REGISTER_LENGTH / 8;
        let mut buffer: [u8; BYTES as usize] = [0; BYTES as usize];

        for chip_no in 0..tlc.count {
            tlc.set_dot_correction_all(&mut buffer, chip_no, DC_BITS, DC_BITS, DC_BITS);
        }

        tlc.fill_control_data(&mut buffer);
        tlc.blocking_write(&mut buffer);

        tlc
    }

    /// Set chip function configuration bits
    pub fn set_function_data(
        &mut self,
        chip: u8,
        dsprpt: bool,
        tmgrst: bool,
        rfresh: bool,
        espwm: bool,
        lsdvlt: bool,
    ) {
        let mut data: u8 = 0;
        data |= if dsprpt { 1 << 0 } else { 0 };
        data |= if tmgrst { 1 << 0 } else { 0 };
        data |= if rfresh { 1 << 0 } else { 0 };
        data |= if espwm { 1 << 0 } else { 0 };
        data |= if lsdvlt { 1 << 0 } else { 0 };

        self.chipsets[chip as usize].function = data;
    }

    /// Set maximum allowed current
    pub fn set_max_current(&mut self, chip: u8, red: u8, green: u8, blue: u8) {
        self.chipsets[chip as usize].max_current = [red, green, blue];
    }

    /// Set maximum allowed brightness
    pub fn set_brightness_current(&mut self, chip: u8, red: u8, green: u8, blue: u8) {
        self.chipsets[chip as usize].brightness = [red, green, blue];
    }

    /// Set dot correction for a particular LED
    pub fn set_dot_correction(
        &mut self,
        buffer: &mut [u8],
        chip: u8,
        led: u8,
        red: u8,
        green: u8,
        blue: u8,
    ) {
        debug_assert!(
            (led < 48) && (buffer.len() as u16 >= (chip as u16 + 1) * CHIP_REGISTER_LENGTH)
        );

        let mut next_size: usize;
        let pos: usize = Self::calculate_bit_position(chip, led);

        next_size = add_bits(buffer, red as u32, 8, pos);
        next_size = add_bits(buffer, green as u32, 8, next_size);
        add_bits(buffer, blue as u32, 8, next_size);
    }

    /// Set dot correction for an entire chip
    pub fn set_dot_correction_all(
        &mut self,
        buffer: &mut [u8],
        chip: u8,
        red: u8,
        green: u8,
        blue: u8,
    ) {
        debug_assert!(buffer.len() as u16 >= (chip as u16 + 1) * CHIP_REGISTER_LENGTH);

        for led_no in 0..CHANNELS_PER_CHIP {
            self.set_dot_correction(buffer, chip, led_no, red, green, blue);
        }
    }

    fn fill_control_data(&mut self, buffer: &mut [u8]) {
        debug_assert!(buffer.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH);

        let mut pos: usize;
        for chip_num in 0..self.count {
            pos = (chip_num as u16 * CHIP_REGISTER_LENGTH) as usize;

            for _index in 0..3 {
                pos = add_bits(
                    buffer,
                    MC_DATA as u32,
                    MC_BITS as u32,
                    pos + MC_REGISTER_OFFSET as usize,
                );
            }

            for _index in 0..3 {
                pos = add_bits(
                    buffer,
                    BC_DATA as u32,
                    BC_BITS as u32,
                    pos + BC_REGISTER_OFFSET as usize,
                );
            }

            add_bits(
                buffer,
                self.chipsets[chip_num as usize].function as u32,
                FUNCTION_BITS as u32,
                pos,
            );
            pos = add_bits(
                buffer,
                CONTROL_MODE as u32,
                8,
                (chip_num as u16 * CHIP_REGISTER_LENGTH + CONTROL_MODE_OFFSET) as usize,
            );

            // Write last to 1
            add_bits(buffer, 1, 1, pos);
        }
    }

    fn blocking_write(&mut self, buffer: &mut [u8]) {
        debug_assert!(buffer.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH);
        transfer(&mut self.spi, buffer);
    }

    pub fn update(&mut self, bytes: &'static mut B) -> impl Transfer
    where
        B: Unsize<[u8]>,
    {
        {
            let slice: &mut [u8] = bytes;
            debug_assert!(slice.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH);
        }
        self.spi.send_dma(bytes)
    }

    /// Calculate bit position
    fn calculate_bit_position(chip: u8, led: u8) -> usize {
        (chip as u16 * CHIP_REGISTER_LENGTH + (led * 8) as u16) as usize
    }
}

/// Insert value `val` with `size` bits at position `pos` in the `buffer`
fn add_bits(buffer: &mut [u8], mut val: u32, mut size: u32, mut pos: usize) -> usize {
    let len = buffer.len();
    while size > 0 {
        if (val & 1) > 0 {
            buffer[len - 1 - (pos >> 3)] |= 1 << (pos & 7);
        }
        val >>= 1;
        pos += 1;
        size -= 1;
    }
    return pos;
}
