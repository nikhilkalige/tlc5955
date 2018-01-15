//! no_std driver for TLC5955
//! A 48 channel PWM driver

#![allow(dead_code)]
#![deny(missing_docs)]
//#![deny(warnings)]
#![feature(const_fn)]
#![feature(unsize)]
#![feature(conservative_impl_trait)]
#![feature(iterator_step_by)]
#![no_std]

extern crate embedded_hal as hal;
extern crate logger;
#[macro_use]
extern crate nb;

use core::marker::Unsize;
use core::marker::PhantomData;

use hal::spi::{DmaWrite};
use hal::dma::Transfer;
use hal::blocking::spi::FullDuplex as BlockingFullDuplex;
use hal::digital::OutputPin;
use hal::PwmPin;

pub mod take_cell;

use take_cell::{MapCell, TakeCell};
use logger::Logger;

/// Trait converts from output pins to spi and vice versa.
pub trait SpiWriteBit
{
    ///asdf
    fn write_bit(&self, bit: u8);
}

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Setup Error
    Setup,
    /// SPI bus error
    Spi(E),
}

/// Track the transmission of data to the chips
#[derive(Debug, Copy, Clone)]
enum TransmitState {
    /// Idle state
    Idle,
    /// Sending data, tracks the current chip and type of data transfer
    Transmit(DataTransferType, u8),
}

/// Track the transmission of data to the chips
#[derive(Debug, Copy, Clone)]
enum DataTransferType {
    /// Control data
    Control,
    /// Grayscale data
    Grayscale,
}

/// TLC5955 Driver
/// Needs constant generics to fix the number of chipsets
pub struct Tlc5955<'a, BW, SPI, LATCH, PWM, LOGGER: 'a>
{
    spi: MapCell<SPI>,
    bit_writer: BW,
    latch: LATCH,
    gsclk: PWM,
    count: u8,
    chipsets: [Chipset; 4],
    log: &'a mut LOGGER,
    state: TransmitState,
    data: Option<&'static mut [u8]>,
    // data: TakeCell<'static, [u8]>,
}

const CHIP_REGISTER_LENGTH: u16 = 768;
const CHIP_REGISTER_LENGTH_BYTES: u16 = (CHIP_REGISTER_LENGTH / 8);
const BUFFER_FOR_4_CHIPS: u16 = CHIP_REGISTER_LENGTH_BYTES * 4;
const CHANNELS_PER_CHIP: u8 = 48;

const DC_BITS: u8 = 7;
const MC_BITS: u8 = 3;
const BC_BITS: u8 = 7;
const FUNCTION_BITS: u8 = 5;
const CONTROL_MODE: u8 = 0x96;

const MC_REGISTER_OFFSET: u16 = 336;
const BC_REGISTER_OFFSET: u16 = 345;
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

impl<'a, BW, SPI, LATCH, PWM, LOGGER> Tlc5955<'a, BW, SPI, LATCH, PWM, LOGGER>
where
    // SPI: BlockingFullDuplex<u8> + DmaWrite<B, u8> + DmaRead<u8> + DmaReadWrite<u8>,
    SPI: BlockingFullDuplex<u8> + DmaWrite<[u8; CHIP_REGISTER_LENGTH_BYTES as usize], u8>,
    BW: SpiWriteBit,
    LATCH: OutputPin,
    PWM: PwmPin<Duty = u16>,
    LOGGER: 'a + Logger,
    // B: Unsize<[u8]> + 'static,
{
    /// Creates a new driver for `4` chips in series
    pub fn new(
        spi: SPI,
        bit_writer: BW,
        latch: LATCH,
        gsclk: PWM,
        log: &'a mut LOGGER,
        _count: u8,
    ) -> Result<Self, Error<SPI::Error>> {
        log.log_string("About to start greatest thing in the world\n");

        let mut tlc = Tlc5955 {
            spi: MapCell::new(spi),
            latch,
            bit_writer,
            gsclk,
            count: 4,
            chipsets: [Default::default(); 4],
            // data: PhantomData,
            log,
            state: TransmitState::Idle,
            data: None,
            // data: TakeCell::empty(),
        };

        tlc.latch.set_low();

        let duty = tlc.gsclk.get_max_duty() / 2;
        tlc.gsclk.set_duty(duty);
        tlc.gsclk.enable();

        tlc.setup()?;

        Ok(tlc)
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
            (led < 48) && (buffer.len() as u16 >= (chip as u16 + 1) * CHIP_REGISTER_LENGTH_BYTES)
        );

        let mut chip_chunk = buffer.chunks_mut(CHIP_REGISTER_LENGTH_BYTES as usize).nth(chip as usize);
        if let Some(chip_buffer) = chip_chunk.as_mut() {
            let offset: usize = (led as usize * DC_BITS as usize);
            let mut next_pos: usize;

            next_pos = add_bits(chip_buffer, red as u32, DC_BITS as u32, offset);
            next_pos = add_bits(chip_buffer, green as u32, DC_BITS as u32, next_pos);
            add_bits(chip_buffer, blue as u32, DC_BITS as u32, next_pos);
        }
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
        debug_assert!(buffer.len() as u16 >= (chip as u16 + 1) * CHIP_REGISTER_LENGTH_BYTES);

        for led_no in 0..CHANNELS_PER_CHIP {
            self.set_dot_correction(buffer, chip, led_no, red, green, blue);
        }
    }

    fn fill_control_data(&mut self, buffer: &mut [u8]) {
        debug_assert!(buffer.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH_BYTES);

        let mut pos: usize;
        let mut chip_no: usize = 0;

        for chip_buffer in buffer.chunks_mut(CHIP_REGISTER_LENGTH_BYTES as usize) {
            pos = MC_REGISTER_OFFSET as usize;
            for _ in 0..3 {
                pos = add_bits(chip_buffer, MC_DATA as u32, MC_BITS as u32, pos);
            }

            pos = BC_REGISTER_OFFSET as usize;
            for _ in 0..3 {
                pos = add_bits(chip_buffer, BC_DATA as u32, BC_BITS as u32, pos);
            }

            add_bits(chip_buffer, self.chipsets[chip_no].function as u32, FUNCTION_BITS as u32, pos);
            add_bits(chip_buffer, CONTROL_MODE as u32, 8, CONTROL_MODE_OFFSET as usize);
        }
    }

    fn blocking_write(&mut self, buffer: &mut [u8], transfer_type: DataTransferType) {
        debug_assert!(buffer.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH_BYTES);

        for chip_buffer in buffer.chunks_mut(CHIP_REGISTER_LENGTH_BYTES as usize) {
            self.transmit_msb(transfer_type);
            self.spi.map(|spi| {
                spi.transfer(chip_buffer);
            });

        }

        // Latch the data
        self.latch.set_high();
        self.latch.set_low();
    }

    /// Send data to the driver
    // pub fn update(&mut self, bytes: &'static mut B) -> Option<impl Transfer>
    pub fn update(&mut self, bytes: &'static mut [u8]) -> Option<impl Transfer>
    {
        match self.state {
            TransmitState::Idle => None,
            TransmitState::Transmit(_, _) => {
                // {
                //     let slice: &mut [u8] = &mut bytes;
                //     debug_assert!(slice.len() as u16 >= self.count as u16 * CHIP_REGISTER_LENGTH_BYTES);
                // }

                self.state = TransmitState::Transmit(DataTransferType::Grayscale, 0);
                self.handle_data_transfer()
            }
        }
    }

    /// Callback to be registered with Spi Dma block
    /// Manages the data transmission state
    pub fn dma_interrupt_handler(&mut self, spi: SPI) -> Option<impl Transfer> {
        self.spi.put(spi);
        self.handle_data_transfer()
    }

    fn handle_data_transfer(&mut self) -> Option<impl Transfer> {
        match (self.state) {
            // This should never be called in idle state.
            TransmitState::Idle => {
                None
            },
            TransmitState::Transmit(transfer_type, mut current_chip) => {
                if current_chip < self.count {
                    current_chip += 1;

                    self.transmit_msb(transfer_type);
                    if let Some(spi) = self.spi.take() {
                        match self.data.take() {
                            Some(bytes) => {
                                let (a, b) = bytes.split_at_mut(CHIP_REGISTER_LENGTH_BYTES as usize);
                                let array = convert_to_array(a);
                                self.data = Some(b);
                                Some(spi.send_dma(array))
                            }
                            None => None
                        }
                    } else {
                        self.state = TransmitState::Idle;
                        None
                    }
                }
                else {
                    // Latch the data
                    self.latch.set_high();
                    self.latch.set_low();
                    self.state = TransmitState::Idle;
                    self.data = None;
                    None
                }
            }
        }
    }

    /// Send the msb based on the type of data transfer
    fn transmit_msb(&mut self, transfer_type: DataTransferType) {
        match transfer_type {
            DataTransferType::Control => self.bit_writer.write_bit(1),
            DataTransferType::Grayscale => self.bit_writer.write_bit(0),
        }
    }

    /// Calculate bit position
    fn calculate_bit_position(chip: u8, led: u8) -> usize {
        // let t = (chip as u16 * CHIP_REGISTER_LENGTH + (led * 8) as u16) as usize;
        (chip as u16 * CHIP_REGISTER_LENGTH + (led as u16 * 8)) as usize
    }

    /// Setup the driver for the first time
    fn setup(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut buffer: [u8; BUFFER_FOR_4_CHIPS as usize] = [0; BUFFER_FOR_4_CHIPS as usize];
        let mut tempbuffer: [u8; BUFFER_FOR_4_CHIPS as usize] = [0; BUFFER_FOR_4_CHIPS as usize];

        let mut setup_done = false;

        while !setup_done {
            clear_buffer(&mut buffer);
            clear_buffer(&mut tempbuffer);

            for chip_no in 0..self.count {
                self.set_dot_correction_all(&mut buffer, chip_no, DC_DATA, DC_DATA, DC_DATA);
            }

            self.fill_control_data(&mut buffer);
            self.log.log_string("Sending control register data to TLC5955\n");
            self.blocking_write(&mut buffer, DataTransferType::Control);
            clear_buffer(&mut buffer);

            self.log.log_string("Read data after zeros.\n");
            self.blocking_write(&mut buffer, DataTransferType::Control);
            self.log.log_buffer(&buffer);

            // Check the read data matches
            for chip_no in 0..self.count {
                self.set_dot_correction_all(&mut tempbuffer, chip_no, DC_DATA, DC_DATA, DC_DATA);
            }
            self.fill_control_data(&mut tempbuffer);

            if !compare_buffers(&buffer, &tempbuffer) {
                self.log
                    .log_string("Ouch, read control data does not match!\n");
                // return Err(Error::Setup);
            } else {
                setup_done = true;
            }
        }

        self.log.log_string("Read control good.\n");

        // Send the control data the second time.
        self.blocking_write(&mut tempbuffer, DataTransferType::Control);
        Ok(())
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

fn clear_buffer(buffer: &mut [u8]) {
    for index in 0..buffer.len() {
        buffer[index] = 0;
    }
}

fn compare_buffers(buffer1: &[u8], buffer2: &[u8]) -> bool {
    buffer1.iter().eq(buffer2)
}

// Custom try_from, fixed by const generics
fn convert_to_array(slice: &mut [u8]) -> &mut [u8; 96] {
    let ptr = slice.as_mut_ptr() as *mut [u8; 96];
    unsafe {&mut *ptr}
}
