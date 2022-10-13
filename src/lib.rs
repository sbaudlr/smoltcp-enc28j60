#![no_std]
#![deny(warnings)]
#![deny(unused)]
#![deny(unsafe_code)]
#![deny(dead_code)]
#![deny(missing_docs)]
#![deny(clippy::panic)]

//! Bindings to ENC28J60 for the smoltcp stack
//!
//! Features:
//!     - `no_std`
//!     - zero (runtime) allocations
//!     - uses [embedded_hal] types for hardware abstraction
//!
//! Limitations:
//!     - RX/TX are limited to a single buffer of size (1518 - 4)
//!     - Only one RX/TX operation at a time, if another operation is attempted while one is in progress then [smoltcp::Error::Illegal] will be returned
//!     - smoltcp will always be requested to perform checksum checking on behalf of the ENC28J60 device

use core::cell::{RefCell, RefMut};

use embedded_hal::blocking;
use embedded_hal::digital::v2::OutputPin;
use enc28j60::{Enc28j60, CRC_SZ, MAX_FRAME_LENGTH};

use smoltcp::phy::{self, Device as SmolDevice, DeviceCapabilities};

/// Maximum message size
const BUFFER_SIZE: usize = (MAX_FRAME_LENGTH - CRC_SZ) as usize;

/// Wrapper for enc28j60 that implements the smoltcp Device trait
pub struct SmolEnc28j60<Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    device: InnerEnc28j60<Spi, Ncs, Int, Reset>,
}

impl<Spi, Ncs, Int, Reset> From<Enc28j60<Spi, Ncs, Int, Reset>>
    for SmolEnc28j60<Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    fn from(enc: Enc28j60<Spi, Ncs, Int, Reset>) -> Self {
        SmolEnc28j60 {
            device: InnerEnc28j60::new(enc),
        }
    }
}

impl<'a, Spi, Ncs, Int, Reset> SmolDevice<'a> for SmolEnc28j60<Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8> + 'a,
    Ncs: OutputPin + 'a,
    Int: enc28j60::IntPin + 'a,
    Reset: enc28j60::ResetPin + 'a,
{
    type RxToken = RxToken<'a, Spi, Ncs, Int, Reset>;

    type TxToken = TxToken<'a, Spi, Ncs, Int, Reset>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        Some((
            RxToken {
                lower: &self.device,
            },
            TxToken {
                lower: &self.device,
            },
        ))
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TxToken {
            lower: &self.device,
        })
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();
        cap.medium = phy::Medium::Ethernet;
        cap.max_transmission_unit = BUFFER_SIZE;
        cap.max_burst_size = Some(1);
        cap
    }
}

struct InnerEnc28j60<Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    device: RefCell<Enc28j60<Spi, Ncs, Int, Reset>>,
    buffer: RefCell<[u8; BUFFER_SIZE]>,
}

impl<Spi, Ncs, Int, Reset> InnerEnc28j60<Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    fn new(device: Enc28j60<Spi, Ncs, Int, Reset>) -> Self {
        InnerEnc28j60 {
            device: RefCell::new(device),
            buffer: RefCell::new([0; BUFFER_SIZE]),
        }
    }

    fn lock(&self) -> Option<SharedBuffer<Spi, Ncs, Int, Reset>> {
        let device = self.device.try_borrow_mut().ok();
        let buffer = self.buffer.try_borrow_mut().ok();

        if let Some(device) = device {
            if let Some(buffer) = buffer {
                return Some(SharedBuffer::new(device, buffer));
            }
        }

        None
    }

    fn send(&self, mut buffer: SharedBuffer<Spi, Ncs, Int, Reset>) -> Result<()> {
        match buffer.device.transmit(buffer.buffer.as_slice()) {
            Ok(_) => Ok(()),
            Err(_) => Err(Error::Illegal),
        }
    }

    fn receive(&self, buffer: &mut SharedBuffer<Spi, Ncs, Int, Reset>) -> Result<()> {
        buffer
            .device
            .receive(buffer.buffer.as_mut_slice())
            .map(|_| ())
            .map_err(|_| Error::Illegal)
    }
}

struct SharedBuffer<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    device: RefMut<'a, Enc28j60<Spi, Ncs, Int, Reset>>,
    buffer: RefMut<'a, [u8; BUFFER_SIZE]>,
}

impl<'a, Spi, Ncs, Int, Reset> SharedBuffer<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    fn new(
        device: RefMut<'a, Enc28j60<Spi, Ncs, Int, Reset>>,
        buffer: RefMut<'a, [u8; BUFFER_SIZE]>,
    ) -> Self {
        SharedBuffer { device, buffer }
    }
}

/// RxToken for enc28j60
pub struct RxToken<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    lower: &'a InnerEnc28j60<Spi, Ncs, Int, Reset>,
}

impl<'a, Spi, Ncs, Int, Reset> phy::RxToken for RxToken<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        let buffer = self.lower.lock();
        match buffer {
            None => Err(smoltcp::Error::Exhausted),
            Some(mut buffer) => {
                self.lower.receive(&mut buffer)?;
                f(buffer.buffer.as_mut_slice())
            }
        }
    }
}

/// TxToken for enc28j60
pub struct TxToken<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    lower: &'a InnerEnc28j60<Spi, Ncs, Int, Reset>,
}

impl<'a, Spi, Ncs, Int, Reset> phy::TxToken for TxToken<'a, Spi, Ncs, Int, Reset>
where
    Spi: blocking::spi::Transfer<u8> + blocking::spi::Write<u8>,
    Ncs: OutputPin,
    Int: enc28j60::IntPin,
    Reset: enc28j60::ResetPin,
{
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        if len > BUFFER_SIZE {
            return Err(smoltcp::Error::Exhausted);
        }

        let buffer = self.lower.lock();
        match buffer {
            None => Err(smoltcp::Error::Exhausted),
            Some(mut buffer) => {
                let result = f(buffer.buffer.as_mut_slice());
                self.lower.send(buffer)?;
                result
            }
        }
    }
}

enum Error {
    /// An operation is not permitted in the current state.
    Illegal,
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

type Result<T> = core::result::Result<T, Error>;

impl From<Error> for smoltcp::Error {
    fn from(_: Error) -> Self {
        smoltcp::Error::Illegal
    }
}
