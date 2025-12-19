#![no_std]
#[cfg(all(feature = "std", feature = "no_std"))]
compile_error!("Only one of std or no-std features can be selected");

#[cfg(not(any(feature = "std", feature = "no_std")))]
compile_error!("Only one of std or no-std may be selected at a time");

extern crate alloc;

use cfg_block::cfg_block;
cfg_block! {
    #[cfg(feature = "std")]{
        extern crate std;
        use std::collections::HashMap;
        use log::info;
        use std::io::{Read, Write};
        use std::time::Duration;
        use std::thread::sleep;
    }
    #[cfg(feature = "no_std")]{
        use hashbrown::HashMap;
        use defmt::info;
        use embedded_io::{Read, Write};
    }
}

use alloc::borrow::ToOwned;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

#[derive(Debug)]
pub enum Command {
    StartMeasurement,
    StopMeasurement,
    ReadMeasuredValue,
    Sleep,
    WakeUp,
    StartFanCleaning,
    RWAutoCleaningInterval,
    DeviceInformation,
    ReadVersion,
    ReadDeviceStatusRegister,
    Reset,
}
impl From<Command> for u8 {
    fn from(value: Command) -> Self {
        match value {
            Command::StartMeasurement => 0x00,
            Command::StopMeasurement => 0x01,
            Command::ReadMeasuredValue => 0x03,
            Command::Sleep => 0x10,
            Command::WakeUp => 0x11,
            Command::StartFanCleaning => 0x56,
            Command::RWAutoCleaningInterval => 0x80,
            Command::DeviceInformation => 0xD0,
            Command::ReadVersion => 0xD1,
            Command::ReadDeviceStatusRegister => 0xD2,
            Command::Reset => 0xD3,
        }
    }
}

impl TryFrom<u8> for Command {
    type Error = CommandError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Command::StartMeasurement),
            0x01 => Ok(Command::StopMeasurement),
            0x03 => Ok(Command::ReadMeasuredValue),
            0x10 => Ok(Command::Sleep),
            0x11 => Ok(Command::WakeUp),
            0x56 => Ok(Command::StartFanCleaning),
            0x80 => Ok(Command::RWAutoCleaningInterval),
            0xD0 => Ok(Command::DeviceInformation),
            0xD1 => Ok(Command::ReadVersion),
            0xD2 => Ok(Command::ReadDeviceStatusRegister),
            0xD3 => Ok(Command::Reset),
            _ => {
                info!("Unknown command for: {}", value);
                Err(CommandError {})
            }
        }
    }
}

#[derive(Debug)]
struct Frame {
    addr: u8,
    cmd: Command,
    data: Vec<u8>,
}

#[derive(Debug)]
pub struct FrameError {}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "FrameError")
    }
}

#[derive(Debug)]
pub struct CommandError {}
impl core::fmt::Display for CommandError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CommandError")
    }
}

#[derive(Debug)]
pub struct DeviceError {}
impl core::fmt::Display for DeviceError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "DeviceError")
    }
}

fn checksum(buf: Vec<u8>) -> u8 {
    let c = buf.iter().fold(0_u8, |acc, x| acc.wrapping_add(*x));
    !c
}

fn slice_to_f32(a: &[u8]) -> f32 {
    let mut t = Vec::new();
    a.clone_into(&mut t);
    let v: [u8; 4] = t.try_into().unwrap();
    f32::from_be_bytes(v)
}

fn to_bool(i: u8) -> bool {
    match i {
        0 => false,
        _ => true,
    }
}

pub struct Sps30<P> {
    port: P,
    running: bool,
}

impl<P: Write + Read> Sps30<P> {
    pub fn new(port: P) -> Self {
        Self {
            port,
            running: false,
        }
    }
    fn send_frame(&mut self, f: Frame) -> Result<(), FrameError> {
        let HDLC_CONFIG: hdlc::SpecialChars = hdlc::SpecialChars {
            fend: 0x7e,
            fesc: 0x7d,
            translate: HashMap::from([
                (0x7e_u8, 0x5e_u8),
                (0x7d, 0x5d),
                (0x11, 0x31),
                (0x13, 0x33),
            ]),
        };
        // println!("Sending frame: {:#x?}", f);
        if f.addr != 0 {
            return Err(FrameError {});
        }
        let mut buffer = Vec::new();
        buffer.push(f.addr);
        buffer.push(f.cmd.into());
        buffer.push(f.data.len() as u8);
        buffer.append(&mut f.data.clone());
        let c = checksum(buffer.clone());
        buffer.push(c);

        let packet = hdlc::encode(&buffer, HDLC_CONFIG).unwrap();
        // println!("Send Packet: {:#x?}", packet);

        self.port.write_all(&packet).unwrap();

        Ok(())
    }
    fn receive_frame(&mut self) -> Result<(u8, Frame), FrameError> {
        let HDLC_CONFIG: hdlc::SpecialChars = hdlc::SpecialChars::new_custom(
            0x7e,
            0x7d,
            HashMap::from([(0x7e, 0x5e), (0x7d, 0x5d), (0x11, 0x31), (0x13, 0x33)]),
        );

        let mut reader = hdlc::FrameReader::new(&mut self.port, HDLC_CONFIG.clone());

        let frame: Vec<u8>;
        loop {
            let f = reader.read_frame();
            if f != None {
                frame = f.unwrap();
                break;
            }
        }

        let mut d = hdlc::decode(&frame, HDLC_CONFIG.clone()).unwrap();
        // println!("Packet read: {:#x?}", d);

        let c = d.pop().unwrap();
        if c != checksum(d.clone()) {
            info!("Checksum error!");
            return Err(FrameError {});
        }

        let addr = d.remove(0);
        let cmd = d.remove(0).try_into().unwrap();
        let state = d.remove(0);
        let l = d.remove(0);
        if d.len() as u8 != l {
            info!("Packet read: l({}) != d.len({})", l, d.len());
            return Err(FrameError {});
        }

        Ok((
            state,
            Frame {
                addr,
                cmd,
                data: d.clone(),
            },
        ))
    }

    pub fn get_device_info(&mut self) -> Option<String> {
        info!("Get Device Info command");

        let f = Frame {
            addr: 0x0,
            cmd: Command::DeviceInformation,
            data: vec![0x0],
        };
        self.send_frame(f).unwrap();

        let d = self.receive_frame();
        info!("Data recevied: {:#x?}", d);

        let data = d.unwrap().1.data;
        let s = str::from_utf8(&data).unwrap();
        info!("Data content: {:?}", s);

        let s = s.to_string();

        Some(s)
    }

    pub fn read_version(&mut self) -> Result<Sps30Version, DeviceError> {
        info!("Read version");

        let f = Frame {
            addr: 0,
            cmd: Command::ReadVersion,
            data: Vec::new(),
        };

        self.send_frame(f).unwrap();
        let (status, frame) = self.receive_frame().unwrap();
        if frame.data.len() != 7 {
            info!("Wrong received data length: {}", frame.data.len());
            return Err(DeviceError {});
        }

        let firmware_major = frame.data[0];
        let firmware_minor = frame.data[1];
        let hardware_rev = frame.data[3];
        let sdlc_major = frame.data[5];
        let sdlc_minor = frame.data[6];

        let mut buf = [0u8; 32];
        let firmware = String::from(
            format_no_std::show(
                &mut buf,
                format_args!("{}.{}", firmware_major, firmware_minor),
            )
            .unwrap(),
        );
        let hardware =
            String::from(format_no_std::show(&mut buf, format_args!("{}", hardware_rev)).unwrap());
        let shdlc = String::from(
            format_no_std::show(&mut buf, format_args!("{}.{}", sdlc_major, sdlc_minor)).unwrap(),
        );

        info!("Firmware: {}.{}", firmware_major, firmware_minor);
        info!("Hardware: {}", hardware_rev);
        info!("SDLC: {}.{}", sdlc_major, sdlc_minor);

        Ok(Sps30Version {
            firmware,
            hardware,
            shdlc,
        })
    }
    pub fn start_measurement(&mut self) -> Result<(), DeviceError> {
        info!("Start Device measurement");
        if self.running {
            info!("Trying to start device when already running");
            return Err(DeviceError {});
        }

        let f = Frame {
            addr: 0x0,
            cmd: Command::StartMeasurement,
            data: vec![0x01u8, 0x03], // ieee floating point
        };
        self.send_frame(f).unwrap();
        let (status, frame) = self.receive_frame().unwrap();

        info!("Status: {:x}", status);
        info!("Received frame: {:#x?}", frame);

        if status != 0 {
            info!("Status is not zero!");
            return Err(DeviceError {});
        }

        Ok(())
    }
    pub fn device_reset(&mut self) -> Result<(), DeviceError> {
        info!("Sending Reset");

        let f = Frame {
            addr: 0x0,
            cmd: Command::Reset,
            data: Vec::new(),
        };

        self.send_frame(f).unwrap();
        sleep(Duration::from_millis(100)); // we need to wait a bit after a reset. FIXME on no-std

        let (status, frame) = self.receive_frame().unwrap();
        info!("Status: {}", status);
        info!("Frame: {:#x?}", frame);

        self.running = false;

        Ok(())
    }

    pub fn read_measurement(&mut self) -> Result<Option<Sps30Measurement>, DeviceError> {
        info!("Read Measurement");

        let f = Frame {
            addr: 0x0,
            cmd: Command::ReadMeasuredValue,
            data: Vec::new(),
        };
        self.send_frame(f).unwrap();
        let (status, frame) = self.receive_frame().unwrap();
        info!("Status: {}", status);
        //println!("Frame: {:#x?}", frame);

        let mut mass_1_0 = 0_f32;
        let mut mass_2_5 = 0_f32;
        let mut mass_4_0 = 0_f32;
        let mut mass_10 = 0_f32;
        let mut concentration_pm005 = 0_f32;
        let mut concentration_pm010 = 0_f32;
        let mut concentration_pm025 = 0_f32;
        let mut concentration_pm040 = 0_f32;
        let mut concentration_pm100 = 0_f32;
        let mut particle = 0_f32;
        if frame.data.len() > 0 {
            mass_1_0 = slice_to_f32(&frame.data[0..4]);
            mass_2_5 = slice_to_f32(&frame.data[4..8]);
            mass_4_0 = slice_to_f32(&frame.data[8..12]);
            mass_10 = slice_to_f32(&frame.data[12..16]);
            concentration_pm005 = slice_to_f32(&frame.data[16..20]);
            concentration_pm010 = slice_to_f32(&frame.data[20..24]);
            concentration_pm025 = slice_to_f32(&frame.data[24..28]);
            concentration_pm040 = slice_to_f32(&frame.data[28..32]);
            concentration_pm100 = slice_to_f32(&frame.data[32..36]);
            particle = slice_to_f32(&frame.data[36..40]);

            info!("mass pm1.0: {} µg/m³", mass_1_0);
            info!("mass pm2.5: {} µg/m³", mass_2_5);
            info!("mass pm4.0: {} µg/m³", mass_4_0);
            info!("mass pm10: {} µg/m³", mass_10);
            info!("concentration pm0.5: {} #/cm³", concentration_pm005);
            info!("concentration pm1.0: {} #/cm³", concentration_pm010);
            info!("concentration pm2.5: {} #/cm³", concentration_pm025);
            info!("concentration pm4.0: {} #/cm³", concentration_pm040);
            info!("concentration pm10.0: {} #/cm³", concentration_pm100);
            info!("Typical particle size: {} nm", particle);
        } else {
            info!("No data changed");
            return Ok(None);
        }

        Ok(Some(Sps30Measurement {
            mass_1_0,
            mass_2_5,
            mass_4_0,
            mass_10,
            concentration_pm005,
            concentration_pm010,
            concentration_pm025,
            concentration_pm040,
            concentration_pm100,
            particle,
        }))
    }

    pub fn read_device_status(&mut self) -> Result<Option<Vec<Sps30Fault>>, DeviceError> {
        info!("Reading device status");

        let f = Frame {
            addr: 0x0,
            cmd: Command::ReadDeviceStatusRegister,
            data: vec![0x01], // clear register after reading
        };

        self.send_frame(f).unwrap();
        let (status, frame) = self.receive_frame().unwrap();

        if frame.data.len() != 5 {
            info!("wrong frame size read: {}", frame.data.len());
            return Err(DeviceError {});
        }
        let fan_err = to_bool(frame.data[3] & (1 << 4));
        let laser_err = to_bool(frame.data[3] & (1 << 5));
        let speed_err = to_bool(frame.data[1] & (1 << 5));

        let mut faults = Vec::new();
        if fan_err {
            faults.push(Sps30Fault::Fan);
        }
        if laser_err {
            faults.push(Sps30Fault::Laser);
        }
        if speed_err {
            faults.push(Sps30Fault::FanSpeed);
        }
        if faults.len() > 0 {
            Ok(Some(faults))
        } else {
            Ok(None)
        }
    }
}

#[derive(Debug)]
pub enum Sps30Fault {
    Fan,
    Laser,
    FanSpeed,
}

#[derive(Debug)]
pub struct Sps30Measurement {
    mass_1_0: f32,
    mass_2_5: f32,
    mass_4_0: f32,
    mass_10: f32,
    concentration_pm005: f32,
    concentration_pm010: f32,
    concentration_pm025: f32,
    concentration_pm040: f32,
    concentration_pm100: f32,
    particle: f32,
}

#[derive(Debug)]
pub struct Sps30Version {
    firmware: String,
    hardware: String,
    shdlc: String,
}
