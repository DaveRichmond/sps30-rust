use core::fmt;
use std::{collections::HashMap, process::exit, thread::sleep, time::Duration};

use clap::{self, Parser};
use hdlc::HDLCError;
use serialport::{self, SerialPort, SerialPortInfo, SerialPortType};

#[derive(clap::Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    port: Option<String>,
    #[arg(long, short, default_value_t = 115200)]
    baud: usize,
}

fn open_port(a: Args) -> Option<Box<dyn SerialPort>> {
    let port: String;
    if let Some(p) = a.port {
        println!("Serialport: {}@{}", p, a.baud);
        port = p.clone();
    } else {
        println!("Serial Ports:");
        for port in serialport::available_ports().expect("No Serial Ports available") {
            let info_s: String;
            match port.port_type {
                SerialPortType::UsbPort(info) => info_s = format!("USB({}:{})", info.vid, info.pid),
                SerialPortType::BluetoothPort => info_s = "Bluetooth".into(),
                SerialPortType::PciPort => info_s = "PCI".into(),
                SerialPortType::Unknown => info_s = "Unkonwn type".into(),
            }
            println!("\t{} ({})", port.port_name, info_s);
        }
        return None;
    }

    let p = serialport::new(port, a.baud.try_into().unwrap())
        .open()
        .expect("Serial port can't be opened");

    Some(p)
}

#[derive(Debug)]
enum Command {
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
                println!("Unknown command for: {}", value);
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
struct FrameError {}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FrameError")
    }
}

#[derive(Debug)]
struct CommandError {}
impl fmt::Display for CommandError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "CommandError")
    }
}

#[derive(Debug)]
struct DeviceError {}
impl fmt::Display for DeviceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DeviceError")
    }
}

fn checksum(buf: Vec<u8>) -> u8 {
    let c = buf.iter().fold(0_u8, |acc, x| acc.wrapping_add(*x));
    !c
}

fn send_frame(mut p: Box<dyn SerialPort>, f: Frame) -> Result<(), FrameError> {
    let HDLC_CONFIG: hdlc::SpecialChars = hdlc::SpecialChars {
        fend: 0x7e,
        fesc: 0x7d,
        translate: HashMap::from([(0x7e, 0x5e), (0x7d, 0x5d), (0x11, 0x31), (0x13, 0x33)]),
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

    p.write_all(&packet).unwrap();

    Ok(())
}

fn receive_frame(mut p: Box<dyn SerialPort>) -> Result<(u8, Frame), FrameError> {
    let HDLC_CONFIG: hdlc::SpecialChars = hdlc::SpecialChars::new_custom(
        0x7e,
        0x7d,
        HashMap::from([(0x7e, 0x5e), (0x7d, 0x5d), (0x11, 0x31), (0x13, 0x33)]),
    );

    let mut reader = hdlc::FrameReader::new(&mut p, HDLC_CONFIG.clone());

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
        println!("Checksum error!");
        return Err(FrameError {});
    }

    let addr = d.remove(0);
    let cmd = d.remove(0).try_into().unwrap();
    let state = d.remove(0);
    let l = d.remove(0);
    if d.len() as u8 != l {
        println!("Packet read: l({}) != d.len({})", l, d.len());
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

fn get_device_info(p: Box<dyn SerialPort>) -> Option<String> {
    println!("Get Device Info command");

    let f = Frame {
        addr: 0x0,
        cmd: Command::DeviceInformation,
        data: vec![0x0],
    };
    send_frame(p.try_clone().unwrap(), f).unwrap();

    let d = receive_frame(p);
    println!("Data recevied: {:#x?}", d);

    let data = d.unwrap().1.data;
    let s = str::from_utf8(&data).unwrap();
    println!("Data content: {:?}", s);

    let s = s.to_string();

    Some(s)
}

fn read_version(p: Box<dyn SerialPort>) -> Result<(), DeviceError> {
    println!("Read version");

    let f = Frame {
        addr: 0,
        cmd: Command::ReadVersion,
        data: Vec::new(),
    };

    send_frame(p.try_clone().unwrap(), f).unwrap();
    let (status, frame) = receive_frame(p).unwrap();
    if frame.data.len() != 7 {
        println!("Wrong received data length: {}", frame.data.len());
        return Err(DeviceError {});
    }

    let firmware_major = frame.data[0];
    let firmware_minor = frame.data[1];
    let hardware_rev = frame.data[3];
    let sdlc_major = frame.data[5];
    let sdlc_minor = frame.data[6];

    println!("Firmware: {}.{}", firmware_major, firmware_minor);
    println!("Hardware: {}", hardware_rev);
    println!("SDLC: {}.{}", sdlc_major, sdlc_minor);

    Ok(())
}

fn start_measurement(mut p: Box<dyn SerialPort>) -> Result<(), DeviceError> {
    println!("Start Device measurement");

    let f = Frame {
        addr: 0x0,
        cmd: Command::StartMeasurement,
        data: vec![0x01u8, 0x03], // ieee floating point
    };
    send_frame(p.try_clone().unwrap(), f).unwrap();
    let (status, frame) = receive_frame(p).unwrap();

    println!("Status: {:x}", status);
    println!("Received frame: {:#x?}", frame);

    if status != 0 {
        eprintln!("Status is not zero!");
        return Err(DeviceError {});
    }

    Ok(())
}

fn device_reset(p: Box<dyn SerialPort>) -> Result<(), DeviceError> {
    println!("Sending Reset");

    let f = Frame {
        addr: 0x0,
        cmd: Command::Reset,
        data: Vec::new(),
    };

    send_frame(p.try_clone().unwrap(), f).unwrap();
    sleep(Duration::from_millis(100)); // we need to wait a bit after a reset

    let (status, frame) = receive_frame(p).unwrap();
    println!("Status: {}", status);
    println!("Frame: {:#x?}", frame);

    Ok(())
}

fn slice_to_f32(a: &[u8]) -> f32 {
    let mut t = Vec::new();
    a.clone_into(&mut t);
    let v: [u8; 4] = t.try_into().unwrap();
    f32::from_be_bytes(v)
}

fn read_measurement(p: Box<dyn SerialPort>) -> Result<(), DeviceError> {
    println!("Read Measurement");

    let f = Frame {
        addr: 0x0,
        cmd: Command::ReadMeasuredValue,
        data: Vec::new(),
    };
    send_frame(p.try_clone().unwrap(), f).unwrap();
    let (status, frame) = receive_frame(p).unwrap();
    println!("Status: {}", status);
    //println!("Frame: {:#x?}", frame);

    if frame.data.len() > 0 {
        let mass_1_0 = slice_to_f32(&frame.data[0..4]);
        let mass_2_5 = slice_to_f32(&frame.data[4..8]);
        let mass_4_0 = slice_to_f32(&frame.data[8..12]);
        let mass_10 = slice_to_f32(&frame.data[12..16]);
        let concentration_pm005 = slice_to_f32(&frame.data[16..20]);
        let concentration_pm010 = slice_to_f32(&frame.data[20..24]);
        let concentration_pm025 = slice_to_f32(&frame.data[24..28]);
        let concentration_pm040 = slice_to_f32(&frame.data[28..32]);
        let concentration_pm100 = slice_to_f32(&frame.data[32..36]);
        let particle = slice_to_f32(&frame.data[36..40]);

        println!("mass 1.0: {} µg/m³", mass_1_0);
        println!("mass 2.5: {} µg/m³", mass_2_5);
        println!("mass 4.0: {} µg/m³", mass_4_0);
        println!("mass 10: {} µg/m³", mass_10);
        println!("concentration pm0.5: {} #/cm³", concentration_pm005);
        println!("concentration pm1.0: {} #/cm³", concentration_pm010);
        println!("concentration pm2.5: {} #/cm³", concentration_pm025);
        println!("concentration pm4.0: {} #/cm³", concentration_pm040);
        println!("concentration pm10.0: {} #/cm³", concentration_pm100);
        println!("Typical particle size: {} nm", particle);
    }

    Ok(())
}

fn main() {
    let args = Args::parse();
    let mut p = open_port(args).unwrap();

    println!("Port: {}", p.name().unwrap());
    p.set_timeout(Duration::from_millis(20)).unwrap();

    println!("Clear existing input");
    let mut buf = Vec::new();
    match p.try_clone().unwrap().read_to_end(&mut buf) {
        Ok(d) => println!("Read {} bytes", d),
        Err(e) => eprintln!("Error: {}", e),
    }

    device_reset(p.try_clone().unwrap()).unwrap();

    println!(
        "Device info: {:#?}",
        get_device_info(p.try_clone().unwrap())
    );
    read_version(p.try_clone().unwrap()).unwrap();
    start_measurement(p.try_clone().unwrap()).unwrap();

    loop {
        sleep(Duration::from_millis(500));

        println!(
            "Measurement: {:#?}",
            read_measurement(p.try_clone().unwrap()),
        );
    }
}
