use core::fmt;
use std::{collections::HashMap, fmt::Debug, process::exit, thread::sleep, time::Duration};

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

    let mut sensor = sps30_hdlc::Sps30::new(p);
    sensor.device_reset().unwrap();

    println!("Device info: {:#?}", sensor.get_device_info());
    println!("Device versions: {:#?}", sensor.read_version().unwrap());
    sensor.start_measurement().unwrap();

    loop {
        sleep(Duration::from_millis(500));

        colour::blue_ln!("Time: {}", chrono::Local::now());
        let status = sensor.read_device_status().unwrap();
        match status {
            None => colour::green_ln!("Sensor OK"),
            Some(e) => colour::red_ln!("Sensor Status: {:#?}", e),
        }

        let measurement = sensor.read_measurement().unwrap();
        match measurement {
            None => colour::yellow_ln!("No new data"),
            Some(m) => println!("{:#?}", m),
        }
    }
}
