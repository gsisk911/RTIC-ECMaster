//! W5500 SPI network bring-up.

use crate::modbus::modbus_slave::{ModbusSlave, NetworkConfig};
use embedded_hal_02::blocking::spi::Transfer;
use heapless::Vec;

pub const DEFAULT_MAC: [u8; 6] = [0x04, 0xE9, 0xE5, 0x10, 0x00, 0x01];
pub const MODBUS_TCP_PORT: u16 = 502;

const COMMON_BLOCK: u8 = 0x00;
const SOCKET0_BLOCK: u8 = 0x01;
const SOCKET0_TX_BLOCK: u8 = 0x02;
const SOCKET0_RX_BLOCK: u8 = 0x03;
const SOCKET1_BLOCK: u8 = 0x05;
const SOCKET1_TX_BLOCK: u8 = 0x06;
const SOCKET1_RX_BLOCK: u8 = 0x07;
const READ_CONTROL: u8 = 0x00;
const WRITE_CONTROL: u8 = 0x04;

const REG_MODE: u16 = 0x0000;
const REG_GATEWAY: u16 = 0x0001;
const REG_SUBNET: u16 = 0x0005;
const REG_MAC: u16 = 0x0009;
const REG_IP: u16 = 0x000F;
const REG_PHY_CONFIG: u16 = 0x002E;
const REG_VERSION: u16 = 0x0039;
const EXPECTED_VERSION: u8 = 0x04;
const PHY_CONFIG_RESET: u8 = 0x80;
const PHY_CONFIG_SOFTWARE_MODE: u8 = 0x40;
const PHY_CONFIG_100BT_FULL_DUPLEX: u8 = 0x18;
const PHY_CONFIG_FORCED_100_FULL: u8 =
    PHY_CONFIG_RESET | PHY_CONFIG_SOFTWARE_MODE | PHY_CONFIG_100BT_FULL_DUPLEX;

const SOCKET_MODE: u16 = 0x0000;
const SOCKET_COMMAND: u16 = 0x0001;
const SOCKET_STATUS: u16 = 0x0003;
const SOCKET_PORT: u16 = 0x0004;
const SOCKET_DESTINATION_IP: u16 = 0x000C;
const SOCKET_RX_BUFFER_SIZE: u16 = 0x001E;
const SOCKET_TX_BUFFER_SIZE: u16 = 0x001F;
const SOCKET_TX_FREE_SIZE: u16 = 0x0020;
const SOCKET_TX_WRITE_POINTER: u16 = 0x0024;
const SOCKET_RX_RECEIVED_SIZE: u16 = 0x0026;
const SOCKET_RX_READ_POINTER: u16 = 0x0028;

const SOCKET_MODE_TCP: u8 = 0x01;
const SOCKET_MODE_IPRAW: u8 = 0x03;
const SOCKET_COMMAND_OPEN: u8 = 0x01;
const SOCKET_COMMAND_DISCONNECT: u8 = 0x08;
const SOCKET_COMMAND_CLOSE: u8 = 0x10;
const SOCKET_COMMAND_LISTEN: u8 = 0x02;
const SOCKET_COMMAND_SEND: u8 = 0x20;
const SOCKET_COMMAND_RECEIVE: u8 = 0x40;
pub const SOCKET_STATUS_CLOSED: u8 = 0x00;
pub const SOCKET_STATUS_INIT: u8 = 0x13;
pub const SOCKET_STATUS_LISTEN: u8 = 0x14;
pub const SOCKET_STATUS_ESTABLISHED: u8 = 0x17;
pub const SOCKET_STATUS_CLOSE_WAIT: u8 = 0x1C;
pub const SOCKET_STATUS_IPRAW: u8 = 0x32;
const COMMAND_POLL_LIMIT: usize = 256;
const STATUS_POLL_LIMIT: usize = 512;
const SOCKET_BUFFER_MASK: u16 = 0x07FF;
const BUFFER_TRANSFER_CHUNK: usize = 32;
const MAX_ICMP_PACKET: usize = 128;
const MAX_MODBUS_TCP_FRAME: usize = 260;
const ICMP_ECHO_REPLY: u8 = 0;
const ICMP_ECHO_REQUEST: u8 = 8;

#[derive(Clone, Copy)]
pub struct W5500Config {
    pub mac: [u8; 6],
    pub network: NetworkConfig,
    pub modbus_port: u16,
}

impl W5500Config {
    pub fn from_network(network: NetworkConfig) -> Self {
        Self {
            mac: DEFAULT_MAC,
            network,
            modbus_port: MODBUS_TCP_PORT,
        }
    }
}

#[derive(Clone, Copy)]
pub struct W5500Status {
    pub spi_ok: bool,
    pub version: u8,
    pub chip_detected: bool,
    pub phy_config: u8,
    pub mode: u8,
    pub mac: [u8; 6],
    pub network: NetworkConfig,
    pub link_up: bool,
    pub socket0_status: u8,
    pub socket0_rx_size: u16,
    pub socket1_status: u8,
    pub socket1_rx_size: u16,
    pub interrupt_asserted: bool,
}

impl W5500Status {
    pub const fn not_checked() -> Self {
        Self {
            spi_ok: false,
            version: 0,
            chip_detected: false,
            phy_config: 0,
            mode: 0,
            mac: [0; 6],
            network: NetworkConfig {
                ip: [0; 4],
                subnet: [0; 4],
                gateway: [0; 4],
            },
            link_up: false,
            socket0_status: SOCKET_STATUS_CLOSED,
            socket0_rx_size: 0,
            socket1_status: SOCKET_STATUS_CLOSED,
            socket1_rx_size: 0,
            interrupt_asserted: false,
        }
    }

    pub const fn spi_error() -> Self {
        Self::not_checked()
    }
}

pub struct W5500Interface {
    config: W5500Config,
}

impl W5500Interface {
    pub const fn new(config: W5500Config) -> Self {
        Self { config }
    }

    pub fn config(&self) -> W5500Config {
        self.config
    }

    pub fn configure(&mut self, config: W5500Config) {
        self.config = config;
    }

    pub fn bring_up<SPI, E>(&mut self, spi: &mut SPI) -> Result<W5500Status, E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        let version = read_common_u8(spi, REG_VERSION)?;
        if version != EXPECTED_VERSION {
            return Ok(status_from_reads(spi, version, false)?);
        }

        write_common_u8(spi, REG_MODE, 0)?;
        write_common(spi, REG_GATEWAY, &self.config.network.gateway)?;
        write_common(spi, REG_SUBNET, &self.config.network.subnet)?;
        write_common(spi, REG_MAC, &self.config.mac)?;
        write_common(spi, REG_IP, &self.config.network.ip)?;
        configure_phy_100_full_duplex(spi)?;
        configure_socket0_listener(spi, self.config.modbus_port)?;

        status_from_reads(spi, version, true)
    }

    pub fn read_status<SPI, E>(spi: &mut SPI) -> Result<W5500Status, E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        let version = read_common_u8(spi, REG_VERSION)?;
        status_from_reads(spi, version, version == EXPECTED_VERSION)
    }

    pub fn ensure_modbus_listener<SPI, E>(spi: &mut SPI, port: u16) -> Result<(), E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        configure_socket0_listener(spi, port)
    }

    pub fn configure_phy_100_full_duplex<SPI, E>(spi: &mut SPI) -> Result<(), E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        configure_phy_100_full_duplex(spi)
    }

    #[allow(dead_code)]
    pub fn poll_icmp_echo<SPI, E>(spi: &mut SPI) -> Result<bool, E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        poll_icmp_echo(spi)
    }

    pub fn poll_modbus_tcp<SPI, E>(
        &self,
        spi: &mut SPI,
        modbus: &mut ModbusSlave,
    ) -> Result<bool, E>
    where
        SPI: Transfer<u8, Error = E>,
    {
        poll_modbus_tcp(spi, modbus, self.config.modbus_port)
    }
}

fn status_from_reads<SPI, E>(
    spi: &mut SPI,
    version: u8,
    chip_detected: bool,
) -> Result<W5500Status, E>
where
    SPI: Transfer<u8, Error = E>,
{
    let phy_config = read_common_u8(spi, REG_PHY_CONFIG)?;
    let mode = read_common_u8(spi, REG_MODE)?;
    let mut gateway = [0u8; 4];
    let mut subnet = [0u8; 4];
    let mut mac = [0u8; 6];
    let mut ip = [0u8; 4];
    read_into(spi, COMMON_BLOCK, REG_GATEWAY, &mut gateway)?;
    read_into(spi, COMMON_BLOCK, REG_SUBNET, &mut subnet)?;
    read_into(spi, COMMON_BLOCK, REG_MAC, &mut mac)?;
    read_into(spi, COMMON_BLOCK, REG_IP, &mut ip)?;
    let socket0_status = read_socket0_u8(spi, SOCKET_STATUS)?;
    let socket0_rx_size = read_socket0_u16(spi, SOCKET_RX_RECEIVED_SIZE)?;
    let socket1_status = read_socket_u8(spi, SOCKET1_BLOCK, SOCKET_STATUS)?;
    let socket1_rx_size = read_socket_u16(spi, SOCKET1_BLOCK, SOCKET_RX_RECEIVED_SIZE)?;

    Ok(W5500Status {
        spi_ok: true,
        version,
        chip_detected,
        phy_config,
        mode,
        mac,
        network: NetworkConfig {
            ip,
            subnet,
            gateway,
        },
        link_up: phy_config & 0x01 != 0,
        socket0_status,
        socket0_rx_size,
        socket1_status,
        socket1_rx_size,
        interrupt_asserted: false,
    })
}

fn configure_socket0_listener<SPI, E>(spi: &mut SPI, port: u16) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_socket0_u8(spi, SOCKET_RX_BUFFER_SIZE, 2)?;
    write_socket0_u8(spi, SOCKET_TX_BUFFER_SIZE, 2)?;
    write_socket0_u8(spi, SOCKET_MODE, SOCKET_MODE_TCP)?;
    write_socket0(spi, SOCKET_PORT, &port.to_be_bytes())?;
    write_socket0_u8(spi, SOCKET_COMMAND, SOCKET_COMMAND_OPEN)?;
    wait_socket0_command_clear(spi)?;

    let status = wait_socket0_status(spi, SOCKET_STATUS_INIT)?;
    if status == SOCKET_STATUS_INIT {
        write_socket0_u8(spi, SOCKET_COMMAND, SOCKET_COMMAND_LISTEN)?;
        wait_socket0_command_clear(spi)?;
        let _ = wait_socket0_status(spi, SOCKET_STATUS_LISTEN)?;
    }

    Ok(())
}

fn configure_socket1_icmp<SPI, E>(spi: &mut SPI) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_COMMAND, SOCKET_COMMAND_CLOSE)?;
    wait_socket_command_clear(spi, SOCKET1_BLOCK)?;
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_RX_BUFFER_SIZE, 2)?;
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_TX_BUFFER_SIZE, 2)?;
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_MODE, SOCKET_MODE_IPRAW)?;
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_COMMAND, SOCKET_COMMAND_OPEN)?;
    wait_socket_command_clear(spi, SOCKET1_BLOCK)?;
    let _ = wait_socket_status(spi, SOCKET1_BLOCK, SOCKET_STATUS_IPRAW)?;
    Ok(())
}

fn poll_icmp_echo<SPI, E>(spi: &mut SPI) -> Result<bool, E>
where
    SPI: Transfer<u8, Error = E>,
{
    if read_socket_u8(spi, SOCKET1_BLOCK, SOCKET_STATUS)? != SOCKET_STATUS_IPRAW {
        configure_socket1_icmp(spi)?;
        return Ok(false);
    }

    let received = read_socket_u16(spi, SOCKET1_BLOCK, SOCKET_RX_RECEIVED_SIZE)? as usize;
    if received < 6 {
        return Ok(false);
    }

    let rx_read = read_socket_u16(spi, SOCKET1_BLOCK, SOCKET_RX_READ_POINTER)?;
    let mut header = [0u8; 6];
    read_buffer(spi, SOCKET1_RX_BLOCK, rx_read, &mut header)?;

    let source_ip = [header[0], header[1], header[2], header[3]];
    let packet_len = u16::from_be_bytes([header[4], header[5]]) as usize;
    if packet_len > MAX_ICMP_PACKET {
        skip_socket_rx(spi, SOCKET1_BLOCK, rx_read, 6 + packet_len)?;
        return Ok(false);
    }

    let mut packet = [0u8; MAX_ICMP_PACKET];
    read_buffer(
        spi,
        SOCKET1_RX_BLOCK,
        rx_read.wrapping_add(6),
        &mut packet[..packet_len],
    )?;
    skip_socket_rx(spi, SOCKET1_BLOCK, rx_read, 6 + packet_len)?;

    if !make_icmp_echo_reply(&mut packet[..packet_len]) {
        return Ok(false);
    }

    send_ipraw(spi, source_ip, &packet[..packet_len])?;
    Ok(true)
}

fn poll_modbus_tcp<SPI, E>(
    spi: &mut SPI,
    modbus: &mut ModbusSlave,
    port: u16,
) -> Result<bool, E>
where
    SPI: Transfer<u8, Error = E>,
{
    match read_socket0_u8(spi, SOCKET_STATUS)? {
        SOCKET_STATUS_ESTABLISHED => {}
        SOCKET_STATUS_CLOSE_WAIT => {
            write_socket0_u8(spi, SOCKET_COMMAND, SOCKET_COMMAND_DISCONNECT)?;
            wait_socket0_command_clear(spi)?;
            return Ok(false);
        }
        SOCKET_STATUS_CLOSED => {
            configure_socket0_listener(spi, port)?;
            return Ok(false);
        }
        _ => return Ok(false),
    }

    let received = read_socket0_u16(spi, SOCKET_RX_RECEIVED_SIZE)? as usize;
    if received < 8 {
        return Ok(false);
    }

    let frame_len = received.min(MAX_MODBUS_TCP_FRAME);
    let rx_read = read_socket0_u16(spi, SOCKET_RX_READ_POINTER)?;
    let mut request = [0u8; MAX_MODBUS_TCP_FRAME];
    read_buffer(spi, SOCKET0_RX_BLOCK, rx_read, &mut request[..frame_len])?;
    skip_socket_rx(spi, SOCKET0_BLOCK, rx_read, frame_len)?;

    if let Some(response) = modbus.process_frame(&request[..frame_len]) {
        send_tcp(spi, response)?;
        return Ok(true);
    }

    Ok(false)
}

fn configure_phy_100_full_duplex<SPI, E>(spi: &mut SPI) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_common_u8(
        spi,
        REG_PHY_CONFIG,
        PHY_CONFIG_FORCED_100_FULL & !PHY_CONFIG_RESET,
    )?;
    write_common_u8(spi, REG_PHY_CONFIG, PHY_CONFIG_FORCED_100_FULL)
}

fn wait_socket0_command_clear<SPI, E>(spi: &mut SPI) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    wait_socket_command_clear(spi, SOCKET0_BLOCK)
}

fn wait_socket_command_clear<SPI, E>(spi: &mut SPI, block: u8) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    for _ in 0..COMMAND_POLL_LIMIT {
        if read_socket_u8(spi, block, SOCKET_COMMAND)? == 0 {
            break;
        }
    }

    Ok(())
}

fn wait_socket0_status<SPI, E>(spi: &mut SPI, expected: u8) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    wait_socket_status(spi, SOCKET0_BLOCK, expected)
}

fn wait_socket_status<SPI, E>(spi: &mut SPI, block: u8, expected: u8) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut status = SOCKET_STATUS_CLOSED;
    for _ in 0..STATUS_POLL_LIMIT {
        status = read_socket_u8(spi, block, SOCKET_STATUS)?;
        if status == expected || status == SOCKET_STATUS_LISTEN || status == SOCKET_STATUS_ESTABLISHED
        {
            break;
        }
    }

    Ok(status)
}

fn read_common_u8<SPI, E>(spi: &mut SPI, address: u16) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    read_u8(spi, COMMON_BLOCK, address)
}

fn read_socket0_u8<SPI, E>(spi: &mut SPI, address: u16) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    read_socket_u8(spi, SOCKET0_BLOCK, address)
}

fn read_socket0_u16<SPI, E>(spi: &mut SPI, address: u16) -> Result<u16, E>
where
    SPI: Transfer<u8, Error = E>,
{
    read_socket_u16(spi, SOCKET0_BLOCK, address)
}

fn read_socket_u8<SPI, E>(spi: &mut SPI, block: u8, address: u16) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    read_u8(spi, block, address)
}

fn read_socket_u16<SPI, E>(spi: &mut SPI, block: u8, address: u16) -> Result<u16, E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut bytes = [0u8; 2];
    read_into(spi, block, address, &mut bytes)?;
    Ok(u16::from_be_bytes(bytes))
}

fn write_socket0_u8<SPI, E>(spi: &mut SPI, address: u16, value: u8) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_socket_u8(spi, SOCKET0_BLOCK, address, value)
}

fn write_socket_u8<SPI, E>(spi: &mut SPI, block: u8, address: u16, value: u8) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write(spi, block, address, &[value])
}

fn write_socket0_u16<SPI, E>(spi: &mut SPI, address: u16, value: u16) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_socket_u16(spi, SOCKET0_BLOCK, address, value)
}

fn write_socket_u16<SPI, E>(spi: &mut SPI, block: u8, address: u16, value: u16) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write(spi, block, address, &value.to_be_bytes())
}

fn write_common_u8<SPI, E>(spi: &mut SPI, address: u16, value: u8) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_common(spi, address, &[value])
}

fn read_u8<SPI, E>(spi: &mut SPI, block: u8, address: u16) -> Result<u8, E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut frame = [address_hi(address), address_lo(address), control(block, false), 0];
    spi.transfer(&mut frame)?;
    Ok(frame[3])
}

fn write_common<SPI, E>(spi: &mut SPI, address: u16, values: &[u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write(spi, COMMON_BLOCK, address, values)
}

fn write_socket0<SPI, E>(spi: &mut SPI, address: u16, values: &[u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write(spi, SOCKET0_BLOCK, address, values)
}

fn read_into<SPI, E>(spi: &mut SPI, block: u8, address: u16, values: &mut [u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut frame = [0u8; 3 + BUFFER_TRANSFER_CHUNK];
    let len = 3 + values.len();

    frame[0] = address_hi(address);
    frame[1] = address_lo(address);
    frame[2] = control(block, false);
    spi.transfer(&mut frame[..len])?;
    values.copy_from_slice(&frame[3..len]);
    Ok(())
}

fn write<SPI, E>(spi: &mut SPI, block: u8, address: u16, values: &[u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut frame = [0u8; 3 + BUFFER_TRANSFER_CHUNK];
    let len = 3 + values.len();

    frame[0] = address_hi(address);
    frame[1] = address_lo(address);
    frame[2] = control(block, true);
    frame[3..len].copy_from_slice(values);
    spi.transfer(&mut frame[..len])?;
    Ok(())
}

fn read_buffer<SPI, E>(spi: &mut SPI, block: u8, pointer: u16, values: &mut [u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut offset = 0usize;
    while offset < values.len() {
        let end = (offset + BUFFER_TRANSFER_CHUNK).min(values.len());
        let address = pointer.wrapping_add(offset as u16) & SOCKET_BUFFER_MASK;
        read_into(spi, block, address, &mut values[offset..end])?;
        offset = end;
    }
    Ok(())
}

fn write_buffer<SPI, E>(spi: &mut SPI, block: u8, pointer: u16, values: &[u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    let mut offset = 0usize;
    while offset < values.len() {
        let end = (offset + BUFFER_TRANSFER_CHUNK).min(values.len());
        let address = pointer.wrapping_add(offset as u16) & SOCKET_BUFFER_MASK;
        write(spi, block, address, &values[offset..end])?;
        offset = end;
    }
    Ok(())
}

fn skip_socket_rx<SPI, E>(spi: &mut SPI, socket_block: u8, rx_read: u16, len: usize) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    write_socket_u16(
        spi,
        socket_block,
        SOCKET_RX_READ_POINTER,
        rx_read.wrapping_add(len as u16),
    )?;
    write_socket_u8(spi, socket_block, SOCKET_COMMAND, SOCKET_COMMAND_RECEIVE)?;
    wait_socket_command_clear(spi, socket_block)
}

fn send_ipraw<SPI, E>(spi: &mut SPI, destination_ip: [u8; 4], packet: &[u8]) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    wait_tx_capacity(spi, SOCKET1_BLOCK, packet.len())?;
    let tx_write = read_socket_u16(spi, SOCKET1_BLOCK, SOCKET_TX_WRITE_POINTER)?;
    write_buffer(spi, SOCKET1_TX_BLOCK, tx_write, packet)?;
    write_socket_u16(
        spi,
        SOCKET1_BLOCK,
        SOCKET_TX_WRITE_POINTER,
        tx_write.wrapping_add(packet.len() as u16),
    )?;
    write(spi, SOCKET1_BLOCK, SOCKET_DESTINATION_IP, &destination_ip)?;
    write_socket_u8(spi, SOCKET1_BLOCK, SOCKET_COMMAND, SOCKET_COMMAND_SEND)?;
    wait_socket_command_clear(spi, SOCKET1_BLOCK)
}

fn send_tcp<SPI, E>(spi: &mut SPI, packet: Vec<u8, MAX_MODBUS_TCP_FRAME>) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    let packet = packet.as_slice();
    wait_tx_capacity(spi, SOCKET0_BLOCK, packet.len())?;
    let tx_write = read_socket0_u16(spi, SOCKET_TX_WRITE_POINTER)?;
    write_buffer(spi, SOCKET0_TX_BLOCK, tx_write, packet)?;
    write_socket0_u16(
        spi,
        SOCKET_TX_WRITE_POINTER,
        tx_write.wrapping_add(packet.len() as u16),
    )?;
    write_socket0_u8(spi, SOCKET_COMMAND, SOCKET_COMMAND_SEND)?;
    wait_socket0_command_clear(spi)
}

fn wait_tx_capacity<SPI, E>(spi: &mut SPI, socket_block: u8, len: usize) -> Result<(), E>
where
    SPI: Transfer<u8, Error = E>,
{
    for _ in 0..STATUS_POLL_LIMIT {
        if read_socket_u16(spi, socket_block, SOCKET_TX_FREE_SIZE)? as usize >= len {
            break;
        }
    }
    Ok(())
}

fn make_icmp_echo_reply(packet: &mut [u8]) -> bool {
    if packet.len() < 8 || packet[0] != ICMP_ECHO_REQUEST || packet[1] != 0 {
        return false;
    }

    packet[0] = ICMP_ECHO_REPLY;
    packet[2] = 0;
    packet[3] = 0;
    let checksum = internet_checksum(packet);
    packet[2..4].copy_from_slice(&checksum.to_be_bytes());
    true
}

fn internet_checksum(bytes: &[u8]) -> u16 {
    let mut sum = 0u32;
    let mut chunks = bytes.chunks_exact(2);
    for chunk in &mut chunks {
        sum += u16::from_be_bytes([chunk[0], chunk[1]]) as u32;
    }
    if let Some(byte) = chunks.remainder().first() {
        sum += (*byte as u32) << 8;
    }

    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    !(sum as u16)
}

const fn address_hi(address: u16) -> u8 {
    (address >> 8) as u8
}

const fn address_lo(address: u16) -> u8 {
    address as u8
}

const fn control(block: u8, write: bool) -> u8 {
    (block << 3) | if write { WRITE_CONTROL } else { READ_CONTROL }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct FakeSpi {
        transfers: Vec<Vec<u8>>,
    }

    impl FakeSpi {
        fn new() -> Self {
            Self {
                transfers: Vec::new(),
            }
        }
    }

    impl Transfer<u8> for FakeSpi {
        type Error = ();

        fn transfer<'a>(&mut self, frame: &'a mut [u8]) -> Result<&'a [u8], Self::Error> {
            match frame {
                [0x00, 0x39, 0x00, value] => *value = EXPECTED_VERSION,
                [0x00, 0x2E, 0x00, value] => *value = 0x01,
                [0x00, 0x03, 0x08, value] => *value = SOCKET_STATUS_INIT,
                _ => {}
            }
            self.transfers.push(frame.to_vec());
            Ok(frame)
        }
    }

    #[test]
    fn read_status_uses_w5500_read_frame() {
        let mut spi = FakeSpi::new();

        let status = W5500Interface::read_status(&mut spi).unwrap();

        assert!(status.chip_detected);
        assert_eq!(status.version, EXPECTED_VERSION);
        assert_eq!(spi.transfers[0], [0x00, 0x39, 0x00, EXPECTED_VERSION]);
    }

    #[test]
    fn bring_up_writes_network_registers_and_listens_on_modbus_port() {
        let mut spi = FakeSpi::new();
        let network = NetworkConfig {
            ip: [10, 0, 0, 55],
            subnet: [255, 255, 255, 0],
            gateway: [10, 0, 0, 1],
        };
        let mut w5500 = W5500Interface::new(W5500Config::from_network(network));

        let status = w5500.bring_up(&mut spi).unwrap();

        assert!(status.spi_ok);
        assert!(spi
            .transfers
            .contains(&std::vec![0x00, 0x01, 0x04, 10, 0, 0, 1]));
        assert!(spi
            .transfers
            .contains(&std::vec![0x00, 0x0F, 0x04, 10, 0, 0, 55]));
        assert!(spi
            .transfers
            .contains(&std::vec![0x00, 0x04, 0x0C, 0x01, 0xF6]));
        assert!(spi
            .transfers
            .contains(&std::vec![0x00, 0x01, 0x0C, SOCKET_COMMAND_LISTEN]));
    }
}
