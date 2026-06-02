//! Small Modbus TCP holding-register helper.
//!
//! Networking is intentionally left to the W5500 socket layer. This module
//! only stores registers and turns complete Modbus TCP frames into responses.

use crate::modbus::register_map::{self, RegisterDef};
use heapless::Vec;

const MAX_REGS: usize = 32;
const MAX_FRAME: usize = 260;
const MBAP_HEADER_LEN: usize = 7;

#[derive(Clone, Copy)]
struct Register {
    address: u16,
    value: u16,
}

pub struct ModbusSlave {
    registers: [Register; MAX_REGS],
    register_count: usize,
}

impl ModbusSlave {
    pub const fn new() -> Self {
        Self {
            registers: [Register {
                address: 0xFFFF,
                value: 0,
            }; MAX_REGS],
            register_count: 0,
        }
    }

    pub fn load_register_map(&mut self, registers: &[RegisterDef]) {
        for register in registers {
            self.add_hreg(register.address, register.default_value);
        }
    }

    pub fn add_hreg(&mut self, address: u16, value: u16) {
        if let Some(register) = self.find_mut(address) {
            register.value = value;
            return;
        }

        if self.register_count < MAX_REGS {
            self.registers[self.register_count] = Register { address, value };
            self.register_count += 1;
        }
    }

    pub fn get_hreg(&self, address: u16) -> u16 {
        self.find(address).map(|register| register.value).unwrap_or(0)
    }

    pub fn set_hreg(&mut self, address: u16, value: u16) -> bool {
        if let Some(register) = self.find_mut(address) {
            register.value = value;
            return true;
        }
        false
    }

    pub fn network_config(&self) -> NetworkConfig {
        NetworkConfig {
            ip: self.ip_from_registers(
                register_map::REG_IP_OCTET_1,
                register_map::REG_IP_OCTET_2,
                register_map::REG_IP_OCTET_3,
                register_map::REG_IP_OCTET_4,
            ),
            subnet: self.ip_from_registers(
                register_map::REG_SUBNET_OCTET_1,
                register_map::REG_SUBNET_OCTET_2,
                register_map::REG_SUBNET_OCTET_3,
                register_map::REG_SUBNET_OCTET_4,
            ),
            gateway: self.ip_from_registers(
                register_map::REG_GW_OCTET_1,
                register_map::REG_GW_OCTET_2,
                register_map::REG_GW_OCTET_3,
                register_map::REG_GW_OCTET_4,
            ),
        }
    }

    pub fn take_network_update(&mut self) -> Option<NetworkConfig> {
        if self.get_hreg(register_map::REG_APPLY_NETWORK) == 0 {
            return None;
        }

        self.set_hreg(register_map::REG_APPLY_NETWORK, 0);
        Some(self.network_config())
    }

    pub fn process_frame(&mut self, frame: &[u8]) -> Option<Vec<u8, MAX_FRAME>> {
        if frame.len() < MBAP_HEADER_LEN + 1 {
            return None;
        }

        let transaction_id = read_u16(frame, 0);
        let unit_id = frame[6];
        let function_code = frame[7];

        Some(match function_code {
            0x03 => self.read_holding(frame, transaction_id, unit_id),
            0x06 => self.write_single(frame, transaction_id, unit_id),
            0x10 => self.write_multiple(frame, transaction_id, unit_id),
            _ => exception(transaction_id, unit_id, function_code, 0x01),
        })
    }

    fn read_holding(&self, frame: &[u8], transaction_id: u16, unit_id: u8) -> Vec<u8, MAX_FRAME> {
        if frame.len() < 12 {
            return exception(transaction_id, unit_id, 0x03, 0x03);
        }

        let start = read_u16(frame, 8);
        let quantity = read_u16(frame, 10);
        let data_bytes = quantity as usize * 2;
        if quantity == 0 || data_bytes + 9 > MAX_FRAME {
            return exception(transaction_id, unit_id, 0x03, 0x03);
        }

        let mut response = header(transaction_id, unit_id, (3 + data_bytes) as u16);
        let _ = response.push(0x03);
        let _ = response.push(data_bytes as u8);
        for offset in 0..quantity {
            push_u16(&mut response, self.get_hreg(start + offset));
        }
        response
    }

    fn write_single(
        &mut self,
        frame: &[u8],
        transaction_id: u16,
        unit_id: u8,
    ) -> Vec<u8, MAX_FRAME> {
        if frame.len() < 12 {
            return exception(transaction_id, unit_id, 0x06, 0x03);
        }

        let address = read_u16(frame, 8);
        let value = read_u16(frame, 10);
        self.set_hreg(address, value);

        let mut response = header(transaction_id, unit_id, 6);
        let _ = response.push(0x06);
        push_u16(&mut response, address);
        push_u16(&mut response, value);
        response
    }

    fn write_multiple(
        &mut self,
        frame: &[u8],
        transaction_id: u16,
        unit_id: u8,
    ) -> Vec<u8, MAX_FRAME> {
        if frame.len() < 13 {
            return exception(transaction_id, unit_id, 0x10, 0x03);
        }

        let start = read_u16(frame, 8);
        let quantity = read_u16(frame, 10);
        let byte_count = frame[12] as usize;
        if quantity == 0 || byte_count != quantity as usize * 2 || frame.len() < 13 + byte_count {
            return exception(transaction_id, unit_id, 0x10, 0x03);
        }

        for index in 0..quantity {
            let offset = 13 + index as usize * 2;
            self.set_hreg(start + index, read_u16(frame, offset));
        }

        let mut response = header(transaction_id, unit_id, 6);
        let _ = response.push(0x10);
        push_u16(&mut response, start);
        push_u16(&mut response, quantity);
        response
    }

    fn find(&self, address: u16) -> Option<&Register> {
        self.registers[..self.register_count]
            .iter()
            .find(|register| register.address == address)
    }

    fn find_mut(&mut self, address: u16) -> Option<&mut Register> {
        self.registers[..self.register_count]
            .iter_mut()
            .find(|register| register.address == address)
    }

    fn ip_from_registers(&self, a: u16, b: u16, c: u16, d: u16) -> [u8; 4] {
        [
            self.get_hreg(a) as u8,
            self.get_hreg(b) as u8,
            self.get_hreg(c) as u8,
            self.get_hreg(d) as u8,
        ]
    }
}

#[derive(Clone, Copy)]
pub struct NetworkConfig {
    pub ip: [u8; 4],
    pub subnet: [u8; 4],
    pub gateway: [u8; 4],
}

fn header(transaction_id: u16, unit_id: u8, length: u16) -> Vec<u8, MAX_FRAME> {
    let mut response = Vec::new();
    push_u16(&mut response, transaction_id);
    push_u16(&mut response, 0);
    push_u16(&mut response, length);
    let _ = response.push(unit_id);
    response
}

fn exception(transaction_id: u16, unit_id: u8, function_code: u8, code: u8) -> Vec<u8, MAX_FRAME> {
    let mut response = header(transaction_id, unit_id, 3);
    let _ = response.push(function_code | 0x80);
    let _ = response.push(code);
    response
}

fn read_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_be_bytes([bytes[offset], bytes[offset + 1]])
}

fn push_u16(vec: &mut Vec<u8, MAX_FRAME>, value: u16) {
    let _ = vec.extend_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::ModbusSlave;
    use crate::modbus::register_map::{REG_IP_OCTET_4, USER_REGISTERS};

    #[test]
    fn reads_default_ip_register() {
        let mut slave = ModbusSlave::new();
        slave.load_register_map(USER_REGISTERS);

        let request = [0, 1, 0, 0, 0, 6, 1, 0x03, 0, 6, 0, 1];
        let response = slave.process_frame(&request).unwrap();

        assert_eq!(response.as_slice(), &[0, 1, 0, 0, 0, 5, 1, 0x03, 2, 0, 55]);
    }

    #[test]
    fn applies_network_update_once() {
        let mut slave = ModbusSlave::new();
        slave.load_register_map(USER_REGISTERS);
        slave.set_hreg(REG_IP_OCTET_4, 42);
        slave.set_hreg(crate::modbus::register_map::REG_APPLY_NETWORK, 1);

        let network = slave.take_network_update().unwrap();

        assert_eq!(network.ip, [10, 0, 0, 42]);
        assert!(slave.take_network_update().is_none());
    }
}
