//! Minimal Modbus holding-register map for the generic device base.

#[derive(Clone, Copy)]
pub struct RegisterDef {
    pub address: u16,
    pub default_value: u16,
    pub name: &'static str,
    pub saveable: bool,
}

pub const REG_IP_OCTET_1: u16 = 0x0000;
pub const REG_IP_OCTET_2: u16 = 0x0002;
pub const REG_IP_OCTET_3: u16 = 0x0004;
pub const REG_IP_OCTET_4: u16 = 0x0006;

pub const REG_SUBNET_OCTET_1: u16 = 0x0008;
pub const REG_SUBNET_OCTET_2: u16 = 0x000A;
pub const REG_SUBNET_OCTET_3: u16 = 0x000C;
pub const REG_SUBNET_OCTET_4: u16 = 0x000E;

pub const REG_GW_OCTET_1: u16 = 0x0010;
pub const REG_GW_OCTET_2: u16 = 0x0012;
pub const REG_GW_OCTET_3: u16 = 0x0014;
pub const REG_GW_OCTET_4: u16 = 0x0016;

pub const REG_APPLY_NETWORK: u16 = 0x0018;
pub const REG_MODBUS_UNIT_ID: u16 = 0x0020;
pub const REG_STATUS: u16 = 0x0022;

pub const USER_REGISTERS: &[RegisterDef] = &[
    RegisterDef { address: REG_IP_OCTET_1, default_value: 10, name: "ip_octet_1", saveable: true },
    RegisterDef { address: REG_IP_OCTET_2, default_value: 0, name: "ip_octet_2", saveable: true },
    RegisterDef { address: REG_IP_OCTET_3, default_value: 0, name: "ip_octet_3", saveable: true },
    RegisterDef { address: REG_IP_OCTET_4, default_value: 55, name: "ip_octet_4", saveable: true },
    RegisterDef { address: REG_SUBNET_OCTET_1, default_value: 255, name: "subnet_octet_1", saveable: true },
    RegisterDef { address: REG_SUBNET_OCTET_2, default_value: 255, name: "subnet_octet_2", saveable: true },
    RegisterDef { address: REG_SUBNET_OCTET_3, default_value: 255, name: "subnet_octet_3", saveable: true },
    RegisterDef { address: REG_SUBNET_OCTET_4, default_value: 0, name: "subnet_octet_4", saveable: true },
    RegisterDef { address: REG_GW_OCTET_1, default_value: 10, name: "gateway_octet_1", saveable: true },
    RegisterDef { address: REG_GW_OCTET_2, default_value: 0, name: "gateway_octet_2", saveable: true },
    RegisterDef { address: REG_GW_OCTET_3, default_value: 0, name: "gateway_octet_3", saveable: true },
    RegisterDef { address: REG_GW_OCTET_4, default_value: 1, name: "gateway_octet_4", saveable: true },
    RegisterDef { address: REG_APPLY_NETWORK, default_value: 0, name: "apply_network", saveable: false },
    RegisterDef { address: REG_MODBUS_UNIT_ID, default_value: 1, name: "modbus_unit_id", saveable: true },
    RegisterDef { address: REG_STATUS, default_value: 0, name: "status", saveable: false },
];

