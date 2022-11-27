#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

#[derive(Debug, Default, Copy, Clone, PartialEq)]
struct FlagsRegister {
    zero: bool,
    subtract: bool,
    half_carry: bool,
    carry: bool,
}

const ZERO_FLAG_BYTE_POSITION: u8 = 7;
const SUBTRACT_FLAG_BYTE_POSITION: u8 = 6;
const HALF_CARRY_FLAG_BYTE_POSITION: u8 = 5;
const CARRY_FLAG_BYTE_POSITION: u8 = 4;

impl std::convert::From<FlagsRegister> for u8 {
    fn from(flag: FlagsRegister) -> u8 {
        (u8::from(flag.zero)) << ZERO_FLAG_BYTE_POSITION
            | u8::from(flag.subtract) << SUBTRACT_FLAG_BYTE_POSITION
            | u8::from(flag.half_carry) << HALF_CARRY_FLAG_BYTE_POSITION
            | u8::from(flag.carry) << CARRY_FLAG_BYTE_POSITION
    }
}

impl std::convert::From<FlagsRegister> for u16 {
    fn from(flag: FlagsRegister) -> u16 {
        u8::from(flag) as u16
    }
}

impl std::convert::From<u8> for FlagsRegister {
    fn from(byte: u8) -> Self {
        let zero = ((byte >> ZERO_FLAG_BYTE_POSITION) & 0b1) != 0;
        let subtract = ((byte >> SUBTRACT_FLAG_BYTE_POSITION) & 0b1) != 0;
        let half_carry = ((byte >> HALF_CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;
        let carry = ((byte >> CARRY_FLAG_BYTE_POSITION) & 0b1) != 0;

        FlagsRegister {
            zero,
            subtract,
            half_carry,
            carry,
        }
    }
}

impl std::convert::From<u16> for FlagsRegister {
    fn from(byte: u16) -> Self {
        FlagsRegister::from(byte as u8)
    }
}

#[derive(Debug, Default, Copy, Clone, PartialEq)]
struct Registers {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    f: FlagsRegister,
    h: u8,
    l: u8,
    sp: u16,
}

impl Registers {
    fn get_af(&self) -> u16 {
        (self.a as u16) << 8 | (u16::from(self.f))
    }

    fn set_af(&mut self, val: u16) {
        self.a = (val >> 8) as u8;
        // Will only take the lower byte
        self.f = FlagsRegister::from(val);
    }

    fn get_bc(&self) -> u16 {
        (self.b as u16) << 8 | (self.c as u16)
    }

    fn set_bc(&mut self, val: u16) {
        self.b = (val >> 8) as u8;
        self.c = val as u8;
    }

    fn get_de(&self) -> u16 {
        (self.d as u16) << 8 | (self.e as u16)
    }

    fn set_de(&mut self, val: u16) {
        self.d = (val >> 8) as u8;
        self.e = val as u8;
    }

    fn get_hl(&self) -> u16 {
        (self.h as u16) << 8 | (self.l as u16)
    }

    fn set_hl(&mut self, val: u16) {
        self.h = (val >> 8) as u8;
        self.l = val as u8;
    }

    fn get_sp(&self) -> u16 {
        self.sp
    }

    fn set_sp(&mut self, val: u16) {
        self.sp = val;
    }
}

enum Instruction {
    ADD(ShortArithmeticTarget),
    ADC(ShortArithmeticTarget),
    SUB(ShortArithmeticTarget),
    SBC(ShortArithmeticTarget),
    AND(ShortArithmeticTarget),
    OR(ShortArithmeticTarget),
    XOR(ShortArithmeticTarget),
    CP(ShortArithmeticTarget),
    INC(ArithmeticTarget),
    DEC(ArithmeticTarget),
    ADDHL(LongArithmeticTarget),
    SWAP(ShortArithmeticTarget),
    CCF,
    SCF,
    NOP,
    CPL,
    RRA,
    RRCA,
    RLA,
    RLCA,
    RR(ShortArithmeticTarget),
    RRC(ShortArithmeticTarget),
    RL(ShortArithmeticTarget),
    RLC(ShortArithmeticTarget),
    SLA(ShortArithmeticTarget),
    SRA(ShortArithmeticTarget),
    SRL(ShortArithmeticTarget),
    BIT(u8, ShortArithmeticTarget),
    SET(u8, ShortArithmeticTarget),
    RESET(u8, ShortArithmeticTarget),
    LD(MemoryTarget, MemoryTarget),
    PUSH(LongMemoryTarget),
    POP(LongMemoryTarget),
    JP(Option<ControlFlowFlag>, LongMemoryTarget),
    JR(Option<ControlFlowFlag>, ShortMemoryTarget),
    DAA,
}

#[derive(Clone, PartialEq, EnumIter)]
enum ControlFlowFlag {
    Zero,
    NonZero,
    Carry,
    NonCarry,
}

/* The order here is the same as it is in the opcodes */
#[derive(EnumIter)]
enum ShortArithmeticTarget {
    B,
    C,
    D,
    E,
    H,
    L,
    ADDR_HL,
    A,
    CONSTANT(u8),
}

enum LongArithmeticTarget {
    BC,
    DE,
    HL,
    SP,
}

enum ArithmeticTarget {
    Short(ShortArithmeticTarget),
    Long(LongArithmeticTarget),
}

/* The order here is the same as it is in the opcodes 0x40-0x7F */
#[derive(EnumIter)]
enum ShortMemoryTarget {
    B,
    C,
    D,
    E,
    H,
    L,
    ADDR_HL,
    A,
    ADDR_BC,
    ADDR_DE,
    ADDR_C, // This is the address 0xFF00 + C
    CONSTANT(u8),
    ADDR_CONSTANT(u16),
}

const ADDR_C_PREFIX: u16 = 0xFF00;

enum LongMemoryTarget {
    AF,
    BC,
    DE,
    HL,
    SP,
    SP_PLUS(i8),
    ADDR_HL,
    CONSTANT(u16),
    ADDR_CONSTANT(u16),
}

enum MemoryTarget {
    Short(ShortMemoryTarget),
    Long(LongMemoryTarget),
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct CPU {
    registers: Registers,
    pc: u16,
    bus: MemoryBus,
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct MemoryBus {
    memory: [u8; 0xFFFF],
}

impl Default for MemoryBus {
    fn default() -> Self {
        MemoryBus {
            memory: [0; 0xFFFF],
        }
    }
}

impl MemoryBus {
    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn write_byte(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }

    fn read_u16(&self, address: u16) -> u16 {
        let first_byte: u16 = self.memory[address as usize].into();
        let second_byte: u16 = self.memory[(address + 1) as usize].into();

        (first_byte << 8) | second_byte
    }

    fn write_u16(&mut self, address: u16, value: u16) {
        self.memory[address as usize] = (value >> 8) as u8;
        self.memory[(address + 1) as usize] = (value & 0xFF) as u8;
    }
}

impl CPU {}

impl Instruction {
    fn from_byte(
        byte: u8,
        prefixed: bool,
        next_byte: u8,
        next_2_bytes: u16,
    ) -> Option<Instruction> {
        if prefixed {
            Instruction::from_byte_prefixed(byte)
        } else {
            Instruction::from_byte_not_prefixed(byte, next_byte, next_2_bytes)
        }
    }

    fn short_arithmetic_target_from_byte(byte: u8) -> Option<ShortArithmeticTarget> {
        ShortArithmeticTarget::iter().nth((byte & 0x07) as usize)
    }

    fn short_memory_targets_from_byte(
        byte: u8,
    ) -> (Option<ShortMemoryTarget>, Option<ShortMemoryTarget>) {
        (
            ShortMemoryTarget::iter().nth((((byte >> 4) - 4) & 0x07) as usize),
            ShortMemoryTarget::iter().nth((byte & 0x07) as usize),
        )
    }

    /* Returns the bit for the opcode from the byte given.
     * This works for BIT, RESET, SET opcodes
     */
    fn get_bit_opcode_arg_from_prefixed_byte(byte: u8) -> u8 {
        (byte & 0x38) >> 3
    }

    fn extra_bytes(&self) -> u16 {
        match self {
            Instruction::LD(_, MemoryTarget::Short(ShortMemoryTarget::CONSTANT(_))) => 1,
            Instruction::LD(_, MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(_))) => 2,
            Instruction::LD(MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(_)), _) => 2,
            Instruction::ADD(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::ADC(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::SUB(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::SBC(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::AND(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::XOR(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::OR(ShortArithmeticTarget::CONSTANT(_)) => 1,
            _ => 0,
        }
    }

    fn from_byte_prefixed(byte: u8) -> Option<Instruction> {
        /* last 3 bits just tell us the register to use */
        match byte & 0xf8 {
            0x00 => Some(Instruction::RLC(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x08 => Some(Instruction::RRC(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x10 => Some(Instruction::RL(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x18 => Some(Instruction::RR(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x20 => Some(Instruction::SLA(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x28 => Some(Instruction::SRA(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x30 => Some(Instruction::SWAP(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x38 => Some(Instruction::SRL(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            (0x40..=0x78) => Some(Instruction::BIT(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            (0x80..=0xB8) => Some(Instruction::RESET(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            (0xC0..=0xF8) => Some(Instruction::SET(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            _ => unreachable!("Shouldn't get here ever as we covered all values which are possible after the and operation"),
        }
    }

    fn from_byte_not_prefixed(byte: u8, next_byte: u8, next_2_bytes: u16) -> Option<Instruction> {
        match byte {
            0x00 => Some(Instruction::NOP),
            0x01 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::BC),
                MemoryTarget::Long(LongMemoryTarget::CONSTANT(next_2_bytes)),
            )),
            0x02 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_BC),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0x03 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::BC,
            ))),
            0x04 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::B,
            ))),
            0x05 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::B,
            ))),
            0x06 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::B),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x07 => Some(Instruction::RLCA),
            0x08 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::ADDR_CONSTANT(next_2_bytes)),
                MemoryTarget::Long(LongMemoryTarget::SP),
            )),
            0x09 => Some(Instruction::ADDHL(LongArithmeticTarget::BC)),
            0x0A => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_BC),
            )),
            0x0B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::BC,
            ))),
            0x0C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::C,
            ))),
            0x0D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::C,
            ))),
            0x0E => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::C),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x0F => Some(Instruction::RRCA),
            0x10 => None, // STOP 0
            0x11 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::DE),
                MemoryTarget::Long(LongMemoryTarget::CONSTANT(next_2_bytes)),
            )),
            0x12 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_DE),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0x13 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::DE,
            ))),
            0x14 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::D,
            ))),
            0x15 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::D,
            ))),
            0x16 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::D),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x17 => Some(Instruction::RLA),
            0x18 => Some(Instruction::JR(
                None,
                ShortMemoryTarget::CONSTANT(next_byte),
            )),
            0x19 => Some(Instruction::ADDHL(LongArithmeticTarget::DE)),
            0x1A => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_DE),
            )),
            0x1B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::DE,
            ))),
            0x1C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::E,
            ))),
            0x1D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::E,
            ))),
            0x1E => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::E),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x1F => Some(Instruction::RRA),
            0x20 => Some(Instruction::JR(
                Some(ControlFlowFlag::NonZero),
                ShortMemoryTarget::CONSTANT(next_byte),
            )),
            0x21 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::HL),
                MemoryTarget::Long(LongMemoryTarget::CONSTANT(next_2_bytes)),
            )),
            0x22 => None, // LD (HL+), A
            0x23 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::HL,
            ))),
            0x24 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::H,
            ))),
            0x25 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::H,
            ))),
            0x26 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::H),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x27 => Some(Instruction::DAA),
            0x28 => Some(Instruction::JR(
                Some(ControlFlowFlag::Zero),
                ShortMemoryTarget::CONSTANT(next_byte),
            )),
            0x29 => Some(Instruction::ADDHL(LongArithmeticTarget::HL)),
            0x2A => None, // LD A, (HL+)
            0x2B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::HL,
            ))),
            0x2C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::L,
            ))),
            0x2D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::L,
            ))),
            0x2E => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::L),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x2F => Some(Instruction::CPL),
            0x30 => Some(Instruction::JR(
                Some(ControlFlowFlag::NonCarry),
                ShortMemoryTarget::CONSTANT(next_byte),
            )),
            0x31 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::SP),
                MemoryTarget::Long(LongMemoryTarget::CONSTANT(next_2_bytes)),
            )),
            0x32 => None, // LD (HL-), A
            0x33 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::SP,
            ))),
            0x34 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::ADDR_HL,
            ))),
            0x35 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::ADDR_HL,
            ))),
            0x36 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_HL),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x37 => Some(Instruction::SCF),
            0x38 => Some(Instruction::JR(
                Some(ControlFlowFlag::Carry),
                ShortMemoryTarget::CONSTANT(next_byte),
            )),
            0x39 => Some(Instruction::ADDHL(LongArithmeticTarget::SP)),
            0x3A => None, // LD A, (HL-)
            0x3B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::SP,
            ))),
            0x3C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::A,
            ))),
            0x3D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::A,
            ))),
            0x3E => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::CONSTANT(next_byte)),
            )),
            0x3F => Some(Instruction::CCF),
            0x40..=0x75 | 0x77..=0x7F => {
                let (dest, src) = Instruction::short_memory_targets_from_byte(byte);
                Some(Instruction::LD(
                    MemoryTarget::Short(dest?),
                    MemoryTarget::Short(src?),
                ))
            }
            0x76 => None, // HALT
            0x80..=0x87 => Some(Instruction::ADD(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x88..=0x8F => Some(Instruction::ADC(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x90..=0x97 => Some(Instruction::SUB(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0x98..=0x9F => Some(Instruction::SBC(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0xA0..=0xA7 => Some(Instruction::AND(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0xA8..=0xAF => Some(Instruction::XOR(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0xB0..=0xB7 => Some(Instruction::OR(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0xB8..=0xBF => Some(Instruction::CP(
                Instruction::short_arithmetic_target_from_byte(byte)?,
            )),
            0xC0 => None, // RET NZ
            0xC1 => Some(Instruction::POP(LongMemoryTarget::BC)),
            0xC2 => Some(Instruction::JP(
                Some(ControlFlowFlag::NonZero),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xC3 => Some(Instruction::JP(
                None,
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xC4 => None, // CALL NZ, a16
            0xC5 => Some(Instruction::PUSH(LongMemoryTarget::BC)),
            0xC6 => Some(Instruction::ADD(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xC7 => None, // RST 00H
            0xC8 => None, // RET Z
            0xC9 => None, // RET
            0xCA => Some(Instruction::JP(
                Some(ControlFlowFlag::Zero),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xCB => panic!("Shouldn't get here, should be handled in another function!"), // PREFIX CB
            0xCC => None, // CALL Z, a16
            0xCD => None, // CALL a16
            0xCE => Some(Instruction::ADC(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xCF => None, // RST 08H
            0xD0 => None, // RET NC
            0xD1 => Some(Instruction::POP(LongMemoryTarget::DE)),
            0xD2 => Some(Instruction::JP(
                Some(ControlFlowFlag::NonCarry),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xD4 => None, // CALL NC, a16
            0xD5 => Some(Instruction::PUSH(LongMemoryTarget::DE)),
            0xD6 => Some(Instruction::SUB(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xD7 => None, // RST 10H
            0xD8 => None, // RET C
            0xD9 => None, // RETI
            0xDA => Some(Instruction::JP(
                Some(ControlFlowFlag::Carry),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xDC => None, // CALL C, a16
            0xDE => Some(Instruction::SBC(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xDF => None, // RST 18H
            0xE0 => None, // LDH (a8), A
            0xE1 => Some(Instruction::POP(LongMemoryTarget::HL)),
            0xE2 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_C),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0xE5 => Some(Instruction::PUSH(LongMemoryTarget::HL)),
            0xE6 => Some(Instruction::AND(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xE7 => None, // RST 20H
            0xE8 => None, // ADD SP, r8
            0xE9 => Some(Instruction::JP(None, LongMemoryTarget::ADDR_HL)),
            0xEA => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(next_2_bytes)),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0xEE => Some(Instruction::XOR(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xEF => None, // RST 28H
            0xF0 => None, // LDH A, (a8)
            0xF1 => Some(Instruction::POP(LongMemoryTarget::AF)),
            0xF2 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_C),
            )),
            0xF3 => None, // DI
            0xF5 => Some(Instruction::PUSH(LongMemoryTarget::AF)),
            0xF6 => Some(Instruction::OR(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xF7 => None, // RST 30H
            0xF8 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::SP_PLUS(next_byte as i8)),
                MemoryTarget::Long(LongMemoryTarget::HL),
            )),
            0xF9 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::SP),
                MemoryTarget::Long(LongMemoryTarget::HL),
            )),
            0xFA => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(next_2_bytes)),
            )),
            0xFB => None, // EI
            0xFE => None, // CP d8
            0xFF => None, // RST 38H

            /* mapping for undefined instructions */
            0xD3 | 0xDB | 0xDD | 0xE3 | 0xE4 | 0xEB | 0xEC | 0xED | 0xF4 | 0xFC | 0xFD => None,
        }
    }
}

impl CPU {
    pub fn step(&mut self) {
        let mut instruction_byte = self.bus.read_byte(self.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.bus.read_byte(self.pc + 1);
        }
        let end_of_instruction = self.pc + 1 + (prefixed as u16);
        let next_byte = self.bus.read_byte(end_of_instruction);
        let next_2_bytes = self.bus.read_u16(end_of_instruction);

        let next_pc = if let Some(instruction) =
            Instruction::from_byte(instruction_byte, prefixed, next_byte, next_2_bytes)
        {
            let extra_bytes = instruction.extra_bytes();
            self.execute(instruction);
            end_of_instruction + extra_bytes
        } else {
            let description = format!(
                "0x{}{:x}",
                if prefixed { "cb" } else { "" },
                instruction_byte
            );
            panic!("Unkown instruction found for: {}", description)
        };

        self.pc = next_pc;
    }
    fn get_short_arithmetic_target_value(&self, target: &ShortArithmeticTarget) -> u8 {
        match target {
            ShortArithmeticTarget::A => self.registers.a,
            ShortArithmeticTarget::B => self.registers.b,
            ShortArithmeticTarget::C => self.registers.c,
            ShortArithmeticTarget::D => self.registers.d,
            ShortArithmeticTarget::E => self.registers.e,
            ShortArithmeticTarget::H => self.registers.h,
            ShortArithmeticTarget::L => self.registers.l,
            // For calculations we can convert hl to u8
            ShortArithmeticTarget::ADDR_HL => self.bus.read_byte(self.registers.get_hl()),
            ShortArithmeticTarget::CONSTANT(val) => *val,
        }
    }

    fn set_short_arithmetic_target_value(&mut self, target: &ShortArithmeticTarget, value: u8) {
        match target {
            ShortArithmeticTarget::A => self.registers.a = value,
            ShortArithmeticTarget::B => self.registers.b = value,
            ShortArithmeticTarget::C => self.registers.c = value,
            ShortArithmeticTarget::D => self.registers.d = value,
            ShortArithmeticTarget::E => self.registers.e = value,
            ShortArithmeticTarget::H => self.registers.h = value,
            ShortArithmeticTarget::L => self.registers.l = value,
            ShortArithmeticTarget::ADDR_HL => self.bus.write_byte(self.registers.get_hl(), value),
            ShortArithmeticTarget::CONSTANT(_) => panic!("Should never set value of constant"),
        };
    }

    fn get_long_arithmetic_target_value(&self, target: &LongArithmeticTarget) -> u16 {
        match target {
            LongArithmeticTarget::BC => self.registers.get_bc(),
            LongArithmeticTarget::DE => self.registers.get_de(),
            LongArithmeticTarget::HL => self.registers.get_hl(),
            LongArithmeticTarget::SP => self.registers.get_sp(),
        }
    }

    fn set_long_arithmetic_target_value(&mut self, target: &LongArithmeticTarget, value: u16) {
        match target {
            LongArithmeticTarget::BC => self.registers.set_bc(value),
            LongArithmeticTarget::DE => self.registers.set_de(value),
            LongArithmeticTarget::HL => self.registers.set_hl(value),
            LongArithmeticTarget::SP => self.registers.set_sp(value),
        };
    }

    fn get_short_memory_target_value(&self, target: &ShortMemoryTarget) -> u8 {
        match target {
            ShortMemoryTarget::A => self.registers.a,
            ShortMemoryTarget::B => self.registers.b,
            ShortMemoryTarget::C => self.registers.c,
            ShortMemoryTarget::D => self.registers.d,
            ShortMemoryTarget::E => self.registers.e,
            ShortMemoryTarget::H => self.registers.h,
            ShortMemoryTarget::L => self.registers.l,
            ShortMemoryTarget::ADDR_HL => self.bus.read_byte(self.registers.get_hl()),
            ShortMemoryTarget::ADDR_BC => self.bus.read_byte(self.registers.get_bc()),
            ShortMemoryTarget::ADDR_DE => self.bus.read_byte(self.registers.get_de()),
            ShortMemoryTarget::ADDR_C => {
                self.bus.read_byte(ADDR_C_PREFIX + self.registers.c as u16)
            }
            ShortMemoryTarget::CONSTANT(val) => *val,
            ShortMemoryTarget::ADDR_CONSTANT(addr) => self.bus.read_byte(*addr),
        }
    }

    fn set_short_memory_target_value(&mut self, target: &ShortMemoryTarget, value: u8) {
        match target {
            ShortMemoryTarget::A => self.registers.a = value,
            ShortMemoryTarget::B => self.registers.b = value,
            ShortMemoryTarget::C => self.registers.c = value,
            ShortMemoryTarget::D => self.registers.d = value,
            ShortMemoryTarget::E => self.registers.e = value,
            ShortMemoryTarget::H => self.registers.h = value,
            ShortMemoryTarget::L => self.registers.l = value,
            ShortMemoryTarget::ADDR_HL => self.bus.write_byte(self.registers.get_hl(), value),
            ShortMemoryTarget::ADDR_BC => self.bus.write_byte(self.registers.get_bc(), value),
            ShortMemoryTarget::ADDR_DE => self.bus.write_byte(self.registers.get_de(), value),
            ShortMemoryTarget::ADDR_C => self
                .bus
                .write_byte(ADDR_C_PREFIX + self.registers.c as u16, value),
            ShortMemoryTarget::CONSTANT(_) => panic!("Should never set value of constant"),
            ShortMemoryTarget::ADDR_CONSTANT(addr) => self.bus.write_byte(*addr, value),
        };
    }

    fn get_long_memory_target_value(&self, target: &LongMemoryTarget) -> u16 {
        match target {
            LongMemoryTarget::AF => self.registers.get_af(),
            LongMemoryTarget::BC => self.registers.get_bc(),
            LongMemoryTarget::DE => self.registers.get_de(),
            LongMemoryTarget::HL => self.registers.get_hl(),
            LongMemoryTarget::SP => self.registers.get_sp(),
            LongMemoryTarget::SP_PLUS(val) => {
                (self.registers.get_sp() as i16).wrapping_add(*val as i16) as u16
            }
            LongMemoryTarget::ADDR_HL => self.bus.read_u16(self.registers.get_hl()),
            LongMemoryTarget::CONSTANT(val) => *val,
            LongMemoryTarget::ADDR_CONSTANT(addr) => self.bus.read_u16(*addr),
        }
    }

    fn set_long_memory_target_value(&mut self, target: &LongMemoryTarget, value: u16) {
        match target {
            LongMemoryTarget::AF => self.registers.set_af(value),
            LongMemoryTarget::BC => self.registers.set_bc(value),
            LongMemoryTarget::DE => self.registers.set_de(value),
            LongMemoryTarget::HL => self.registers.set_hl(value),
            LongMemoryTarget::SP => self.registers.set_sp(value),
            LongMemoryTarget::SP_PLUS(_) => {
                panic!("Shouldn't try to set SP_PLUS long memory target")
            }
            LongMemoryTarget::ADDR_HL => self.bus.write_u16(self.registers.get_hl(), value),
            LongMemoryTarget::CONSTANT(_) => {
                panic!("Shouldn't try to set constant long memory target")
            }
            LongMemoryTarget::ADDR_CONSTANT(addr) => self.bus.write_u16(*addr, value),
        };
    }

    fn get_control_flow_flag_value(&self, flag: &ControlFlowFlag) -> bool {
        match flag {
            ControlFlowFlag::Zero => self.registers.f.zero,
            ControlFlowFlag::NonZero => !self.registers.f.zero,
            ControlFlowFlag::Carry => self.registers.f.carry,
            ControlFlowFlag::NonCarry => !self.registers.f.carry,
        }
    }

    #[cfg(test)]
    fn set_control_flow_flag_value(&mut self, flag: &ControlFlowFlag) {
        match flag {
            ControlFlowFlag::Zero | ControlFlowFlag::NonZero => {
                self.registers.f.zero = *flag == ControlFlowFlag::Zero;
            }
            ControlFlowFlag::Carry | ControlFlowFlag::NonCarry => {
                self.registers.f.carry = *flag == ControlFlowFlag::Carry;
            }
        }
    }

    fn execute(&mut self, instruction: Instruction) {
        match instruction {
            Instruction::ADD(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.add(value);
            }
            Instruction::ADC(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.adc(value);
            }
            Instruction::SUB(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.sub(value);
            }
            Instruction::SBC(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.sbc(value);
            }
            Instruction::AND(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.and(value);
            }
            Instruction::OR(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.or(value);
            }
            Instruction::XOR(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.registers.a = self.xor(value);
            }
            Instruction::CP(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                // Just like sub without updating the A register
                self.sub(value);
            }
            Instruction::INC(target) => match target {
                ArithmeticTarget::Short(s) => {
                    let value = self.get_short_arithmetic_target_value(&s);
                    let result = self.inc(value);
                    self.set_short_arithmetic_target_value(&s, result);
                }
                ArithmeticTarget::Long(l) => {
                    let value = self.get_long_arithmetic_target_value(&l);
                    self.set_long_arithmetic_target_value(&l, value.wrapping_add(1));
                }
            },
            Instruction::DEC(target) => match target {
                ArithmeticTarget::Short(s) => {
                    let value = self.get_short_arithmetic_target_value(&s);
                    let result = self.dec(value);
                    self.set_short_arithmetic_target_value(&s, result);
                }
                ArithmeticTarget::Long(l) => {
                    let value = self.get_long_arithmetic_target_value(&l);
                    self.set_long_arithmetic_target_value(&l, value.wrapping_sub(1));
                }
            },
            Instruction::ADDHL(target) => {
                let value = self.get_long_arithmetic_target_value(&target);
                let result = self.addhl(value);
                self.registers.set_hl(result);
            }
            Instruction::CCF => {
                self.registers.f.carry = !self.registers.f.carry;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;
            }
            Instruction::SCF => {
                self.registers.f.carry = true;
                self.registers.f.subtract = false;
                self.registers.f.half_carry = false;
            }
            Instruction::NOP => {}
            Instruction::CPL => {
                self.registers.a = self.cpl();
            }
            Instruction::RRA => {
                self.registers.a = self.rr(self.registers.a);

                // On "A" variant of rotate operations zero is always false.
                self.registers.f.zero = false;
            }
            Instruction::RRCA => {
                self.registers.a = self.rrc(self.registers.a);

                // On "A" variant of rotate operations zero is always false.
                self.registers.f.zero = false;
            }
            Instruction::RLA => {
                self.registers.a = self.rl(self.registers.a);

                // On "A" variant of rotate operations zero is always false.
                self.registers.f.zero = false;
            }
            Instruction::RLCA => {
                self.registers.a = self.rlc(self.registers.a);

                // On "A" variant of rotate operations zero is always false.
                self.registers.f.zero = false;
            }
            Instruction::RR(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.rr(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::RRC(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.rrc(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::RL(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.rl(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::RLC(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.rlc(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::SLA(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.sla(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::SRA(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.sra(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::SRL(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.srl(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::SWAP(target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.swap(value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::BIT(bit, target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                self.bit(bit, value);
            }
            Instruction::SET(bit, target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.set(bit, value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::RESET(bit, target) => {
                let value = self.get_short_arithmetic_target_value(&target);
                let result = self.reset(bit, value);
                self.set_short_arithmetic_target_value(&target, result);
            }
            Instruction::PUSH(target) => {
                self.push(self.get_long_memory_target_value(&target));
            }
            Instruction::POP(target) => {
                let value = self.pop();
                self.set_long_memory_target_value(&target, value);
            }
            Instruction::LD(dest, src) => self.ld(dest, src),
            Instruction::JP(flag_option, target) => {
                if let Some(flag) = flag_option {
                    if self.get_control_flow_flag_value(&flag) {
                        self.pc = self.get_long_memory_target_value(&target);
                    }
                } else {
                    self.pc = self.get_long_memory_target_value(&target);
                }
            }
            Instruction::JR(flag_option, target) => {
                if let Some(flag) = flag_option {
                    if self.get_control_flow_flag_value(&flag) {
                        self.pc = (self.pc as i16)
                            .wrapping_add(self.get_short_memory_target_value(&target) as i8 as i16)
                            as u16;
                    }
                } else {
                    self.pc = (self.pc as i16)
                        .wrapping_add(self.get_short_memory_target_value(&target) as i8 as i16)
                        as u16;
                }
            }
            Instruction::DAA => {
                self.daa();
            }
        }
    }

    fn add(&mut self, value: u8) -> u8 {
        let (result, did_overflow) = self.registers.a.overflowing_add(value);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;

        result
    }

    fn adc(&mut self, value: u8) -> u8 {
        let carry = self.registers.f.carry as u8;

        let (inter_result, inter_did_overflow) = value.overflowing_add(carry);
        let (result, did_overflow) = self.registers.a.overflowing_add(inter_result);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow || inter_did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) + carry > 0xF;

        result
    }

    fn sub(&mut self, value: u8) -> u8 {
        let (result, did_overflow) = self.registers.a.overflowing_sub(value);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = true;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF).wrapping_sub(value & 0xF) > 0xF;

        result
    }

    fn sbc(&mut self, value: u8) -> u8 {
        let carry = self.registers.f.carry as u8;

        let (inter_result, inter_did_overflow) = self.registers.a.overflowing_sub(value);
        let (result, did_overflow) = inter_result.overflowing_sub(carry);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = true;
        self.registers.f.carry = did_overflow || inter_did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF)
            .wrapping_sub(value & 0xF)
            .wrapping_sub(carry)
            > 0xF;

        result
    }

    fn and(&mut self, value: u8) -> u8 {
        let result = self.registers.a & value;

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;

        // This seems to be always true after the and operation
        self.registers.f.half_carry = true;

        result
    }

    fn or(&mut self, value: u8) -> u8 {
        let result = self.registers.a | value;

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;
        self.registers.f.half_carry = false;

        result
    }

    fn xor(&mut self, value: u8) -> u8 {
        let result = self.registers.a ^ value;

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = false;
        self.registers.f.half_carry = false;

        result
    }

    fn inc(&mut self, value: u8) -> u8 {
        let result = value.wrapping_add(1);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;

        // No carry on inc operation
        self.registers.f.carry = false;

        result
    }

    fn dec(&mut self, value: u8) -> u8 {
        let result = value.wrapping_sub(1);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = true;

        // No carry on dec operation
        self.registers.f.carry = false;

        result
    }

    fn addhl(&mut self, value: u16) -> u16 {
        let (result, did_overflow) = self.registers.get_hl().overflowing_add(value);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.get_hl() & 0xFFF) + (value & 0xFFF) > 0xFFF;

        result
    }

    fn cpl(&mut self) -> u8 {
        self.registers.f.subtract = true;
        self.registers.f.half_carry = true;

        !self.registers.a
    }

    fn rr(&mut self, value: u8) -> u8 {
        let new_carry = value & 0x1 == 1;
        let mut new_value = value >> 1;
        new_value |= (self.registers.f.carry as u8) << 7;

        self.registers.f.carry = new_carry;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn rrc(&mut self, value: u8) -> u8 {
        let old_bit = value & 0x1;
        let mut new_value = value >> 1;
        new_value |= old_bit << 7;

        self.registers.f.carry = old_bit == 1;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn rl(&mut self, value: u8) -> u8 {
        let new_carry = value & 0x80 != 0;
        let mut new_value = value << 1;
        new_value |= self.registers.f.carry as u8;

        self.registers.f.carry = new_carry;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn rlc(&mut self, value: u8) -> u8 {
        let old_bit = (value & 0x80) >> 7;
        let mut new_value = value << 1;
        new_value |= old_bit;

        self.registers.f.carry = old_bit != 0;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn sla(&mut self, value: u8) -> u8 {
        // This is like rlc but zero the LSB
        let mut new_value = self.rlc(value);

        // Make the LSB 0
        new_value &= !(1);

        // Recalculate the zero bit
        self.registers.f.zero = new_value == 0;

        new_value
    }

    fn sra(&mut self, value: u8) -> u8 {
        let old_lsb = value & 0x1;
        let old_msb = value & 0x80;
        let mut new_value = value >> 1;
        // Keep MSB
        new_value |= old_msb;

        self.registers.f.carry = old_lsb == 1;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn srl(&mut self, value: u8) -> u8 {
        // This is like sra just zero MSB
        let mut new_value = self.sra(value);

        // Make MSB zero
        new_value &= !0x80;

        // Recalculate the zero bit
        self.registers.f.zero = new_value == 0;

        new_value
    }

    fn swap(&mut self, value: u8) -> u8 {
        let ms_nibble = value & 0xF0;
        let ls_nibble = value & 0x0F;
        let new_value = (ls_nibble << 4) | (ms_nibble >> 4);

        self.registers.f.carry = false;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn bit(&mut self, bit: u8, value: u8) {
        self.registers.f.half_carry = true;
        self.registers.f.subtract = false;
        self.registers.f.zero = (value & (1 << bit)) == 0;
    }

    fn set(&mut self, bit: u8, value: u8) -> u8 {
        value | (1 << bit)
    }

    fn reset(&mut self, bit: u8, value: u8) -> u8 {
        value & !(1 << bit)
    }

    fn push(&mut self, value: u16) {
        self.registers.sp = self.registers.sp.wrapping_sub(2);
        self.bus.write_u16(self.registers.sp, value);
    }

    fn pop(&mut self) -> u16 {
        let value = self.bus.read_u16(self.registers.sp);
        self.registers.sp = self.registers.sp.wrapping_add(2);

        value
    }

    fn ld(&mut self, dest: MemoryTarget, src: MemoryTarget) {
        match (dest, src) {
            (MemoryTarget::Short(s_dest), MemoryTarget::Short(s_src)) => {
                let value = self.get_short_memory_target_value(&s_src);
                self.set_short_memory_target_value(&s_dest, value);
            }
            (MemoryTarget::Long(l_dest), MemoryTarget::Long(l_src)) => {
                let value = self.get_long_memory_target_value(&l_src);
                self.set_long_memory_target_value(&l_dest, value);
            }
            _ => panic!("Can't LD short to long or long to short"),
        }
    }

    fn daa(&mut self) {
        let mut a = self.registers.a;

        // after an addition, adjust if (half-)carry occurred or if result is out of bounds
        if !self.registers.f.subtract {
            if self.registers.f.carry || a > 0x99 {
                a = a.wrapping_add(0x60);
                self.registers.f.carry = true;
            }
            if self.registers.f.half_carry || (a & 0x0f) > 0x09 {
                a = a.wrapping_add(0x6);
            }
        }
        // after a subtraction, only adjust if (half-)carry occurred
        else {
            if self.registers.f.carry {
                a = a.wrapping_sub(0x60);
            }
            if self.registers.f.half_carry {
                a = a.wrapping_sub(0x6);
            }
        }
        self.registers.a = a;

        // these flags are always updated
        self.registers.f.zero = a == 0; // the usual zero flag
        self.registers.f.half_carry = false; // half-carry flag is always cleared
    }
}

#[cfg(test)]
mod cpu_tests {
    use crate::cpu::*;

    fn assert_flags_arent_changed(cpu: &CPU) {
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_add_c() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADD(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, (7 + 6) as u8);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_add_c_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 7;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADD(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, u8::MAX.wrapping_add(7));
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_add_c_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 2;
        cpu.registers.a = 15;
        cpu.execute(Instruction::ADD(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 2);
        assert_eq!(cpu.registers.a, (2 + 15) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 7);
        assert_eq!(cpu.registers.a, (7 + 6 + 1) as u8);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_adc_b_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = 7;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 7);
        assert_eq!(cpu.registers.a, u8::MAX.wrapping_add(7).wrapping_add(1));
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b_double_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = u8::MAX;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, u8::MAX);
        assert_eq!(
            cpu.registers.a,
            u8::MAX.wrapping_add(u8::MAX).wrapping_add(1)
        );
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = 2;
        cpu.registers.a = 15;
        cpu.execute(Instruction::ADC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 2);
        assert_eq!(cpu.registers.a, (2 + 15 + 1) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = 6;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SUB(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 6);
        assert_eq!(cpu.registers.a, (7 - 6) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = u8::MAX;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SUB(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, u8::MAX);
        assert_eq!(cpu.registers.a, 7_u8.wrapping_sub(u8::MAX));
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = 15;
        cpu.registers.a = 1;
        cpu.execute(Instruction::SUB(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 15);
        assert_eq!(cpu.registers.a, 1_u8.wrapping_sub(15));

        // I don't think there is a way to do half carry with no carry
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.e = 6;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SBC(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 6);
        assert_eq!(cpu.registers.a, (7 - 6 - 1) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.e = 7;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SBC(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 7);
        assert_eq!(cpu.registers.a, ((7 - 7) as u8).wrapping_sub(1));
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e_another_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.e = u8::MAX;
        cpu.registers.a = 0;
        cpu.execute(Instruction::SBC(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, u8::MAX);
        assert_eq!(cpu.registers.a, 0_u8.wrapping_sub(u8::MAX).wrapping_sub(1));
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);

        /*
        0 - 255 - 1
        \_____/  /
             1 - 1 => 0
        */
        assert!(cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.e = 15;
        cpu.registers.a = 2;
        cpu.execute(Instruction::SBC(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 15);
        assert_eq!(cpu.registers.a, 2_u8.wrapping_sub(15).wrapping_sub(1));

        // I don't think there is a way to do half carry with no carry
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_and_h_non_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.h = 0xf0;
        cpu.registers.a = 0x1f;
        cpu.execute(Instruction::AND(ShortArithmeticTarget::H));

        assert_eq!(cpu.registers.h, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 & 0x1f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);

        // Always on after "and" operation
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_and_h_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.h = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::AND(ShortArithmeticTarget::H));

        assert_eq!(cpu.registers.h, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 & 0x0f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);

        // Always on after "and" operation
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_or_l_non_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.l = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::OR(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 | 0x0f);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_or_l_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.l = 0x0;
        cpu.registers.a = 0x0;
        cpu.execute(Instruction::OR(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0x0);
        assert_eq!(cpu.registers.a, 0x0);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_xor_a() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0x1f; // Shouldn't matter what value we have here
        cpu.execute(Instruction::XOR(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x1f ^ 0x1f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_xor_b_non_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.b = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::XOR(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 ^ 0x0f);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_xor_b_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.b = 0xf0;
        cpu.registers.a = 0xf0;
        cpu.execute(Instruction::XOR(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 ^ 0xf0);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_eq() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 0xf2;
        cpu.registers.a = 0xf2;
        cpu.execute(Instruction::CP(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf2);
        assert_eq!(cpu.registers.a, 0xf2);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_larger() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 0xf3;
        cpu.registers.a = 0xf2;
        cpu.execute(Instruction::CP(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf3);
        assert_eq!(cpu.registers.a, 0xf2);

        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_smaller() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 0xf2;
        cpu.registers.a = 0xf3;
        cpu.execute(Instruction::CP(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf2);
        assert_eq!(cpu.registers.a, 0xf3);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = 7;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, 7 + 1);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_inc_d_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = 15;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, 15 + 1);

        // half-carry shouldn't change on "inc" operation
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_inc_d_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = u8::MAX;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, u8::MAX.wrapping_add(1));

        // Shouldn't indicate carry on "inc" operation
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        // Shouldn't change on "inc" operation
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e() {
        let mut cpu: CPU = Default::default();
        cpu.registers.e = 7;
        cpu.execute(Instruction::DEC(ArithmeticTarget::Short(
            ShortArithmeticTarget::E,
        )));

        assert_eq!(cpu.registers.e, 7 - 1);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.e = 1;
        cpu.execute(Instruction::DEC(ArithmeticTarget::Short(
            ShortArithmeticTarget::E,
        )));

        assert_eq!(cpu.registers.e, 1 - 1);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.e = 0;
        cpu.execute(Instruction::DEC(ArithmeticTarget::Short(
            ShortArithmeticTarget::E,
        )));

        assert_eq!(cpu.registers.e, 0_u8.wrapping_sub(1));

        // Shouldn't indicate carry on "dec" operation
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        // Shouldn't change on "dec" operation
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_addhl_bc() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_bc(8942);
        cpu.registers.set_hl(10000);
        cpu.execute(Instruction::ADDHL(LongArithmeticTarget::BC));

        assert_eq!(cpu.registers.get_bc(), 8942);
        assert_eq!(cpu.registers.get_hl(), 10000 + 8942);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_addhl_bc_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_bc(0xF00);
        cpu.registers.set_hl(0xF00);
        cpu.execute(Instruction::ADDHL(LongArithmeticTarget::BC));

        assert_eq!(cpu.registers.get_bc(), 0xF00);
        assert_eq!(cpu.registers.get_hl(), 0xF00 + 0xF00);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_addhl_bc_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_bc(1);
        cpu.registers.set_hl(u16::MAX);
        cpu.execute(Instruction::ADDHL(LongArithmeticTarget::BC));

        assert_eq!(cpu.registers.get_bc(), 1);
        assert_eq!(cpu.registers.get_hl(), u16::MAX.wrapping_add(1));

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_de() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_de(7);
        cpu.execute(Instruction::INC(ArithmeticTarget::Long(
            LongArithmeticTarget::DE,
        )));

        assert_eq!(cpu.registers.get_de(), 7 + 1);

        // Shouldn't update flags register on long inc
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_inc_de_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_de(0xFFF);
        cpu.execute(Instruction::INC(ArithmeticTarget::Long(
            LongArithmeticTarget::DE,
        )));

        assert_eq!(cpu.registers.get_de(), 0xFFF + 1);

        // Shouldn't update flags register on long inc
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_inc_de_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_de(u16::MAX);
        cpu.execute(Instruction::INC(ArithmeticTarget::Long(
            LongArithmeticTarget::DE,
        )));

        assert_eq!(cpu.registers.get_de(), u16::MAX.wrapping_add(1));

        // Shouldn't update flags register on long inc
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_dec_hl() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_hl(7);
        cpu.execute(Instruction::DEC(ArithmeticTarget::Long(
            LongArithmeticTarget::HL,
        )));

        assert_eq!(cpu.registers.get_hl(), 7 - 1);

        // Shouldn't update flags register on long dec
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_dec_hl_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_hl(0);
        cpu.execute(Instruction::DEC(ArithmeticTarget::Long(
            LongArithmeticTarget::HL,
        )));

        assert_eq!(cpu.registers.get_hl(), 0_u16.wrapping_sub(1));

        // Shouldn't update flags register on long dec
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_dec_hl_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.set_hl(1);
        cpu.execute(Instruction::DEC(ArithmeticTarget::Long(
            LongArithmeticTarget::HL,
        )));

        assert_eq!(cpu.registers.get_hl(), 0);

        // Shouldn't update flags register on long dec
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_swap_a() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0x73;
        cpu.execute(Instruction::SWAP(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x37);

        // Everything should be false except for zero which depends on the value
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_swap_a_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0x0;
        cpu.execute(Instruction::SWAP(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x0);

        // Everything should be false except for zero which depends on the value
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_ccf() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.f.zero = true;
        cpu.execute(Instruction::CCF);

        assert_eq!(cpu.registers.a, 0x0);

        // Carry flips and both subtract and half-carry should be false (zero doesn't change)
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_scf() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.f.zero = true;
        cpu.execute(Instruction::SCF);

        assert_eq!(cpu.registers.a, 0x0);

        // Carry is set to true and both subtract and half-carry should be false (zero doesn't change)
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_nop() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.f.zero = true;
        cpu.registers.f.half_carry = true;
        cpu.registers.f.subtract = true;
        cpu.registers.a = 0x5f;

        let cpu_copy = cpu.clone();

        cpu.execute(Instruction::NOP);

        assert_eq!(cpu, cpu_copy);
    }

    #[test]
    fn test_cpl() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::CPL);

        assert_eq!(cpu.registers.a, 0b01010001);

        // Only change subtract and half-carry to true the others aren't changed
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rra() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RRA);

        assert_eq!(cpu.registers.a, 0b11010111);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rra_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.a = 0b00000001;

        cpu.execute(Instruction::RRA);

        assert_eq!(cpu.registers.a, 0b00000000);

        // Only "calculate" carry flag which should true (zero is false anyway)
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rrca() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RRCA);

        assert_eq!(cpu.registers.a, 0b01010111);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rrca_change_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.a = 0b00000001;

        cpu.execute(Instruction::RRCA);

        assert_eq!(cpu.registers.a, 0b10000000);

        // Only "calculate" carry flag which should true (zero is always false)
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rla() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b01110101;

        cpu.execute(Instruction::RLA);

        assert_eq!(cpu.registers.a, 0b11101011);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rla_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.a = 0b10000000;

        cpu.execute(Instruction::RLA);

        assert_eq!(cpu.registers.a, 0b00000000);

        // Only "calculate" carry flag which should true (zero is false anyway)
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rlca() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b01110101;

        cpu.execute(Instruction::RLCA);

        assert_eq!(cpu.registers.a, 0b11101010);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rlca_change_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.a = 0b10000000;

        cpu.execute(Instruction::RLCA);

        assert_eq!(cpu.registers.a, 0b00000001);

        // Only "calculate" carry flag which should true (zero is always false)
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rr_a() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RR(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0b11010111);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rr_a_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.a = 0b00000001;

        cpu.execute(Instruction::RR(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0b00000000);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rrc_b() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = 0b10101110;

        cpu.execute(Instruction::RRC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0b01010111);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rrc_b_change_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.b = 0b00000001;

        cpu.execute(Instruction::RRC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0b10000000);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rrc_b_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.b = 0b00000000;

        cpu.execute(Instruction::RRC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0b00000000);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rl_c() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.c = 0b01110101;

        cpu.execute(Instruction::RL(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0b11101011);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rl_c_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.c = 0b10000000;

        cpu.execute(Instruction::RL(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0b00000000);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rlc_d() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.d = 0b01110101;

        cpu.execute(Instruction::RLC(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 0b11101010);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rlc_d_change_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.d = 0b10000000;

        cpu.execute(Instruction::RLC(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 0b00000001);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_rlc_d_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.d = 0b00000000;

        cpu.execute(Instruction::RLC(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 0b00000000);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sla_e() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.e = 0b11110101;

        cpu.execute(Instruction::SLA(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 0b11101010);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sla_e_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.e = 0b00000000;

        cpu.execute(Instruction::SLA(ShortArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 0b00000000);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sra_h() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.h = 0b10000001;

        cpu.execute(Instruction::SRA(ShortArithmeticTarget::H));

        assert_eq!(cpu.registers.h, 0b11000000);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sra_h_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.h = 0b00000000;

        cpu.execute(Instruction::SRA(ShortArithmeticTarget::H));

        assert_eq!(cpu.registers.h, 0b00000000);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_srl_l() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = false;
        cpu.registers.l = 0b10101111;

        cpu.execute(Instruction::SRL(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0b01010111);

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_srl_l_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.l = 0b00000000;

        cpu.execute(Instruction::SRL(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0b00000000);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_bit_a() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0b10101010;

        for bit in 0_u8..=7 {
            cpu.execute(Instruction::BIT(bit, ShortArithmeticTarget::A));

            // Shouldn't change
            assert_eq!(cpu.registers.a, 0b10101010);

            if bit % 2 == 0 {
                assert!(cpu.registers.f.zero);
            } else {
                assert!(!cpu.registers.f.zero);
            }

            assert!(!cpu.registers.f.carry);
            assert!(!cpu.registers.f.subtract);
            assert!(cpu.registers.f.half_carry);
        }
    }

    #[test]
    fn test_set_b() {
        let mut cpu: CPU = Default::default();
        let mut val = 0b10101010;
        cpu.registers.b = val;

        for bit in 0_u8..=7 {
            cpu.execute(Instruction::SET(bit, ShortArithmeticTarget::B));
            val |= 1 << bit;

            // Shouldn't change
            assert_eq!(cpu.registers.b, val);

            assert_flags_arent_changed(&cpu);
        }
    }

    #[test]
    fn test_reset_c() {
        let mut cpu: CPU = Default::default();
        let mut val = 0b10101010;
        cpu.registers.c = val;

        for bit in 0_u8..=7 {
            cpu.execute(Instruction::RESET(bit, ShortArithmeticTarget::C));
            val &= !(1 << bit);

            // Shouldn't change
            assert_eq!(cpu.registers.c, val);

            assert_flags_arent_changed(&cpu);
        }
    }

    #[test]
    fn test_push_hl() {
        const VAL: u16 = 0x1234;
        const ADDR: u16 = 0x0123;

        let mut cpu: CPU = Default::default();
        cpu.registers.set_hl(VAL);
        cpu.registers.set_sp(ADDR);

        cpu.execute(Instruction::PUSH(LongMemoryTarget::HL));

        // Shouldn't change
        assert_eq!(cpu.registers.get_hl(), VAL);
        assert_flags_arent_changed(&cpu);

        // SP get 2 subtracted from it
        assert_eq!(cpu.registers.get_sp(), ADDR - 2);
        assert_eq!(cpu.bus.read_u16(ADDR - 2), VAL);
    }

    #[test]
    fn test_pop_bc() {
        const VAL: u16 = 0x1234;
        const ADDR: u16 = 0x0123;

        let mut cpu: CPU = Default::default();
        cpu.bus.write_u16(ADDR, VAL);
        cpu.registers.set_sp(ADDR);

        cpu.execute(Instruction::POP(LongMemoryTarget::BC));

        // Shouldn't change
        assert_eq!(cpu.bus.read_u16(ADDR), VAL);
        assert_flags_arent_changed(&cpu);

        // SP get 2 subtracted from it
        assert_eq!(cpu.registers.get_sp(), ADDR + 2);
        assert_eq!(cpu.registers.get_bc(), VAL);
    }

    #[test]
    fn test_ld_c_addr_de() {
        const VAL: u8 = 0x34;
        const ADDR: u16 = 0x0123;

        let mut cpu: CPU = Default::default();
        cpu.bus.write_byte(ADDR, VAL);
        cpu.registers.set_de(ADDR);

        cpu.execute(Instruction::LD(
            MemoryTarget::Short(ShortMemoryTarget::C),
            MemoryTarget::Short(ShortMemoryTarget::ADDR_DE),
        ));

        // Shouldn't change
        assert_eq!(cpu.bus.read_byte(ADDR), VAL);
        assert_eq!(cpu.registers.get_de(), ADDR);
        assert_flags_arent_changed(&cpu);

        assert_eq!(cpu.registers.c, VAL);
    }

    #[test]
    fn test_ld_addr_de_addr_bc() {
        const VAL: u8 = 0x3f;
        const ADDR1: u16 = 0x0123;
        const ADDR2: u16 = 0x7777;

        let mut cpu: CPU = Default::default();
        cpu.bus.write_byte(ADDR1, VAL);
        cpu.registers.set_bc(ADDR1);
        cpu.registers.set_de(ADDR2);

        cpu.execute(Instruction::LD(
            MemoryTarget::Short(ShortMemoryTarget::ADDR_DE),
            MemoryTarget::Short(ShortMemoryTarget::ADDR_BC),
        ));

        // Shouldn't change
        assert_eq!(cpu.bus.read_byte(ADDR1), VAL);
        assert_eq!(cpu.registers.get_bc(), ADDR1);
        assert_eq!(cpu.registers.get_de(), ADDR2);
        assert_flags_arent_changed(&cpu);

        assert_eq!(cpu.bus.read_byte(ADDR2), VAL);
    }

    #[test]
    fn test_ld_bc_de() {
        const VAL: u16 = 0x0123;

        let mut cpu: CPU = Default::default();
        cpu.registers.set_de(VAL);

        cpu.execute(Instruction::LD(
            MemoryTarget::Long(LongMemoryTarget::BC),
            MemoryTarget::Long(LongMemoryTarget::DE),
        ));

        assert_eq!(cpu.registers.get_de(), VAL);
        assert_flags_arent_changed(&cpu);

        assert_eq!(cpu.registers.get_bc(), VAL);
    }

    #[test]
    fn test_ld_bc_addr_hl() {
        const VAL: u16 = 0xf00d;
        const ADDR: u16 = 0x0123;

        let mut cpu: CPU = Default::default();
        cpu.bus.write_u16(ADDR, VAL);
        cpu.registers.set_hl(ADDR);

        cpu.execute(Instruction::LD(
            MemoryTarget::Long(LongMemoryTarget::BC),
            MemoryTarget::Long(LongMemoryTarget::ADDR_HL),
        ));

        // Shouldn't change
        assert_eq!(cpu.bus.read_u16(ADDR), VAL);
        assert_eq!(cpu.registers.get_hl(), ADDR);
        assert_flags_arent_changed(&cpu);

        assert_eq!(cpu.registers.get_bc(), VAL);
    }

    #[test]
    fn test_jp_no_condition() {
        const ADDR: u16 = 0x0123;

        let mut cpu: CPU = Default::default();

        cpu.execute(Instruction::JP(None, LongMemoryTarget::CONSTANT(ADDR)));

        assert_eq!(cpu.pc, ADDR);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_jp_with_condition() {
        let mut addr: u16 = 0x0123;

        let mut cpu: CPU = Default::default();

        for flag in ControlFlowFlag::iter() {
            cpu.set_control_flow_flag_value(&flag);
            cpu.execute(Instruction::JP(
                Some(flag.clone()),
                LongMemoryTarget::CONSTANT(addr),
            ));

            assert_eq!(cpu.pc, addr);
            assert!(cpu.get_control_flow_flag_value(&flag));

            addr += 1;
        }
    }

    #[test]
    fn test_jr_no_condition() {
        const ADDR: u16 = 0x0123;
        const DIFF: i8 = -89;

        let mut cpu = CPU {
            pc: ADDR,
            ..Default::default()
        };

        cpu.execute(Instruction::JR(
            None,
            ShortMemoryTarget::CONSTANT(DIFF as u8),
        ));

        assert_eq!(cpu.pc, (ADDR as i16 + DIFF as i16) as u16);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_jr_with_condition() {
        const ADDR: u16 = 0x0123;
        let mut diff: i8 = -89;

        let mut cpu: CPU = Default::default();

        for flag in ControlFlowFlag::iter() {
            cpu.pc = ADDR;

            cpu.set_control_flow_flag_value(&flag);
            cpu.execute(Instruction::JR(
                Some(flag.clone()),
                ShortMemoryTarget::CONSTANT(diff as u8),
            ));

            assert_eq!(cpu.pc, (ADDR as i16 + diff as i16) as u16);
            assert!(cpu.get_control_flow_flag_value(&flag));

            diff += 1;
        }
    }

    #[test]
    fn test_daa_add_b() {
        const A: u8 = 0x45;
        const B: u8 = 0x38;
        const BCD_ADDITION: u8 = 0x83;

        let mut cpu: CPU = Default::default();

        cpu.registers.a = A;
        cpu.registers.b = B;

        cpu.execute(Instruction::ADD(ShortArithmeticTarget::B));
        assert_eq!(cpu.registers.a, A.wrapping_add(B));

        cpu.execute(Instruction::DAA);
        assert_eq!(cpu.registers.a, BCD_ADDITION);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);

        // Should always get cleared
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_daa_sub_c() {
        const A: u8 = 0x83;
        const C: u8 = 0x38;
        const BCD_SUBTRACTION: u8 = 0x45;

        let mut cpu: CPU = Default::default();

        cpu.registers.a = A;
        cpu.registers.c = C;

        cpu.execute(Instruction::SUB(ShortArithmeticTarget::C));
        assert_eq!(cpu.registers.a, A.wrapping_sub(C));

        cpu.execute(Instruction::DAA);
        assert_eq!(cpu.registers.a, BCD_SUBTRACTION);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);

        // Should always get cleared
        assert!(!cpu.registers.f.half_carry);
    }
}
