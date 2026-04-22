#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

pub use strum::IntoEnumIterator;
use strum_macros::EnumIter;

#[derive(Clone, PartialEq, EnumIter, Debug)]
pub enum ControlFlowFlag {
    Zero,
    NonZero,
    Carry,
    NonCarry,
}

/* The order here is the same as it is in the opcodes */
#[derive(EnumIter, Debug, Clone, PartialEq)]
pub enum ShortArithmeticTarget {
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

#[derive(Debug, Clone, PartialEq)]
pub enum LongArithmeticTarget {
    BC,
    DE,
    HL,
    SP,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ArithmeticTarget {
    Short(ShortArithmeticTarget),
    Long(LongArithmeticTarget),
}

/* The order here is the same as it is in the opcodes 0x40-0x7F */
#[derive(EnumIter, Debug, Clone, PartialEq)]
pub enum ShortMemoryTarget {
    B,
    C,
    D,
    E,
    H,
    L,
    ADDR_HL,
    A,
    ADDR_HLI, // Increase HL after read
    ADDR_HLD, // Decrease HL after read
    ADDR_BC,
    ADDR_DE,
    ADDR_C, // This is the address 0xFF00 + C
    CONSTANT(u8),
    ADDR_PLUS_CONSTANT(u8), // This is the address 0xFF00 + a8
    ADDR_CONSTANT(u16),
}

pub const ADDR_PREFIX: u16 = 0xFF00;

#[derive(Debug, Clone, PartialEq)]
pub enum LongMemoryTarget {
    AF,
    BC,
    DE,
    HL,
    SP,
    SP_PLUS(i8),
    CONSTANT(u16),
    ADDR_CONSTANT(u16),
}

#[derive(Debug, Clone, PartialEq)]
pub enum MemoryTarget {
    Short(ShortMemoryTarget),
    Long(LongMemoryTarget),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Instruction {
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
    ADDSP(i8),
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
    RST(u8),
    CALL(Option<ControlFlowFlag>, u16),
    RET(Option<ControlFlowFlag>),
    RETI,
}

impl Instruction {
    pub fn from_byte(
        byte: u8,
        prefixed: bool,
        next_byte: u8,
        next_2_bytes: u16,
    ) -> Option<Instruction> {
        let ins = if prefixed {
            Instruction::from_byte_prefixed(byte)
        } else {
            Instruction::from_byte_not_prefixed(byte, next_byte, next_2_bytes)
        };

        #[cfg(test)]
        return ins.or(Some(Instruction::NOP));
        #[cfg(not(test))]
        ins
    }

    fn short_arithmetic_target_from_byte(byte: u8) -> Option<ShortArithmeticTarget> {
        ShortArithmeticTarget::iter().nth((byte & 0x07) as usize)
    }

    fn short_memory_targets_from_byte(
        byte: u8,
    ) -> (Option<ShortMemoryTarget>, Option<ShortMemoryTarget>) {
        (
            ShortMemoryTarget::iter().nth((((byte >> 3) - 8) & 0x07) as usize),
            ShortMemoryTarget::iter().nth((byte & 0x07) as usize),
        )
    }

    /* Returns the bit for the opcode from the byte given.
     * This works for BIT, RESET, SET opcodes
     */
    fn get_bit_opcode_arg_from_prefixed_byte(byte: u8) -> u8 {
        (byte & 0x38) >> 3
    }

    pub fn extra_bytes(&self) -> u16 {
        println!("{self:?}");
        match self {
            Instruction::LD(_, MemoryTarget::Short(ShortMemoryTarget::CONSTANT(_))) => 1,
            Instruction::LD(MemoryTarget::Short(ShortMemoryTarget::CONSTANT(_)), _) => 1,
            Instruction::LD(_, MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(_))) => 2,
            Instruction::LD(_, MemoryTarget::Long(LongMemoryTarget::CONSTANT(_))) => 2,
            Instruction::LD(_, MemoryTarget::Long(LongMemoryTarget::ADDR_CONSTANT(_))) => 2,
            Instruction::LD(MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(_)), _) => 2,
            Instruction::LD(MemoryTarget::Long(LongMemoryTarget::ADDR_CONSTANT(_)), _) => 2,
            Instruction::LD(MemoryTarget::Short(ShortMemoryTarget::ADDR_PLUS_CONSTANT(_)), _) => 1,
            Instruction::LD(_, MemoryTarget::Short(ShortMemoryTarget::ADDR_PLUS_CONSTANT(_))) => 1,
            Instruction::LD(_, MemoryTarget::Long(LongMemoryTarget::SP_PLUS(_))) => 1,
            Instruction::ADD(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::ADDSP(_) => 1,
            Instruction::ADC(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::SUB(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::SBC(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::AND(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::XOR(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::OR(ShortArithmeticTarget::CONSTANT(_)) => 1,
            Instruction::JR(_, ShortMemoryTarget::CONSTANT(_)) => 1,
            Instruction::JP(_, LongMemoryTarget::CONSTANT(_)) => 2,
            Instruction::CALL(_, _) => 2,
            Instruction::CP(ShortArithmeticTarget::CONSTANT(_)) => 1,
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
            0x22 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_HLI),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
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
            0x2A => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_HLI),
            )),
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
            0x32 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_HLD),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
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
            0x3A => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_HLD),
            )),
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
            0xC0 => Some(Instruction::RET(Some(ControlFlowFlag::NonZero))),
            0xC1 => Some(Instruction::POP(LongMemoryTarget::BC)),
            0xC2 => Some(Instruction::JP(
                Some(ControlFlowFlag::NonZero),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xC3 => Some(Instruction::JP(
                None,
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xC4 => Some(Instruction::CALL(
                Some(ControlFlowFlag::NonZero),
                next_2_bytes,
            )),
            0xC5 => Some(Instruction::PUSH(LongMemoryTarget::BC)),
            0xC6 => Some(Instruction::ADD(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xC7 => Some(Instruction::RST(0x00)),
            0xC8 => Some(Instruction::RET(Some(ControlFlowFlag::Zero))),
            0xC9 => Some(Instruction::RET(None)),
            0xCA => Some(Instruction::JP(
                Some(ControlFlowFlag::Zero),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xCB => panic!("Shouldn't get here, should be handled in another function!"), // PREFIX CB
            0xCC => Some(Instruction::CALL(Some(ControlFlowFlag::Zero), next_2_bytes)),
            0xCD => Some(Instruction::CALL(None, next_2_bytes)),
            0xCE => Some(Instruction::ADC(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xCF => Some(Instruction::RST(0x08)),
            0xD0 => Some(Instruction::RET(Some(ControlFlowFlag::NonCarry))),
            0xD1 => Some(Instruction::POP(LongMemoryTarget::DE)),
            0xD2 => Some(Instruction::JP(
                Some(ControlFlowFlag::NonCarry),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xD4 => Some(Instruction::CALL(
                Some(ControlFlowFlag::NonCarry),
                next_2_bytes,
            )),
            0xD5 => Some(Instruction::PUSH(LongMemoryTarget::DE)),
            0xD6 => Some(Instruction::SUB(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xD7 => Some(Instruction::RST(0x10)),
            0xD8 => Some(Instruction::RET(Some(ControlFlowFlag::Carry))),
            0xD9 => Some(Instruction::RETI),
            0xDA => Some(Instruction::JP(
                Some(ControlFlowFlag::Carry),
                LongMemoryTarget::CONSTANT(next_2_bytes),
            )),
            0xDC => Some(Instruction::CALL(
                Some(ControlFlowFlag::Carry),
                next_2_bytes,
            )),
            0xDE => Some(Instruction::SBC(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xDF => Some(Instruction::RST(0x18)),
            0xE0 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_PLUS_CONSTANT(next_byte)),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0xE1 => Some(Instruction::POP(LongMemoryTarget::HL)),
            0xE2 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_C),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0xE5 => Some(Instruction::PUSH(LongMemoryTarget::HL)),
            0xE6 => Some(Instruction::AND(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xE7 => Some(Instruction::RST(0x20)),
            0xE8 => Some(Instruction::ADDSP(next_byte as i8)),
            0xE9 => Some(Instruction::JP(None, LongMemoryTarget::HL)),
            0xEA => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::ADDR_CONSTANT(next_2_bytes)),
                MemoryTarget::Short(ShortMemoryTarget::A),
            )),
            0xEE => Some(Instruction::XOR(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xEF => Some(Instruction::RST(0x28)),
            0xF0 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_PLUS_CONSTANT(next_byte)),
            )),
            0xF1 => Some(Instruction::POP(LongMemoryTarget::AF)),
            0xF2 => Some(Instruction::LD(
                MemoryTarget::Short(ShortMemoryTarget::A),
                MemoryTarget::Short(ShortMemoryTarget::ADDR_C),
            )),
            0xF3 => None, // DI
            0xF5 => Some(Instruction::PUSH(LongMemoryTarget::AF)),
            0xF6 => Some(Instruction::OR(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xF7 => Some(Instruction::RST(0x30)),
            0xF8 => Some(Instruction::LD(
                MemoryTarget::Long(LongMemoryTarget::HL),
                MemoryTarget::Long(LongMemoryTarget::SP_PLUS(next_byte as i8)),
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
            0xFE => Some(Instruction::CP(ShortArithmeticTarget::CONSTANT(next_byte))),
            0xFF => Some(Instruction::RST(0x38)),

            /* mapping for undefined instructions */
            0xD3 | 0xDB | 0xDD | 0xE3 | 0xE4 | 0xEB | 0xEC | 0xED | 0xF4 | 0xFC | 0xFD => None,
        }
    }
}
