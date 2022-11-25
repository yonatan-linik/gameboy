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
        (if flag.zero { 1 } else { 0 }) << ZERO_FLAG_BYTE_POSITION
            | (if flag.subtract { 1 } else { 0 }) << SUBTRACT_FLAG_BYTE_POSITION
            | (if flag.half_carry { 1 } else { 0 }) << HALF_CARRY_FLAG_BYTE_POSITION
            | (if flag.carry { 1 } else { 0 }) << CARRY_FLAG_BYTE_POSITION
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
}

enum LongArithmeticTarget {
    BC,
    DE,
    HL,
}

enum ArithmeticTarget {
    Short(ShortArithmeticTarget),
    Long(LongArithmeticTarget),
}

#[derive(Debug, Default, Clone, PartialEq)]
struct CPU {
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
}

impl CPU {}

impl Instruction {
    fn from_byte(byte: u8, prefixed: bool) -> Option<Instruction> {
        if prefixed {
            Instruction::from_byte_prefixed(byte)
        } else {
            Instruction::from_byte_not_prefixed(byte)
        }
    }

    fn short_target_from_byte(byte: u8) -> Option<ShortArithmeticTarget> {
        ShortArithmeticTarget::iter().nth((byte & 0x07) as usize)
    }

    /* Returns the bit for the opcode from the byte given.
     * This works for BIT, RESET, SET opcodes
     */
    fn get_bit_opcode_arg_from_prefixed_byte(byte: u8) -> u8 {
        (byte & 0x38) >> 3
    }

    fn from_byte_prefixed(byte: u8) -> Option<Instruction> {
        /* last 3 bits just tell us the register to use */
        match byte & 0xf8 {
            0x00 => Some(Instruction::RLC(Instruction::short_target_from_byte(byte)?)),
            0x08 => Some(Instruction::RRC(Instruction::short_target_from_byte(byte)?)),
            0x10 => Some(Instruction::RL(Instruction::short_target_from_byte(byte)?)),
            0x18 => Some(Instruction::RR(Instruction::short_target_from_byte(byte)?)),
            0x20 => Some(Instruction::SLA(Instruction::short_target_from_byte(byte)?)),
            0x28 => Some(Instruction::SRA(Instruction::short_target_from_byte(byte)?)),
            0x30 => Some(Instruction::SWAP(Instruction::short_target_from_byte(
                byte,
            )?)),
            0x38 => Some(Instruction::SRL(Instruction::short_target_from_byte(byte)?)),
            (0x40..=0x78) => Some(Instruction::BIT(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_target_from_byte(byte)?,
            )),
            (0x80..=0xB8) => Some(Instruction::RESET(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_target_from_byte(byte)?,
            )),
            (0xC0..=0xF8) => Some(Instruction::SET(
                Instruction::get_bit_opcode_arg_from_prefixed_byte(byte),
                Instruction::short_target_from_byte(byte)?,
            )),
            _ => None, // Shouldn't get here ever as we covered all values which are possible after the and operation
        }
    }

    fn from_byte_not_prefixed(byte: u8) -> Option<Instruction> {
        match byte {
            0x00 => Some(Instruction::NOP),
            0x01 => None, // LD BC, d16 (constant I guess?)
            0x02 => None, // LD (BC), A
            0x03 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::BC,
            ))),
            0x04 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::B,
            ))),
            0x05 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::B,
            ))),
            0x06 => None, // LD B, d8 (constant I guess?)
            0x07 => Some(Instruction::RLCA),
            0x08 => None, // LD (a16), SP
            0x09 => Some(Instruction::ADDHL(LongArithmeticTarget::BC)),
            0x0A => None, // LD A, (BC)
            0x0B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::BC,
            ))),
            0x0C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::C,
            ))),
            0x0D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::C,
            ))),
            0x0E => None, // LD C, d8 (constant I guess?)
            0x0F => Some(Instruction::RRCA),
            0x10 => None, // STOP 0
            0x11 => None, // LD DE, d16
            0x12 => None, // LD (DE), A
            0x13 => Some(Instruction::INC(ArithmeticTarget::Long(
                LongArithmeticTarget::DE,
            ))),
            0x14 => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::D,
            ))),
            0x15 => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::D,
            ))),
            0x16 => None, // LD D, d8 (constant I guess?)
            0x17 => Some(Instruction::RLA),
            0x18 => None, // JR r8
            0x19 => Some(Instruction::ADDHL(LongArithmeticTarget::DE)),
            0x1A => None, // LD A, (DE)
            0x1B => Some(Instruction::DEC(ArithmeticTarget::Long(
                LongArithmeticTarget::DE,
            ))),
            0x1C => Some(Instruction::INC(ArithmeticTarget::Short(
                ShortArithmeticTarget::E,
            ))),
            0x1D => Some(Instruction::DEC(ArithmeticTarget::Short(
                ShortArithmeticTarget::E,
            ))),
            0x1E => None, // LD E, d8 (constant I guess?)
            0x1F => Some(Instruction::RRA),

            0x40..=0x75 | 0x77..=0x7F => None, // LD calculate dest use short_target_from_byte() for src
            0x76 => None,                      // HALT
            0x80..=0x87 => Some(Instruction::ADD(Instruction::short_target_from_byte(byte)?)),
            0x88..=0x8F => Some(Instruction::ADC(Instruction::short_target_from_byte(byte)?)),
            0x90..=0x97 => Some(Instruction::SUB(Instruction::short_target_from_byte(byte)?)),
            0x98..=0x9F => Some(Instruction::SBC(Instruction::short_target_from_byte(byte)?)),
            0xA0..=0xA7 => Some(Instruction::AND(Instruction::short_target_from_byte(byte)?)),
            0xA8..=0xAF => Some(Instruction::XOR(Instruction::short_target_from_byte(byte)?)),
            0xB0..=0xB7 => Some(Instruction::OR(Instruction::short_target_from_byte(byte)?)),
            0xB8..=0xBF => Some(Instruction::CP(Instruction::short_target_from_byte(byte)?)),
            _ =>
            /* TODO: Add mapping for rest of instructions */
            {
                None
            }
        }
    }
}

impl CPU {
    fn step(&mut self) {
        let mut instruction_byte = self.bus.read_byte(self.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.bus.read_byte(self.pc + 1);
        }

        let next_pc = if let Some(instruction) = Instruction::from_byte(instruction_byte, prefixed)
        {
            self.execute(instruction);
            self.pc + 1 + (prefixed as u16)
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
        };
    }

    fn get_long_arithmetic_target_value(&self, target: &LongArithmeticTarget) -> u16 {
        match target {
            LongArithmeticTarget::BC => self.registers.get_bc(),
            LongArithmeticTarget::DE => self.registers.get_de(),
            LongArithmeticTarget::HL => self.registers.get_hl(),
        }
    }

    fn set_long_arithmetic_target_value(&mut self, target: &LongArithmeticTarget, value: u16) {
        match target {
            LongArithmeticTarget::BC => self.registers.set_bc(value),
            LongArithmeticTarget::DE => self.registers.set_de(value),
            LongArithmeticTarget::HL => self.registers.set_hl(value),
        };
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
}

#[cfg(test)]
mod cpu_tests {
    use crate::cpu::*;

    #[test]
    fn test_add_c() {
        let mut cpu: CPU = Default::default();
        cpu.registers.c = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADD(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, (7 + 6) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_or_l_zero() {
        let mut cpu: CPU = Default::default();
        cpu.registers.l = 0x0;
        cpu.registers.a = 0x0;
        cpu.execute(Instruction::OR(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0x0);
        assert_eq!(cpu.registers.a, 0x0 | 0x0);

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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d_half_carry() {
        let mut cpu: CPU = Default::default();
        cpu.registers.d = 15;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, 15 + 1);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        // Shouldn't change on "inc" operation
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_swap_a() {
        let mut cpu: CPU = Default::default();
        cpu.registers.a = 0x73;
        cpu.execute(Instruction::SWAP(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x37);

        // Everything should be false except for zero which depends on the value
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
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

            // Flags are not changed
            assert!(!cpu.registers.f.zero);
            assert!(!cpu.registers.f.carry);
            assert!(!cpu.registers.f.subtract);
            assert!(!cpu.registers.f.half_carry);
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

            // Flags are not changed
            assert!(!cpu.registers.f.zero);
            assert!(!cpu.registers.f.carry);
            assert!(!cpu.registers.f.subtract);
            assert!(!cpu.registers.f.half_carry);
        }
    }
}
