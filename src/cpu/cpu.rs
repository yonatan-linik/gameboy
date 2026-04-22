#![allow(clippy::upper_case_acronyms)]

use crate::cpu::instruction::*;
use crate::cpu::register::Registers;
use crate::mem::Memory;

#[derive(Debug, Clone, PartialEq)]
pub struct CPU<M: Memory> {
    registers: Registers,
    pc: u16,
    bus: M,
    next_instruction: Instruction,
}

impl<M: Memory> Default for CPU<M> {
    fn default() -> Self {
        Self {
            registers: Default::default(),
            pc: Default::default(),
            bus: Default::default(),
            next_instruction: Instruction::NOP,
        }
    }
}

impl<M: Memory> CPU<M> {
    pub fn prefetch(&mut self) {
        let mut instruction_byte = self.bus.read_byte(self.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.bus.read_byte(self.pc + 1);
        }
        let end_of_instruction = self.pc + 1 + (prefixed as u16);
        let next_byte = self.bus.read_byte(end_of_instruction);
        let next_2_bytes = self.bus.read_u16(end_of_instruction);

        let Some(instruction) =
            Instruction::from_byte(instruction_byte, prefixed, next_byte, next_2_bytes)
        else {
            panic!(
                "Unkown instruction found: 0x{}{instruction_byte:x}",
                if prefixed { "cb" } else { "" },
            )
        };

        self.pc = end_of_instruction + instruction.extra_bytes();
        self.next_instruction = instruction;
    }

    pub fn step(&mut self) {
        self.execute(self.next_instruction.clone());
    }

    // Test helper methods
    #[cfg(test)]
    pub fn new() -> Self {
        CPU {
            pc: 0,
            ..Default::default()
        }
    }

    #[cfg(test)]
    pub fn set_registers(&mut self, registers: Registers) {
        self.registers = registers;
    }

    #[cfg(test)]
    pub fn get_registers(&self) -> Registers {
        self.registers
    }

    #[cfg(test)]
    pub fn set_pc(&mut self, pc: u16) {
        self.pc = pc;
    }

    #[cfg(test)]
    pub fn get_pc(&self) -> u16 {
        self.pc
    }

    #[cfg(test)]
    pub fn write_byte(&mut self, address: u16, value: u8) {
        self.bus.write_byte(address, value);
    }

    #[cfg(test)]
    pub fn read_byte(&self, address: u16) -> u8 {
        self.bus.read_byte(address)
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
            ShortMemoryTarget::ADDR_HL
            | ShortMemoryTarget::ADDR_HLI
            | ShortMemoryTarget::ADDR_HLD => self.bus.read_byte(self.registers.get_hl()),
            ShortMemoryTarget::ADDR_BC => self.bus.read_byte(self.registers.get_bc()),
            ShortMemoryTarget::ADDR_DE => self.bus.read_byte(self.registers.get_de()),
            ShortMemoryTarget::ADDR_C => self.bus.read_byte(ADDR_PREFIX + self.registers.c as u16),
            ShortMemoryTarget::CONSTANT(val) => *val,
            ShortMemoryTarget::ADDR_PLUS_CONSTANT(val) => {
                self.bus.read_byte(ADDR_PREFIX.wrapping_add(*val as u16))
            }
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
            ShortMemoryTarget::ADDR_HL
            | ShortMemoryTarget::ADDR_HLI
            | ShortMemoryTarget::ADDR_HLD => self.bus.write_byte(self.registers.get_hl(), value),
            ShortMemoryTarget::ADDR_BC => self.bus.write_byte(self.registers.get_bc(), value),
            ShortMemoryTarget::ADDR_DE => self.bus.write_byte(self.registers.get_de(), value),
            ShortMemoryTarget::ADDR_C => self
                .bus
                .write_byte(ADDR_PREFIX + self.registers.c as u16, value),
            ShortMemoryTarget::CONSTANT(_) => panic!("Should never set value of constant"),
            ShortMemoryTarget::ADDR_PLUS_CONSTANT(val) => self
                .bus
                .write_byte(ADDR_PREFIX.wrapping_add(*val as u16), value),
            ShortMemoryTarget::ADDR_CONSTANT(addr) => self.bus.write_byte(*addr, value),
        };
    }

    fn get_long_memory_target_value(&mut self, target: &LongMemoryTarget) -> u16 {
        match target {
            LongMemoryTarget::AF => self.registers.get_af(),
            LongMemoryTarget::BC => self.registers.get_bc(),
            LongMemoryTarget::DE => self.registers.get_de(),
            LongMemoryTarget::HL => self.registers.get_hl(),
            LongMemoryTarget::SP => self.registers.get_sp(),
            LongMemoryTarget::SP_PLUS(val) => self.addsp(*val),
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

    fn apply_short_arithmetic_target<F>(&mut self, target: &ShortArithmeticTarget, f: F)
    where
        F: FnOnce(&mut Self, u8) -> u8,
    {
        let value = self.get_short_arithmetic_target_value(target);
        f(self, value);
    }

    fn apply_set_short_arithmetic_target<F>(&mut self, target: &ShortArithmeticTarget, f: F)
    where
        F: FnOnce(&mut Self, u8) -> u8,
    {
        let value = self.get_short_arithmetic_target_value(target);
        let result = f(self, value);
        self.set_short_arithmetic_target_value(target, result);
    }

    fn set_a_with_short_arithmetic_target<F>(&mut self, target: &ShortArithmeticTarget, f: F)
    where
        F: FnOnce(&mut Self, u8) -> u8,
    {
        let value = self.get_short_arithmetic_target_value(target);
        self.registers.a = f(self, value);
    }

    fn set_a_reset_zero<F>(&mut self, f: F)
    where
        F: FnOnce(&mut Self, u8) -> u8,
    {
        self.registers.a = f(self, self.registers.a);
        self.registers.f.zero = false;
    }

    /// Returns true if jumped and false otherwise
    fn execute(&mut self, instruction: Instruction) {
        match instruction {
            Instruction::ADD(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::add);
            }
            Instruction::ADC(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::adc);
            }
            Instruction::SUB(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::sub);
            }
            Instruction::SBC(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::sbc);
            }
            Instruction::AND(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::and);
            }
            Instruction::OR(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::or);
            }
            Instruction::XOR(target) => {
                self.set_a_with_short_arithmetic_target(&target, Self::xor);
            }
            Instruction::CP(target) => {
                // Just like sub without updating the A register
                self.apply_short_arithmetic_target(&target, Self::sub);
            }
            Instruction::INC(target) => match target {
                ArithmeticTarget::Short(s) => {
                    self.apply_set_short_arithmetic_target(&s, Self::inc);
                }
                ArithmeticTarget::Long(l) => {
                    let value = self.get_long_arithmetic_target_value(&l);
                    self.set_long_arithmetic_target_value(&l, value.wrapping_add(1));
                }
            },
            Instruction::DEC(target) => match target {
                ArithmeticTarget::Short(s) => {
                    self.apply_set_short_arithmetic_target(&s, Self::dec);
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
            // On "A" variant of rotate operations zero is always false.
            Instruction::RRA => {
                self.set_a_reset_zero(Self::rr);
            }
            Instruction::RRCA => {
                self.set_a_reset_zero(Self::rrc);
            }
            Instruction::RLA => {
                self.set_a_reset_zero(Self::rl);
            }
            Instruction::RLCA => {
                self.set_a_reset_zero(Self::rlc);
            }
            Instruction::RR(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::rr);
            }
            Instruction::RRC(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::rrc);
            }
            Instruction::RL(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::rl);
            }
            Instruction::RLC(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::rlc);
            }
            Instruction::SLA(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::sla);
            }
            Instruction::SRA(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::sra);
            }
            Instruction::SRL(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::srl);
            }
            Instruction::SWAP(target) => {
                self.apply_set_short_arithmetic_target(&target, Self::swap);
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
                let res = self.get_long_memory_target_value(&target);
                self.push(res);
            }
            Instruction::POP(target) => {
                let value = self.pop();
                self.set_long_memory_target_value(&target, value);
            }
            Instruction::LD(dest, src) => self.ld(dest, src),
            Instruction::JP(flag_option, target) => {
                if flag_option
                    .map(|f| self.get_control_flow_flag_value(&f))
                    .unwrap_or(true)
                {
                    self.pc = self.get_long_memory_target_value(&target);
                }
            }
            Instruction::JR(flag_option, target) => {
                if flag_option
                    .map(|f| self.get_control_flow_flag_value(&f))
                    .unwrap_or(true)
                {
                    self.pc = (self.pc as i16)
                        .wrapping_add(self.get_short_memory_target_value(&target) as i8 as i16)
                        as u16;
                }
            }
            Instruction::DAA => {
                self.daa();
            }
            Instruction::RST(n) => self.rst(n),
            Instruction::RET(control_flow_flag) => {
                if control_flow_flag
                    .map(|f| self.get_control_flow_flag_value(&f))
                    .unwrap_or(true)
                {
                    self.ret();
                }
            }
            // TODO: Missing enable interrupts
            Instruction::RETI => self.ret(),
            Instruction::CALL(control_flow_flag, a) => {
                if control_flow_flag
                    .map(|f| self.get_control_flow_flag_value(&f))
                    .unwrap_or(true)
                {
                    self.call(a);
                }
            }
            Instruction::ADDSP(n) => {
                let res = self.addsp(n);
                self.registers.set_sp(res);
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

        self.registers.f.half_carry = value & 0xF == 0xF;

        // No carry on inc operation

        result
    }

    fn dec(&mut self, value: u8) -> u8 {
        let result = value.wrapping_sub(1);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = true;

        // Only when bits 0-3 are all 0 will decrementing the value cause a half carry
        // If it happens all 4 bits will be 1
        self.registers.f.half_carry =
            (value & 0b0000_1111 == 0b0000_0000) && (result & 0b0000_1111 == 0b0000_1111);
        // No carry on dec operation

        result
    }

    fn addhl(&mut self, value: u16) -> u16 {
        let (result, did_overflow) = self.registers.get_hl().overflowing_add(value);

        // Zero flag isn't affected by addhl
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.get_hl() & 0xFFF) + (value & 0xFFF) > 0xFFF;

        result
    }

    fn addsp(&mut self, value: i8) -> u16 {
        let sp = self.registers.get_sp() as i16;
        let result = sp.wrapping_add(value as i16);

        self.registers.f.zero = false;
        self.registers.f.subtract = false;
        self.registers.f.carry = (self.registers.get_sp() & 0xFF) + (value as u16 & 0xFF) > 0xFF;
        self.registers.f.half_carry = (self.registers.get_sp() & 0xF) + (value as u16 & 0xF) > 0xF;

        result as u16
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

    fn ld_hli_or_hld(&mut self, target: &ShortMemoryTarget) {
        match target {
            ShortMemoryTarget::ADDR_HLI => self
                .registers
                .set_hl(self.registers.get_hl().wrapping_add(1)),
            ShortMemoryTarget::ADDR_HLD => self
                .registers
                .set_hl(self.registers.get_hl().wrapping_sub(1)),
            _ => (),
        }
    }

    fn ld(&mut self, dest: MemoryTarget, src: MemoryTarget) {
        match (dest, src) {
            (MemoryTarget::Short(s_dest), MemoryTarget::Short(s_src)) => {
                let value = self.get_short_memory_target_value(&s_src);
                self.set_short_memory_target_value(&s_dest, value);

                // Increase or decrease HL if we need to
                self.ld_hli_or_hld(&s_src);
                self.ld_hli_or_hld(&s_dest);
            }
            (MemoryTarget::Long(l_dest), MemoryTarget::Long(l_src)) => {
                println!("load from {l_src:?} to {l_dest:?}");
                let value = self.get_long_memory_target_value(&l_src);
                println!("value from src is {value}");
                println!("State before is {:?}", self.registers);
                self.set_long_memory_target_value(&l_dest, value);
                println!("State after is {:?}", self.registers);
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

    fn call(&mut self, a: u16) {
        self.push(self.pc);
        self.pc = a;
    }

    fn ret(&mut self) {
        self.pc = self.pop();
    }

    fn rst(&mut self, n: u8) {
        self.push(self.pc);
        self.pc = n as u16;
    }
}

#[cfg(test)]
mod cpu_tests {
    use super::*;
    use crate::mem::MemoryBus;

    type TCPU = CPU<MemoryBus>;

    fn assert_flags_arent_changed(cpu: &TCPU) {
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_add_c() {
        let mut cpu: TCPU = Default::default();
        cpu.registers.c = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADD(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, (7 + 6) as u8);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_add_c_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.l = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::OR(ShortArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 | 0x0f);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_or_l_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.b = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::XOR(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 ^ 0x0f);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_xor_b_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.d = 7;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, 7 + 1);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_inc_d_half_carry() {
        let mut cpu: TCPU = Default::default();
        cpu.registers.d = 15;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, 15 + 1);

        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d_carry() {
        let mut cpu: TCPU = Default::default();
        cpu.registers.d = u8::MAX;
        cpu.execute(Instruction::INC(ArithmeticTarget::Short(
            ShortArithmeticTarget::D,
        )));

        assert_eq!(cpu.registers.d, u8::MAX.wrapping_add(1));

        // Shouldn't indicate carry on "inc" operation
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.e = 0;
        cpu.execute(Instruction::DEC(ArithmeticTarget::Short(
            ShortArithmeticTarget::E,
        )));

        assert_eq!(cpu.registers.e, 0_u8.wrapping_sub(1));

        // Shouldn't indicate carry on "dec" operation
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_addhl_bc() {
        let mut cpu: TCPU = Default::default();
        cpu.registers.set_bc(8942);
        cpu.registers.set_hl(10000);
        cpu.execute(Instruction::ADDHL(LongArithmeticTarget::BC));

        assert_eq!(cpu.registers.get_bc(), 8942);
        assert_eq!(cpu.registers.get_hl(), 10000 + 8942);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_addhl_bc_half_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.set_bc(1);
        cpu.registers.set_hl(u16::MAX);
        cpu.execute(Instruction::ADDHL(LongArithmeticTarget::BC));

        assert_eq!(cpu.registers.get_bc(), 1);
        assert_eq!(cpu.registers.get_hl(), u16::MAX.wrapping_add(1));

        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        // Zero flag remains unchanged
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_de() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.a = 0x73;
        cpu.execute(Instruction::SWAP(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x37);

        // Everything should be false except for zero which depends on the value
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_swap_a_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RRA);

        assert_eq!(cpu.registers.a, 0b11010111);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rra_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RRCA);

        assert_eq!(cpu.registers.a, 0b01010111);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rrca_change_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b01110101;

        cpu.execute(Instruction::RLA);

        assert_eq!(cpu.registers.a, 0b11101011);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rla_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b01110101;

        cpu.execute(Instruction::RLCA);

        assert_eq!(cpu.registers.a, 0b11101010);

        // Only "calculate" carry flag which should be false
        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rlca_change_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.a = 0b10101110;

        cpu.execute(Instruction::RR(ShortArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0b11010111);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rr_a_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.b = 0b10101110;

        cpu.execute(Instruction::RRC(ShortArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0b01010111);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rrc_b_change_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.c = 0b01110101;

        cpu.execute(Instruction::RL(ShortArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0b11101011);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rl_c_zero() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
        cpu.registers.f.carry = true;
        cpu.registers.d = 0b01110101;

        cpu.execute(Instruction::RLC(ShortArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 0b11101010);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_rlc_d_change_carry() {
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        let mut cpu: TCPU = Default::default();
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
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();
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
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();
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
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();
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
        const ADDR1: u16 = 0xC123;
        const ADDR2: u16 = 0xD777;

        let mut cpu: TCPU = Default::default();
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
    fn test_ld_addr_hli_a() {
        const VAL: u8 = 0xf0;
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();
        cpu.registers.a = VAL;
        cpu.registers.set_hl(ADDR);
        cpu.bus.write_byte(ADDR, !VAL);

        cpu.execute(Instruction::LD(
            MemoryTarget::Short(ShortMemoryTarget::ADDR_HLI),
            MemoryTarget::Short(ShortMemoryTarget::A),
        ));

        // Shouldn't change
        assert_flags_arent_changed(&cpu);

        assert_eq!(cpu.registers.get_hl(), ADDR.wrapping_add(1));
        assert_eq!(cpu.bus.read_byte(ADDR), VAL);
    }

    #[test]
    fn test_ld_a_addr_hld() {
        const VAL: u8 = 0xf0;
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();
        cpu.registers.a = !VAL;
        cpu.registers.set_hl(ADDR);
        cpu.bus.write_byte(ADDR, VAL);

        cpu.execute(Instruction::LD(
            MemoryTarget::Short(ShortMemoryTarget::A),
            MemoryTarget::Short(ShortMemoryTarget::ADDR_HLD),
        ));

        // Shouldn't change
        assert_flags_arent_changed(&cpu);
        assert_eq!(cpu.bus.read_byte(ADDR), VAL);

        assert_eq!(cpu.registers.get_hl(), ADDR.wrapping_sub(1));
        assert_eq!(cpu.registers.a, VAL);
    }

    #[test]
    fn test_ld_a_addr_plus_const() {
        const VAL: u8 = 0xf0;
        const ADD_TO_ADDR: u8 = 0x83;

        let mut cpu: TCPU = Default::default();
        cpu.registers.a = !VAL;
        cpu.bus.write_byte(ADDR_PREFIX + ADD_TO_ADDR as u16, VAL);

        cpu.execute(Instruction::LD(
            MemoryTarget::Short(ShortMemoryTarget::A),
            MemoryTarget::Short(ShortMemoryTarget::ADDR_PLUS_CONSTANT(ADD_TO_ADDR)),
        ));

        // Shouldn't change
        assert_flags_arent_changed(&cpu);
        assert_eq!(cpu.bus.read_byte(ADDR_PREFIX + ADD_TO_ADDR as u16), VAL);

        assert_eq!(cpu.registers.a, VAL);
    }

    #[test]
    fn test_ld_bc_de() {
        const VAL: u16 = 0x0123;

        let mut cpu: TCPU = Default::default();
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
    fn test_jp_no_condition() {
        const ADDR: u16 = 0xC123;

        let mut cpu: TCPU = Default::default();

        cpu.execute(Instruction::JP(None, LongMemoryTarget::CONSTANT(ADDR)));

        assert_eq!(cpu.pc, ADDR);

        assert_flags_arent_changed(&cpu);
    }

    #[test]
    fn test_jp_with_condition() {
        let mut addr: u16 = 0x0123;

        let mut cpu: TCPU = Default::default();

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
        const ADDR: u16 = 0xC123;
        const DIFF: i8 = -89;

        let mut cpu = TCPU {
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
        const ADDR: u16 = 0xC123;
        let mut diff: i8 = -89;

        let mut cpu: TCPU = Default::default();

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

        let mut cpu: TCPU = Default::default();

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

        let mut cpu: TCPU = Default::default();

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
