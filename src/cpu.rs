#[derive(Default, Copy, Clone)]
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

#[derive(Default)]
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
    ADD(ArithmeticTarget),
    ADC(ArithmeticTarget),
    SUB(ArithmeticTarget),
    SBC(ArithmeticTarget),
    AND(ArithmeticTarget),
    OR(ArithmeticTarget),
    XOR(ArithmeticTarget),
    CP(ArithmeticTarget),
    INC(ArithmeticTarget),
    DEC(ArithmeticTarget),
    ADDHL(LongArithmeticTarget),
    INCL(LongArithmeticTarget),
    DECL(LongArithmeticTarget),
    SWAP(ArithmeticTarget),
    CCF,
    SCF,
    NOP,
    CPL,
    RRA,
    RRCA,
    RLA,
    RLCA,
    RR(ArithmeticTarget),
    RRC(ArithmeticTarget),
    RL(ArithmeticTarget),
    RLC(ArithmeticTarget),
    SLA(ArithmeticTarget),
    SRA(ArithmeticTarget),
    SRL(ArithmeticTarget),
    BIT(u8, ArithmeticTarget),
    SET(u8, ArithmeticTarget),
    RESET(u8, ArithmeticTarget),
}

enum ArithmeticTarget {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

enum LongArithmeticTarget {
    BC,
    DE,
    HL,
}

struct CPU {
    registers: Registers,
}

impl CPU {
    fn get_arithmetic_target_value(&self, target: &ArithmeticTarget) -> u8 {
        match target {
            ArithmeticTarget::A => self.registers.a,
            ArithmeticTarget::B => self.registers.b,
            ArithmeticTarget::C => self.registers.c,
            ArithmeticTarget::D => self.registers.d,
            ArithmeticTarget::E => self.registers.e,
            ArithmeticTarget::H => self.registers.h,
            ArithmeticTarget::L => self.registers.l,
        }
    }

    fn set_arithmetic_target_value(&mut self, target: &ArithmeticTarget, value: u8) {
        match target {
            ArithmeticTarget::A => self.registers.a = value,
            ArithmeticTarget::B => self.registers.b = value,
            ArithmeticTarget::C => self.registers.c = value,
            ArithmeticTarget::D => self.registers.d = value,
            ArithmeticTarget::E => self.registers.e = value,
            ArithmeticTarget::H => self.registers.h = value,
            ArithmeticTarget::L => self.registers.l = value,
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
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.add(value);
            }
            Instruction::ADC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.adc(value);
            }
            Instruction::SUB(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.sub(value);
            }
            Instruction::SBC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.sbc(value);
            }
            Instruction::AND(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.and(value);
            }
            Instruction::OR(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.or(value);
            }
            Instruction::XOR(target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.registers.a = self.xor(value);
            }
            Instruction::CP(target) => {
                let value = self.get_arithmetic_target_value(&target);
                // Just like sub without updating the A register
                self.sub(value);
            }
            Instruction::INC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.inc(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::DEC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.dec(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::ADDHL(target) => {
                let value = self.get_long_arithmetic_target_value(&target);
                let result = self.addhl(value);
                self.registers.set_hl(result);
            }
            Instruction::INCL(target) => {
                let value = self.get_long_arithmetic_target_value(&target);
                /* INC for long targets doesn't affect the flags register */
                self.set_long_arithmetic_target_value(&target, value.wrapping_add(1));
            }
            Instruction::DECL(target) => {
                let value = self.get_long_arithmetic_target_value(&target);
                /* DEC for long targets doesn't affect the flags register */
                self.set_long_arithmetic_target_value(&target, value.wrapping_sub(1));
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

                // On "A" variant of rotate operations carry is always zero.
                self.registers.f.carry = false;
            }
            Instruction::RRCA => {
                self.registers.a = self.rrc(self.registers.a);

                // On "A" variant of rotate operations carry is always zero.
                self.registers.f.carry = false;
            }
            Instruction::RLA => {
                self.registers.a = self.rl(self.registers.a);

                // On "A" variant of rotate operations carry is always zero.
                self.registers.f.carry = false;
            }
            Instruction::RLCA => {
                self.registers.a = self.rlc(self.registers.a);

                // On "A" variant of rotate operations carry is always zero.
                self.registers.f.carry = false;
            }
            Instruction::RR(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.rr(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::RRC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.rrc(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::RL(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.rl(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::RLC(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.rlc(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::SLA(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.sla(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::SRA(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.sra(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::SRL(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.srl(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::SWAP(target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.swap(value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::BIT(bit, target) => {
                let value = self.get_arithmetic_target_value(&target);
                self.bit(bit, value);
            }
            Instruction::BIT(bit, target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.set(bit, value);
                self.set_arithmetic_target_value(&target, result);
            }
            Instruction::BIT(bit, target) => {
                let value = self.get_arithmetic_target_value(&target);
                let result = self.reset(bit, value);
                self.set_arithmetic_target_value(&target, result);
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
        let old_bit = value & 0x80;
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
        let mut new_value = self.rlc(value)

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
        let new_value = self.sra(value);

        // Make MSB zero
        new_value &= !0x80;

        // Recalculate the zero bit
        self.registers.f.zero = new_value == 0;

        new_value
    }

    fn swap(&mut self, value: u8) -> u8 {
        let ms_nibble = value & 0xF0;
        let ls_nibble = value & 0x0F;
        let mut new_value = (ls_nibble << 4) | (ms_nibble >> 4);

        self.registers.f.carry = false;
        self.registers.f.zero = new_value == 0;
        self.registers.f.subtract = false;
        self.registers.f.half_carry = false;

        new_value
    }

    fn bit(&mut self, bit: u8, value: u8) {
        self.registers.f.half_carry = true;
        self.registers.f.subtract = false;
        self.registers.f.zero = (value & (1 << bit)) != 0;
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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADD(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, (7 + 6) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_add_c_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 7;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADD(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, u8::MAX.wrapping_add(7));
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_add_c_half_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 2;
        cpu.registers.a = 15;
        cpu.execute(Instruction::ADD(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 2);
        assert_eq!(cpu.registers.a, (2 + 15) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.b = 7;
        cpu.registers.a = 6;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 7);
        assert_eq!(cpu.registers.a, (7 + 6 + 1) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.b = 7;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 7);
        assert_eq!(cpu.registers.a, u8::MAX.wrapping_add(7).wrapping_add(1));
        assert!(cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_adc_b_double_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.b = u8::MAX;
        cpu.registers.a = u8::MAX;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.b = 2;
        cpu.registers.a = 15;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 2);
        assert_eq!(cpu.registers.a, (2 + 15 + 1) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = 6;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SUB(ArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 6);
        assert_eq!(cpu.registers.a, (7 - 6) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = u8::MAX;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SUB(ArithmeticTarget::D));

        assert_eq!(cpu.registers.d, u8::MAX);
        assert_eq!(cpu.registers.a, 7_u8.wrapping_sub(u8::MAX));
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sub_d_half_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = 15;
        cpu.registers.a = 1;
        cpu.execute(Instruction::SUB(ArithmeticTarget::D));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.e = 6;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SBC(ArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 6);
        assert_eq!(cpu.registers.a, (7 - 6 - 1) as u8);
        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.e = 7;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SBC(ArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 7);
        assert_eq!(cpu.registers.a, ((7 - 7) as u8).wrapping_sub(1));
        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_sbc_e_another_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.e = u8::MAX;
        cpu.registers.a = 0;
        cpu.execute(Instruction::SBC(ArithmeticTarget::E));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.f.carry = true;
        cpu.registers.e = 15;
        cpu.registers.a = 2;
        cpu.execute(Instruction::SBC(ArithmeticTarget::E));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.h = 0xf0;
        cpu.registers.a = 0x1f;
        cpu.execute(Instruction::AND(ArithmeticTarget::H));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.h = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::AND(ArithmeticTarget::H));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.l = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::OR(ArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 | 0x0f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_or_l_zero() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.l = 0x0;
        cpu.registers.a = 0x0;
        cpu.execute(Instruction::OR(ArithmeticTarget::L));

        assert_eq!(cpu.registers.l, 0x0);
        assert_eq!(cpu.registers.a, 0x0 | 0x0);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_xor_a() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.a = 0x1f; // Shouldn't matter what value we have here
        cpu.execute(Instruction::XOR(ArithmeticTarget::A));

        assert_eq!(cpu.registers.a, 0x1f ^ 0x1f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_xor_b_non_zero() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.b = 0xf0;
        cpu.registers.a = 0x0f;
        cpu.execute(Instruction::XOR(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 ^ 0x0f);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_xor_b_zero() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.b = 0xf0;
        cpu.registers.a = 0xf0;
        cpu.execute(Instruction::XOR(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 0xf0);
        assert_eq!(cpu.registers.a, 0xf0 ^ 0xf0);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_eq() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 0xf2;
        cpu.registers.a = 0xf2;
        cpu.execute(Instruction::CP(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf2);
        assert_eq!(cpu.registers.a, 0xf2);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_larger() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 0xf3;
        cpu.registers.a = 0xf2;
        cpu.execute(Instruction::CP(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf3);
        assert_eq!(cpu.registers.a, 0xf2);

        assert!(cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(cpu.registers.f.half_carry);
    }

    #[test]
    fn test_cp_c_smaller() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.c = 0xf2;
        cpu.registers.a = 0xf3;
        cpu.execute(Instruction::CP(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 0xf2);
        assert_eq!(cpu.registers.a, 0xf3);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = 7;
        cpu.execute(Instruction::INC(ArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 7 + 1);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d_half_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = 15;
        cpu.execute(Instruction::INC(ArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 15 + 1);

        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        // Shouldn't change on "inc" operation
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_d_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.d = u8::MAX;
        cpu.execute(Instruction::INC(ArithmeticTarget::D));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.e = 7;
        cpu.execute(Instruction::DEC(ArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 7 - 1);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e_zero() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.e = 1;
        cpu.execute(Instruction::DEC(ArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 1 - 1);

        assert!(!cpu.registers.f.carry);
        assert!(cpu.registers.f.subtract);
        assert!(cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_e_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.e = 0;
        cpu.execute(Instruction::DEC(ArithmeticTarget::E));

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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
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
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_de(7);
        cpu.execute(Instruction::INCL(LongArithmeticTarget::DE));

        assert_eq!(cpu.registers.get_de(), 7 + 1);

        // Shouldn't update flags register on long inc
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_de_half_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_de(0xFFF);
        cpu.execute(Instruction::INCL(LongArithmeticTarget::DE));

        assert_eq!(cpu.registers.get_de(), 0xFFF + 1);

        // Shouldn't update flags register on long inc
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_inc_de_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_de(u16::MAX);
        cpu.execute(Instruction::INCL(LongArithmeticTarget::DE));

        assert_eq!(cpu.registers.get_de(), u16::MAX.wrapping_add(1));

        // Shouldn't update flags register on long inc
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_hl() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_hl(7);
        cpu.execute(Instruction::DECL(LongArithmeticTarget::HL));

        assert_eq!(cpu.registers.get_hl(), 7 - 1);

        // Shouldn't update flags register on long dec
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_hl_carry() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_hl(0);
        cpu.execute(Instruction::DECL(LongArithmeticTarget::HL));

        assert_eq!(cpu.registers.get_hl(), 0_u16.wrapping_sub(1));

        // Shouldn't update flags register on long dec
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }

    #[test]
    fn test_dec_hl_zero() {
        let mut cpu = CPU {
            registers: { Default::default() },
        };
        cpu.registers.set_hl(1);
        cpu.execute(Instruction::DECL(LongArithmeticTarget::HL));

        assert_eq!(cpu.registers.get_hl(), 0);

        // Shouldn't update flags register on long dec
        assert!(!cpu.registers.f.carry);
        assert!(!cpu.registers.f.subtract);
        assert!(!cpu.registers.f.zero);
        assert!(!cpu.registers.f.half_carry);
    }
}
