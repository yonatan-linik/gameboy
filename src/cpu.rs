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
    ADDHL(ArithmeticTarget),
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

struct CPU {
    registers: Registers,
}

impl CPU {
    fn get_arithmetic_target_value(&self, target: ArithmeticTarget) -> u8 {
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

    fn execute(&mut self, instruction: Instruction) {
        match instruction {
            Instruction::ADD(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.add(value);
            }
            Instruction::ADC(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.adc(value);
            }
            Instruction::SUB(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.sub(value);
            }
            Instruction::SBC(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.sbc(value);
            }
            Instruction::AND(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.and(value);
            }
            Instruction::OR(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.or(value);
            }
            Instruction::XOR(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                self.registers.a = self.xor(value);
            }
            Instruction::ADDHL(target) => {
                let value: u8 = self.get_arithmetic_target_value(target);
                let result = self.addhl(value);
                self.registers.set_hl(result);
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

    fn addhl(&mut self, value: u8) -> u16 {
        let (result, did_overflow) = self.registers.get_hl().overflowing_add(value as u16);

        self.registers.f.zero = result == 0;
        self.registers.f.subtract = false;
        self.registers.f.carry = did_overflow;
        self.registers.f.half_carry = (self.registers.a & 0xF) + (value & 0xF) > 0xF;

        result
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
        cpu.registers.a = 255;
        cpu.execute(Instruction::ADD(ArithmeticTarget::C));

        assert_eq!(cpu.registers.c, 7);
        assert_eq!(cpu.registers.a, (7 + 255) as u8);
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
        cpu.registers.a = 255;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 7);
        assert_eq!(cpu.registers.a, (7 + 255 + 1) as u8);
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
        cpu.registers.b = 255;
        cpu.registers.a = 255;
        cpu.execute(Instruction::ADC(ArithmeticTarget::B));

        assert_eq!(cpu.registers.b, 255);
        assert_eq!(cpu.registers.a, (255 + 255 + 1) as u8);
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
        cpu.registers.d = 255;
        cpu.registers.a = 7;
        cpu.execute(Instruction::SUB(ArithmeticTarget::D));

        assert_eq!(cpu.registers.d, 255);
        assert_eq!(cpu.registers.a, 7_u8.wrapping_sub(255) as u8);
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
        assert_eq!(cpu.registers.a, 1_u8.wrapping_sub(15) as u8);

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
        cpu.registers.e = 255;
        cpu.registers.a = 0;
        cpu.execute(Instruction::SBC(ArithmeticTarget::E));

        assert_eq!(cpu.registers.e, 255);
        assert_eq!(cpu.registers.a, 0_u8.wrapping_sub(255).wrapping_sub(1));
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
}
