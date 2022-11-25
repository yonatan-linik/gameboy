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
    fn execute(&mut self, instruction: Instruction) {
        match instruction {
            Instruction::ADD(target) => {
                let value: u8;

                match target {
                    ArithmeticTarget::A => {
                        value = self.registers.a;
                    }
                    ArithmeticTarget::B => {
                        value = self.registers.b;
                    }
                    ArithmeticTarget::C => {
                        value = self.registers.c;
                    }
                    ArithmeticTarget::D => {
                        value = self.registers.d;
                    }
                    ArithmeticTarget::E => {
                        value = self.registers.e;
                    }
                    ArithmeticTarget::H => {
                        value = self.registers.h;
                    }
                    ArithmeticTarget::L => {
                        value = self.registers.l;
                    }
                }

                self.registers.a = self.add(value);
            }
            _ => { /* TODO: support more instructions */ }
        }
    }

    fn add(&mut self, value: u8) -> u8 {
        let (result, did_overflow) = self.registers.a.overflowing_add(value);

        // Handle overflow
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
}
