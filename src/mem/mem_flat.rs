use super::Memory;

// #[derive(Debug, Clone, PartialEq)]
pub struct MemoryFlat {
    memory: [u8; 0xFFFF],
}

impl Default for MemoryFlat {
    fn default() -> Self {
        Self { memory: [0; _] }
    }
}

impl Memory for MemoryFlat {
    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn write_byte(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }
}
