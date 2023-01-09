#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

pub const VRAM_BEGIN: usize = 0x8000;
pub const VRAM_END: usize = 0x9FFF;
const VRAM_SIZE: usize = VRAM_END - VRAM_BEGIN + 1;

pub const CRAM_BEGIN: usize = 0xA000;
pub const CRAM_END: usize = 0xBFFF;

pub const WRAM_BEGIN: usize = 0xC000;
pub const WRAM_END: usize = 0xDFFF;

pub const ERAM_BEGIN: usize = 0xE000;
pub const ERAM_END: usize = 0xFDFF;

#[cfg(test)]
pub const ERAM_SIZE: usize = ERAM_END - ERAM_BEGIN + 1;

pub const ECHO_WORK_RAM_DIFF: usize = ERAM_BEGIN - WRAM_BEGIN;

pub const URAM_BEGIN: usize = 0xFEA0;
pub const URAM_END: usize = 0xFEFF;

pub const IO_REGS_BEGIN: usize = 0xFF00;
pub const IO_REGS_END: usize = 0xFF7F;

pub const HRAM_BEGIN: usize = 0xFF80;
pub const HRAM_END: usize = 0xFFFE;

#[derive(Debug, Copy, Clone, Default, PartialEq)]
enum TilePixelValue {
    #[default]
    Zero,
    One,
    Two,
    Three,
}

type Tile = [[TilePixelValue; 8]; 8];
fn empty_tile() -> Tile {
    [[TilePixelValue::Zero; 8]; 8]
}

const TILE_SET_1_BEGIN: usize = 0x8000;
const TILE_SET_1_END: usize = 0x8FFF;
const TILE_SET_2_BEGIN: usize = 0x8800;
const TILE_SET_2_END: usize = 0x97FF;

const TILE_SET_2_INDEX_BASE: usize = (0x9000 - TILE_SET_1_BEGIN) / TILE_SIZE;

// 16 bytes every 2 bytes are one line
const TILE_SIZE: usize = 16;
const TILE_SETS_MEM_SIZE: usize = TILE_SET_2_END - TILE_SET_1_BEGIN + 1;
const TILE_SETS_SIZE: usize = TILE_SETS_MEM_SIZE / TILE_SIZE;

#[derive(Debug, Clone, PartialEq, Default)]
enum IndexingMethod {
    #[default]
    METHOD_8000,
    METHOD_8800,
}

#[derive(Debug, Clone, PartialEq)]
struct GPU {
    vram: [u8; VRAM_SIZE],
    tile_set: [Tile; TILE_SETS_SIZE],
    indexing_method: IndexingMethod,
}

impl Default for GPU {
    fn default() -> Self {
        GPU {
            vram: [0; VRAM_SIZE],
            tile_set: [empty_tile(); TILE_SETS_SIZE],
            indexing_method: Default::default(),
        }
    }
}

impl GPU {
    fn read_vram(&self, address: usize) -> u8 {
        self.vram[address]
    }

    fn write_vram(&mut self, index: usize, value: u8) {
        self.vram[index] = value;
        // If our index is greater than 0x1800, we're not writing to the tile set storage
        // so we can just return.
        if index >= TILE_SETS_MEM_SIZE {
            return;
        }

        // Tiles rows are encoded in two bytes with the first byte always
        // on an even address. Bitwise ANDing the address with 0xffe
        // gives us the address of the first byte.
        // For example: `12 & 0xFFFE == 12` and `13 & 0xFFFE == 12`
        let normalized_index = index & 0xFFFE;

        // First we need to get the two bytes that encode the tile row.
        let byte1 = self.vram[normalized_index];
        let byte2 = self.vram[normalized_index + 1];

        // A tiles is 8 rows tall. Since each row is encoded with two bytes a tile
        // is therefore 16 bytes in total.
        let tile_index = index / TILE_SIZE;
        // Every two bytes is a new row
        let row_index = (index % TILE_SIZE) / 2;

        // Now we're going to loop 8 times to get the 8 pixels that make up a given row.
        for pixel_index in 0..8 {
            // To determine a pixel's value we must first find the corresponding bit that encodes
            // that pixels value:
            // 1111_1111
            // 0123 4567
            //
            // As you can see the bit that corresponds to the nth pixel is the bit in the nth
            // position *from the left*. Bits are normally indexed from the right.
            //
            // To find the first pixel (a.k.a pixel 0) we find the left most bit (a.k.a bit 7). For
            // the second pixel (a.k.a pixel 1) we first the second most left bit (a.k.a bit 6) and
            // so on.
            //
            // We then create a mask with a 1 at that position and 0s everywhere else.
            //
            // Bitwise ANDing this mask with our bytes will leave that particular bit with its
            // original value and every other bit with a 0.
            let mask = 1 << (7 - pixel_index);
            let lsb = byte1 & mask;
            let msb = byte2 & mask;

            // If the masked values are not 0 the masked bit must be 1. If they are 0, the masked
            // bit must be 0.
            //
            // Finally we can tell which of the four tile values the pixel is. For example, if the least
            // significant byte's bit is 1 and the most significant byte's bit is also 1, then we
            // have tile value `Three`.
            let value = match (lsb != 0, msb != 0) {
                (true, true) => TilePixelValue::Three,
                (false, true) => TilePixelValue::Two,
                (true, false) => TilePixelValue::One,
                (false, false) => TilePixelValue::Zero,
            };

            self.tile_set[tile_index][row_index][pixel_index] = value;
        }
    }

    fn index_tile_8000(&self, address: u8) -> Tile {
        self.tile_set[address as usize]
    }

    fn index_tile_8800(&self, address: u8) -> Tile {
        self.tile_set[(TILE_SET_2_INDEX_BASE as isize + (address as i8) as isize) as usize]
    }

    fn index_tile(&self, address: u8) -> Tile {
        match self.indexing_method {
            IndexingMethod::METHOD_8000 => self.index_tile_8000(address),
            IndexingMethod::METHOD_8800 => self.index_tile_8800(address),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct LCDCRegister {
    lcd_n_ppu_enable: bool,
    win_tile_map_area: bool,
    win_enable: bool,
    bg_n_win_tile_data_area: bool,
    bg_tile_map_area: bool,
    obj_size: bool,
    obj_enable: bool,
    bg_n_win_enable: bool,
}

impl std::convert::From<LCDCRegister> for u8 {
    fn from(reg: LCDCRegister) -> u8 {
        [
            reg.bg_n_win_enable,
            reg.obj_enable,
            reg.obj_size,
            reg.bg_tile_map_area,
            reg.bg_n_win_tile_data_area,
            reg.win_enable,
            reg.win_tile_map_area,
            reg.lcd_n_ppu_enable,
        ]
        .iter()
        .enumerate()
        .fold(0, |acc, (index, &flag)| acc | (u8::from(flag) << index))
    }
}

impl std::convert::From<u8> for LCDCRegister {
    fn from(byte: u8) -> Self {
        let mut reg = LCDCRegister::default();
        for (index, flag) in [
            &mut reg.bg_n_win_enable,
            &mut reg.obj_enable,
            &mut reg.obj_size,
            &mut reg.bg_tile_map_area,
            &mut reg.bg_n_win_tile_data_area,
            &mut reg.win_enable,
            &mut reg.win_tile_map_area,
            &mut reg.lcd_n_ppu_enable,
        ]
        .iter_mut()
        .enumerate()
        {
            **flag = (byte & (1_u8 << index)) != 0;
        }

        reg
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
struct IORegisters {
    lcdc: LCDCRegister,
}

impl IORegisters {
    fn read(&self, address: usize) -> u8 {
        match address {
            0xFF40 => u8::from(self.lcdc),
            non_supported_reg => {
                panic!("reading from non supported I/O reg {non_supported_reg:#04x}")
            }
        }
    }

    fn write(&mut self, address: usize, value: u8) {
        match address {
            0xFF40 => self.lcdc = value.into(),
            non_supported_reg => {
                panic!("writing to non supported I/O reg {non_supported_reg:#04x}")
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct MemoryBus {
    memory: [u8; 0xFFFF],
    io_regs: IORegisters,
    gpu: GPU,
}

impl Default for MemoryBus {
    fn default() -> Self {
        MemoryBus {
            memory: [0; 0xFFFF],
            io_regs: Default::default(),
            gpu: Default::default(),
        }
    }
}

impl MemoryBus {
    pub fn read_byte(&self, address: u16) -> u8 {
        let address = address as usize;
        match address {
            VRAM_BEGIN..=VRAM_END => {
                // Don't read from VRAM if LCD & PPU are enabled - return undefined data (0xff)
                if self.io_regs.lcdc.lcd_n_ppu_enable {
                    0xff
                } else {
                    self.gpu.read_vram(address - VRAM_BEGIN)
                }
            }
            CRAM_BEGIN..=CRAM_END | WRAM_BEGIN..=WRAM_END | HRAM_BEGIN..=HRAM_END => {
                self.memory[address]
            }
            ERAM_BEGIN..=ERAM_END => self.memory[address - ECHO_WORK_RAM_DIFF],
            URAM_BEGIN..=URAM_END => 0,
            IO_REGS_BEGIN..=IO_REGS_BEGIN => self.io_regs.read(address),
            _ => panic!("TODO: support other areas of memory"),
        }
    }

    pub fn write_byte(&mut self, address: u16, value: u8) {
        let address = address as usize;
        match address {
            VRAM_BEGIN..=VRAM_END => {
                // Write to VRAM only if LCD & PPU are disabled - otherwise ignore write
                if !self.io_regs.lcdc.lcd_n_ppu_enable {
                    self.gpu.write_vram(address - VRAM_BEGIN, value)
                }
            }
            CRAM_BEGIN..=CRAM_END | WRAM_BEGIN..=WRAM_END | HRAM_BEGIN..=HRAM_END => {
                self.memory[address] = value
            }
            ERAM_BEGIN..=ERAM_END => self.memory[address - ECHO_WORK_RAM_DIFF] = value,
            URAM_BEGIN..=URAM_END => (),
            IO_REGS_BEGIN..=IO_REGS_BEGIN => self.io_regs.write(address, value),
            _ => panic!("TODO: support other areas of memory"),
        }
    }

    pub fn read_u16(&self, address: u16) -> u16 {
        let first_byte: u16 = self.read_byte(address).into();
        let second_byte: u16 = self.read_byte(address.wrapping_add(1)).into();

        (first_byte << 8) | second_byte
    }

    pub fn write_u16(&mut self, address: u16, value: u16) {
        self.write_byte(address, (value >> 8) as u8);
        self.write_byte(address.wrapping_add(1), value as u8);
    }
}

#[cfg(test)]
mod mem_bus_tests {
    use crate::mem::*;

    #[test]
    fn test_working_ram() {
        let mut bus: MemoryBus = Default::default();

        for addr in WRAM_BEGIN..=WRAM_END {
            let addr = addr as u16;
            assert_eq!(bus.read_byte(addr), 0);
            bus.write_byte(addr, addr as u8);
            assert_eq!(bus.read_byte(addr), addr as u8);
        }
    }

    #[test]
    fn test_echo_ram() {
        let mut bus: MemoryBus = Default::default();

        for addr in ERAM_BEGIN..=ERAM_END {
            let addr = addr as u16;
            assert_eq!(bus.read_byte(addr), 0);
            bus.write_byte(addr, addr as u8);
            assert_eq!(bus.read_byte(addr), addr as u8);
        }
    }

    #[test]
    fn test_working_and_echo_ram() {
        let mut bus: MemoryBus = Default::default();

        // Echo RAM doesn't cover the same size as working RAM
        for addr in WRAM_BEGIN..=WRAM_BEGIN + ERAM_SIZE - 1 {
            let addr = addr as u16;
            assert_eq!(bus.read_byte(addr), 0);

            // Write to echo RAM read from working RAM
            bus.write_byte(addr + ECHO_WORK_RAM_DIFF as u16, addr as u8);
            assert_eq!(bus.read_byte(addr), addr as u8);

            // Write to working RAM read from echo RAM
            bus.write_byte(addr, 0);
            assert_eq!(bus.read_byte(addr + ECHO_WORK_RAM_DIFF as u16), 0);
        }
    }

    #[test]
    fn test_cartridge_ram() {
        let mut bus: MemoryBus = Default::default();

        for addr in CRAM_BEGIN..=CRAM_END {
            let addr = addr as u16;
            assert_eq!(bus.read_byte(addr), 0);
            bus.write_byte(addr, addr as u8);
            assert_eq!(bus.read_byte(addr), addr as u8);
        }
    }

    #[test]
    fn test_high_ram() {
        let mut bus: MemoryBus = Default::default();

        for addr in HRAM_BEGIN..=HRAM_END {
            let addr = addr as u16;
            assert_eq!(bus.read_byte(addr), 0);
            bus.write_byte(addr, addr as u8);
            assert_eq!(bus.read_byte(addr), addr as u8);
        }
    }

    #[test]
    fn test_lcdc_reg() {
        let reg = LCDCRegister {
            bg_n_win_enable: true,
            obj_size: true,
            bg_n_win_tile_data_area: true,
            win_tile_map_area: true,
            ..Default::default()
        };

        let byte = u8::from(reg);
        assert_eq!(byte, 0b01010101);

        assert_eq!(LCDCRegister::from(byte), reg);
    }
}
