#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

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

pub const OAM_RAM_BEGIN: usize = 0xFE00;
pub const OAM_RAM_END: usize = 0xFE9F;

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

const OAM_SIZE: usize = OAM_RAM_END - OAM_RAM_BEGIN + 1;
const SPRITE_ATTRS_SIZE: usize = 4;
const SPRITES_IN_OAM: usize = OAM_SIZE / SPRITE_ATTRS_SIZE;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct SpriteFlags {
    bg_and_win_over_obj: bool,
    y_flip: bool,
    x_flip: bool,
    palette_number: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct SpriteAttributes {
    y_pos: u8,
    x_pos: u8,
    tile_index: u8,
    sprite_flags: SpriteFlags,
}

#[derive(Debug, Clone, PartialEq)]
struct GPU {
    vram: [u8; VRAM_SIZE],
    tile_set: [Tile; TILE_SETS_SIZE],
    oam: [SpriteAttributes; SPRITES_IN_OAM],
    indexing_method: IndexingMethod,
}

impl Default for GPU {
    fn default() -> Self {
        GPU {
            vram: [0; VRAM_SIZE],
            tile_set: [empty_tile(); TILE_SETS_SIZE],
            oam: [SpriteAttributes::default(); SPRITES_IN_OAM],
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

#[derive(Debug, Clone, Copy, PartialEq, Default, EnumIter)]
enum LCDControllerMode {
    #[default]
    H_BLANK = 0,
    V_BLANK,
    SEARCHING_OAM,
    TRANSFER_DATA_TO_LCD_CONTROLLER,
}

impl std::convert::From<LCDControllerMode> for u8 {
    fn from(mode: LCDControllerMode) -> u8 {
        mode as u8
    }
}

impl std::convert::TryFrom<u8> for LCDControllerMode {
    type Error = &'static str;

    fn try_from(mode: u8) -> Result<Self, Self::Error> {
        LCDControllerMode::iter()
            .nth(mode as usize)
            .ok_or("Got invalid mode to convert from")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct StatRegister {
    lyc_eq_ly_interrupt: bool,
    mode_2_oam_interrupt: bool,
    mode_1_vblank_interrupt: bool,
    mode_0_hblank_interrupt: bool,
    lyc_eq_ly: bool,
    lcd_controller_mode: LCDControllerMode,
}

impl std::convert::From<StatRegister> for u8 {
    fn from(reg: StatRegister) -> u8 {
        u8::from(reg.lyc_eq_ly_interrupt) << 7
            | u8::from(reg.mode_2_oam_interrupt) << 6
            | u8::from(reg.mode_1_vblank_interrupt) << 5
            | u8::from(reg.mode_0_hblank_interrupt) << 4
            | u8::from(reg.lyc_eq_ly) << 3
            | u8::from(reg.lcd_controller_mode) << 1
    }
}

impl std::convert::From<u8> for StatRegister {
    fn from(byte: u8) -> Self {
        StatRegister {
            lyc_eq_ly_interrupt: byte & (1_u8 << 7) != 0,
            mode_2_oam_interrupt: byte & (1_u8 << 6) != 0,
            mode_1_vblank_interrupt: byte & (1_u8 << 5) != 0,
            mode_0_hblank_interrupt: byte & (1_u8 << 4) != 0,
            lyc_eq_ly: byte & (1_u8 << 3) != 0,
            lcd_controller_mode: LCDControllerMode::try_from((byte & 0b110) >> 1)
                .expect("Couln't convert bits to LCDCController mode"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default, EnumIter)]
enum Color {
    #[default]
    WHITE = 0,
    LIGHT_GRAY,
    DARK_GRAY,
    BLACK,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct BGPallete {
    index_3: Color,
    index_2: Color,
    index_1: Color,
    index_0: Color,
}

impl std::convert::From<BGPallete> for u8 {
    fn from(reg: BGPallete) -> u8 {
        (reg.index_3 as u8) << 6
            | (reg.index_2 as u8) << 4
            | (reg.index_1 as u8) << 2
            | reg.index_0 as u8
    }
}

impl std::convert::From<u8> for BGPallete {
    fn from(reg: u8) -> Self {
        BGPallete {
            index_3: Color::iter().nth((reg >> 6) as usize & 0b11).unwrap(),
            index_2: Color::iter().nth((reg >> 4) as usize & 0b11).unwrap(),
            index_1: Color::iter().nth((reg >> 2) as usize & 0b11).unwrap(),
            index_0: Color::iter().nth(reg as usize & 0b11).unwrap(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct ObjPallete {
    index_3: Color,
    index_2: Color,
    index_1: Color,
}

impl std::convert::From<ObjPallete> for u8 {
    fn from(reg: ObjPallete) -> u8 {
        (reg.index_3 as u8) << 6 | (reg.index_2 as u8) << 4 | (reg.index_1 as u8) << 2
    }
}

impl std::convert::From<u8> for ObjPallete {
    fn from(reg: u8) -> Self {
        ObjPallete {
            index_3: Color::iter().nth((reg >> 6) as usize & 0b11).unwrap(),
            index_2: Color::iter().nth((reg >> 4) as usize & 0b11).unwrap(),
            index_1: Color::iter().nth((reg >> 2) as usize & 0b11).unwrap(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
struct IORegisters {
    lcdc: LCDCRegister,
    stat: StatRegister,
    scy: u8,
    scx: u8,
    ly: u8,
    lyc: u8,
    oam_dma_transfer: u8,
    bgp: BGPallete,
    obp0: ObjPallete,
    obp1: ObjPallete,
    wy: u8,
    wx: u8,
}

impl IORegisters {
    fn read(&self, address: usize) -> u8 {
        match address {
            0xFF40 => u8::from(self.lcdc),
            0xFF41 => u8::from(self.stat),
            0xFF42 => self.scy,
            0xFF43 => self.scx,
            0xFF44 => self.ly,
            0xFF45 => self.lyc,
            0xFF46 => self.oam_dma_transfer,
            0xFF47 => u8::from(self.bgp),
            0xFF48 => u8::from(self.obp0),
            0xFF49 => u8::from(self.obp1),
            0xFF4A => self.wy,
            0xFF4B => self.wx,
            non_supported_reg => {
                panic!("reading from non supported I/O reg {non_supported_reg:#04x}")
            }
        }
    }

    fn write(&mut self, address: usize, value: u8) {
        match address {
            0xFF40 => self.lcdc = value.into(),
            0xFF41 => self.stat = value.into(),
            0xFF42 => self.scy = value,
            0xFF43 => self.scx = value,
            0xFF44 => (), // LY is read-only
            0xFF45 => self.lyc = value,
            0xFF46 => self.oam_dma_transfer = value, // Start transfer to OAM here
            0xFF47 => self.bgp = value.into(),
            0xFF48 => self.obp0 = value.into(),
            0xFF49 => self.obp1 = value.into(),
            0xFF4A => self.wy = value,
            0xFF4B => self.wx = value,
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
            OAM_RAM_BEGIN..=OAM_RAM_END => 0,
            URAM_BEGIN..=URAM_END => 0,
            IO_REGS_BEGIN..=IO_REGS_END => self.io_regs.read(address),
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
            OAM_RAM_BEGIN..=OAM_RAM_END => (),
            URAM_BEGIN..=URAM_END => (),
            IO_REGS_BEGIN..=IO_REGS_END => self.io_regs.write(address, value),
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

    #[test]
    fn test_stat_reg() {
        let reg = StatRegister {
            lyc_eq_ly_interrupt: true,
            mode_2_oam_interrupt: true,
            mode_1_vblank_interrupt: true,
            mode_0_hblank_interrupt: true,
            lyc_eq_ly: true,
            lcd_controller_mode: LCDControllerMode::TRANSFER_DATA_TO_LCD_CONTROLLER,
        };

        let byte = u8::from(reg);
        assert_eq!(byte, 0b11111110);

        assert_eq!(StatRegister::try_from(byte).unwrap(), reg);

        // LSB is ignored
        let byte: u8 = 0b11111111;
        assert_eq!(StatRegister::try_from(byte).unwrap(), reg);
    }

    #[test]
    fn test_bgp_reg() {
        let reg = BGPallete {
            index_3: Color::BLACK,
            index_2: Color::LIGHT_GRAY,
            index_1: Color::WHITE,
            index_0: Color::DARK_GRAY,
        };

        let byte = u8::from(reg);
        assert_eq!(byte, 0b11010010);

        assert_eq!(BGPallete::from(byte), reg);
    }

    #[test]
    fn test_obp_reg() {
        let reg = ObjPallete {
            index_3: Color::BLACK,
            index_2: Color::LIGHT_GRAY,
            index_1: Color::WHITE,
        };

        let byte = u8::from(reg);
        assert_eq!(byte, 0b11010000);

        assert_eq!(ObjPallete::from(byte), reg);
    }
}
