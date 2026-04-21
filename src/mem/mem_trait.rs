pub trait Memory: Default {
    fn read_byte(&self, address: u16) -> u8;
    fn write_byte(&mut self, address: u16, value: u8);

    fn read_u16(&self, address: u16) -> u16 {
        let first_byte: u16 = self.read_byte(address).into();
        let second_byte: u16 = self.read_byte(address.wrapping_add(1)).into();

        (second_byte << 8) | first_byte
    }

    fn write_u16(&mut self, address: u16, value: u16) {
        self.write_byte(address, value as u8);
        self.write_byte(address.wrapping_add(1), (value >> 8) as u8);
    }
}
