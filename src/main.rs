mod cpu;
mod mem;

fn main() {
    println!("Hello, world!");
    let mut c: cpu::CPU<mem::MemoryBus> = Default::default();
    c.step();
}
