mod cpu;
mod mem;

fn main() {
    let mut c: cpu::CPU<mem::MemoryBus> = Default::default();
    c.prefetch();
    c.step();
}
