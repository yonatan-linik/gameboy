mod cpu;
mod mem;

fn main() {
    println!("Hello, world!");
    let mut c: cpu::CPU = Default::default();
    c.step();
}
