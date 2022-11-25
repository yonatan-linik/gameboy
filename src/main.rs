mod cpu;

fn main() {
    println!("Hello, world!");
    let mut c: cpu::CPU = Default::default();
    c.step();
}
