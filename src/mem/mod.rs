mod mem_bus;
mod mem_trait;

#[cfg(test)]
mod mem_flat;

#[cfg(test)]
pub use mem_flat::MemoryFlat;

pub use mem_bus::MemoryBus;
pub use mem_trait::Memory;
