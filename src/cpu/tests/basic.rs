//! JSON-based CPU instruction tests
//!
//! This module provides infrastructure for running JSON-based test cases that verify
//! CPU instruction execution. The tests are loaded from JSON files in the `cases/` directory.
//!
//! # Test Format
//!
//! Each test case in the JSON file contains:
//! - `name`: A descriptive name for the test
//! - `initial`: The initial CPU state (registers, PC, SP, and RAM contents)
//! - `final`: The expected CPU state after executing one instruction
//! - `cycles`: Array of memory bus operations that occurred during execution
//!
//! # Usage
//!
//! Tests are automatically run via the `#[test]` functions at the bottom of this file.
//! Each JSON file in the `cases/` directory can be tested individually.
//!
//! # Note
//!
//! The test data in these JSON files comes from external test suites and may not always
//! match the expected behavior of this emulator implementation. Some tests may fail due to
//! differences in how instructions are implemented or timing behavior.

use crate::cpu::cpu::CPU;
use crate::cpu::register::{FlagsRegister, Registers};
use serde::{Deserialize, Serialize};
use std::fs;

/// Represents a single test case from the JSON test data
#[derive(Debug, Deserialize, Serialize, Clone)]
struct TestCase {
    /// Descriptive name for this test case
    name: String,
    /// Initial CPU and memory state before instruction execution
    initial: CpuState,
    /// Expected CPU and memory state after instruction execution
    #[serde(rename = "final")]
    final_state: CpuState,
    /// List of memory bus cycles: (address, value, operation_type)
    cycles: Vec<Option<(u16, u8, String)>>,
}

/// Represents the complete state of the CPU and relevant memory
#[derive(Debug, Deserialize, Serialize, Clone)]
struct CpuState {
    /// Accumulator register
    a: u8,
    /// B register
    b: u8,
    /// C register
    c: u8,
    /// D register
    d: u8,
    /// E register
    e: u8,
    /// Flags register (as u8)
    f: u8,
    /// H register
    h: u8,
    /// L register
    l: u8,
    /// Program counter
    pc: u16,
    /// Stack pointer
    sp: u16,
    /// Memory contents: list of (address, value) pairs
    ram: Vec<(u16, u8)>,
}

impl CpuState {
    /// Convert this state into a Registers struct
    fn to_registers(&self) -> Registers {
        Registers {
            a: self.a,
            b: self.b,
            c: self.c,
            d: self.d,
            e: self.e,
            f: FlagsRegister::from(self.f),
            h: self.h,
            l: self.l,
            sp: self.sp,
        }
    }

    /// Apply this state to a CPU instance, setting all registers and memory
    fn apply_to_cpu(&self, cpu: &mut CPU) {
        let registers = self.to_registers();
        cpu.set_registers(registers);
        // In decode-execute-prefetch architecture, PC points to the next instruction
        // The initial state shows PC after the previous instruction's prefetch
        // We need to set PC back by 1 to point at the instruction we want to execute
        cpu.set_pc(self.pc.wrapping_sub(1));

        // Set up RAM state
        for (address, value) in &self.ram {
            cpu.write_byte(*address, *value);
        }
    }

    /// Check if the given CPU matches this expected state
    ///
    /// Returns true if all registers and memory locations match.
    /// Prints detailed error messages for any mismatches.
    fn matches_cpu(&self, cpu: &CPU) -> bool {
        let registers = cpu.get_registers();
        let pc = cpu.get_pc();

        // Check registers
        if self.a != registers.a {
            println!(
                "Register A mismatch: expected {}, got {}",
                self.a, registers.a
            );
            return false;
        }
        if self.b != registers.b {
            println!("b = {registers:?}");
            println!(
                "Register B mismatch: expected {}, got {}",
                self.b, registers.b
            );
            return false;
        }
        if self.c != registers.c {
            println!(
                "Register C mismatch: expected {}, got {}",
                self.c, registers.c
            );
            return false;
        }
        if self.d != registers.d {
            println!(
                "Register D mismatch: expected {}, got {}",
                self.d, registers.d
            );
            return false;
        }
        if self.e != registers.e {
            println!(
                "Register E mismatch: expected {}, got {}",
                self.e, registers.e
            );
            return false;
        }
        if self.f != u8::from(registers.f) {
            println!(
                "Register F mismatch: expected {:#04x}, got {:#04x}",
                self.f,
                u8::from(registers.f)
            );
            return false;
        }
        if self.h != registers.h {
            println!(
                "Register H mismatch: expected {}, got {}",
                self.h, registers.h
            );
            return false;
        }
        if self.l != registers.l {
            println!(
                "Register L mismatch: expected {}, got {}",
                self.l, registers.l
            );
            return false;
        }
        if self.sp != registers.sp {
            println!(
                "Register SP mismatch: expected {:#06x}, got {:#06x}",
                self.sp, registers.sp
            );
            return false;
        }
        if self.pc != pc {
            println!("PC mismatch: expected {:#06x}, got {:#06x}", self.pc, pc);
            return false;
        }

        // Check RAM state
        for (address, expected_value) in &self.ram {
            let actual_value = cpu.read_byte(*address);
            if *expected_value != actual_value {
                println!(
                    "RAM mismatch at address {:#06x}: expected {:#04x}, got {:#04x}",
                    address, expected_value, actual_value
                );
                return false;
            }
        }

        true
    }
}

/// Run a single test case
///
/// Returns true if the test passed, false otherwise
fn run_test(test: TestCase) -> bool {
    run_test_with_debug(test, false)
}

/// Run a single test case with optional debugging
fn run_test_with_debug(test: TestCase, debug: bool) -> bool {
    // Create CPU and set initial state
    let mut cpu = CPU::new();

    if debug {
        println!("\n=== Test: {} ===", test.name);
        println!("Initial PC in test: {:#06x}", test.initial.pc);
        println!(
            "Actual PC for execution: {:#06x}",
            test.initial.pc.wrapping_sub(1)
        );
        if let Some((addr, val)) = test
            .initial
            .ram
            .iter()
            .find(|(a, _)| *a == test.initial.pc.wrapping_sub(1))
        {
            println!("Instruction byte: {:#04x} at {:#06x}", val, addr);
        }
    }

    test.initial.apply_to_cpu(&mut cpu);

    // Execute one instruction
    cpu.step();

    if true {
        let regs = cpu.get_registers();
        println!(
            "After execution - PC: {:#06x}, B: {:#04x}, C: {:#04x}",
            cpu.get_pc(),
            regs.b,
            regs.c
        );
    }

    // Simulate the prefetch of the next instruction by reading it
    // This matches the decode-execute-prefetch architecture where the final
    // state includes having fetched (but not executed) the next instruction
    let next_pc = cpu.get_pc();
    let _prefetch = cpu.read_byte(next_pc);

    // In a real prefetch architecture, PC would advance during the prefetch
    // The test data expects PC to point past the prefetched instruction
    cpu.set_pc(next_pc.wrapping_add(1));

    if debug {
        println!("After prefetch - PC: {:#06x}", cpu.get_pc());
        println!("Expected PC: {:#06x}", test.final_state.pc);
    }

    // Verify final state
    test.final_state.matches_cpu(&cpu)
}

/// Load and run all test cases from a JSON file
///
/// This function:
/// 1. Reads the JSON file
/// 2. Parses it into a vector of TestCase structs
/// 3. Runs each test case
/// 4. Reports summary statistics
/// 5. Asserts that all tests passed
fn run_json(file_name: &str) {
    let json = fs::read_to_string(file_name).expect("Failed to read test file");
    let tests: Vec<TestCase> = serde_json::from_str(&json).expect("Failed to parse JSON");

    let mut passed = 0;
    let mut failed = 0;
    let mut first_failures = Vec::new();

    for (i, test) in tests.iter().enumerate() {
        let debug = failed == 0 && i == 0; // Debug first test only
        if run_test_with_debug(test.clone(), debug) {
            passed += 1;
        } else {
            failed += 1;
            if first_failures.len() < 3 {
                first_failures.push(test.name.clone());
            }
        }
    }

    println!("\n=== Test Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    println!("Total:  {}", passed + failed);

    if !first_failures.is_empty() {
        println!("\nFirst failures:");
        for name in &first_failures {
            println!("  - {}", name);
        }
    }

    assert_eq!(failed, 0, "{} test(s) failed", failed);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple test to verify the test infrastructure works correctly
    ///
    /// This test manually creates a test case and verifies that the test runner
    /// can properly set up CPU state, execute instructions, and verify results.
    ///
    /// Note: In decode-execute-prefetch architecture, the initial PC points to
    /// the instruction AFTER the one we want to execute (since it's been prefetched).
    /// So PC at 0x1001 means we'll execute the instruction at 0x1000.
    #[test]
    fn test_manual_simple_case() {
        // Create a simple test case for NOP instruction
        // PC starts at 0x1001, which means instruction at 0x1000 has been fetched
        let test = TestCase {
            name: "Manual NOP test".to_string(),
            initial: CpuState {
                a: 100,
                b: 200,
                c: 50,
                d: 75,
                e: 25,
                f: 0,
                h: 150,
                l: 175,
                pc: 0x1001, // Points to next instruction (0x1000 already fetched)
                sp: 0xFFFE,
                ram: vec![(0x1000, 0x00), (0x1001, 0x00)], // NOP instructions
            },
            final_state: CpuState {
                a: 100,
                b: 200,
                c: 50,
                d: 75,
                e: 25,
                f: 0,
                h: 150,
                l: 175,
                pc: 0x1002, // After executing NOP at 0x1000, PC at 0x1002
                sp: 0xFFFE,
                ram: vec![(0x1000, 0x00), (0x1001, 0x00)],
            },
            cycles: vec![Some((0x1000, 0x00, "read".to_string()))],
        };

        assert!(run_test(test), "Manual NOP test should pass");
    }

    /// Test opcode 0x00 (NOP) instruction from JSON
    ///
    /// This test loads test cases from 00.json which tests the NOP instruction.
    /// The test format assumes decode-execute-prefetch architecture where PC
    /// points to the next instruction after the one being tested.
    #[test]
    fn test_00_json() {
        run_json("src/cpu/tests/cases/00.json");
    }

    /// Test opcode 0x01 (LD BC, nn) instruction from JSON
    ///
    /// This test loads test cases from 01.json which tests the LD BC, nn instruction.
    #[test]
    fn test_01_json() {
        run_json("src/cpu/tests/cases/01.json");
    }

    #[test]
    fn test_02_json() {
        run_json("src/cpu/tests/cases/02.json");
    }

    #[test]
    fn test_03_json() {
        run_json("src/cpu/tests/cases/03.json");
    }

    #[test]
    fn test_04_json() {
        run_json("src/cpu/tests/cases/04.json");
    }

    #[test]
    fn test_05_json() {
        run_json("src/cpu/tests/cases/05.json");
    }

    #[test]
    fn test_06_json() {
        run_json("src/cpu/tests/cases/06.json");
    }

    #[test]
    fn test_07_json() {
        run_json("src/cpu/tests/cases/07.json");
    }

    #[test]
    fn test_08_json() {
        run_json("src/cpu/tests/cases/08.json");
    }
}
