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
use crate::mem::MemoryFlat;
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
    fn apply_to_cpu(&self, cpu: &mut CPU<MemoryFlat>) {
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
    fn matches_cpu(&self, cpu: &CPU<MemoryFlat>) -> bool {
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
    cpu.prefetch();
    cpu.step();

    if debug {
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

/// Uses JSON tests from https://github.com/adtennant/GameboyCPUTests/tree/master/v2
/// These tests don't include STOP/HALT/DI/EI instructions, and there are no tests for not supported instructions
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

    ///
    /// This test loads test cases from 00.json which tests the NOP instruction.
    /// The test format assumes decode-execute-prefetch architecture where PC
    /// points to the next instruction after the one being tested.
    #[test]
    fn test_00_json() {
        run_json("src/cpu/tests/cases/00.json");
    }

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

    #[test]
    fn test_09_json() {
        run_json("src/cpu/tests/cases/09.json");
    }

    #[test]
    fn test_0a_json() {
        run_json("src/cpu/tests/cases/0a.json");
    }

    #[test]
    fn test_0b_json() {
        run_json("src/cpu/tests/cases/0b.json");
    }

    #[test]
    fn test_0c_json() {
        run_json("src/cpu/tests/cases/0c.json");
    }

    #[test]
    fn test_0d_json() {
        run_json("src/cpu/tests/cases/0d.json");
    }

    #[test]
    fn test_0e_json() {
        run_json("src/cpu/tests/cases/0e.json");
    }

    #[test]
    fn test_0f_json() {
        run_json("src/cpu/tests/cases/0f.json");
    }

    #[test]
    fn test_11_json() {
        run_json("src/cpu/tests/cases/11.json");
    }

    #[test]
    fn test_12_json() {
        run_json("src/cpu/tests/cases/12.json");
    }

    #[test]
    fn test_13_json() {
        run_json("src/cpu/tests/cases/13.json");
    }

    #[test]
    fn test_14_json() {
        run_json("src/cpu/tests/cases/14.json");
    }

    #[test]
    fn test_15_json() {
        run_json("src/cpu/tests/cases/15.json");
    }

    #[test]
    fn test_16_json() {
        run_json("src/cpu/tests/cases/16.json");
    }

    #[test]
    fn test_17_json() {
        run_json("src/cpu/tests/cases/17.json");
    }

    #[test]
    fn test_18_json() {
        run_json("src/cpu/tests/cases/18.json");
    }

    #[test]
    fn test_19_json() {
        run_json("src/cpu/tests/cases/19.json");
    }

    #[test]
    fn test_1a_json() {
        run_json("src/cpu/tests/cases/1a.json");
    }

    #[test]
    fn test_1b_json() {
        run_json("src/cpu/tests/cases/1b.json");
    }

    #[test]
    fn test_1c_json() {
        run_json("src/cpu/tests/cases/1c.json");
    }

    #[test]
    fn test_1d_json() {
        run_json("src/cpu/tests/cases/1d.json");
    }

    #[test]
    fn test_1e_json() {
        run_json("src/cpu/tests/cases/1e.json");
    }

    #[test]
    fn test_1f_json() {
        run_json("src/cpu/tests/cases/1f.json");
    }

    #[test]
    fn test_20_json() {
        run_json("src/cpu/tests/cases/20.json");
    }

    #[test]
    fn test_21_json() {
        run_json("src/cpu/tests/cases/21.json");
    }

    #[test]
    fn test_22_json() {
        run_json("src/cpu/tests/cases/22.json");
    }

    #[test]
    fn test_23_json() {
        run_json("src/cpu/tests/cases/23.json");
    }

    #[test]
    fn test_24_json() {
        run_json("src/cpu/tests/cases/24.json");
    }

    #[test]
    fn test_25_json() {
        run_json("src/cpu/tests/cases/25.json");
    }

    #[test]
    fn test_26_json() {
        run_json("src/cpu/tests/cases/26.json");
    }

    #[test]
    fn test_27_json() {
        run_json("src/cpu/tests/cases/27.json");
    }

    #[test]
    fn test_28_json() {
        run_json("src/cpu/tests/cases/28.json");
    }

    #[test]
    fn test_29_json() {
        run_json("src/cpu/tests/cases/29.json");
    }

    #[test]
    fn test_2a_json() {
        run_json("src/cpu/tests/cases/2a.json");
    }

    #[test]
    fn test_2b_json() {
        run_json("src/cpu/tests/cases/2b.json");
    }

    #[test]
    fn test_2c_json() {
        run_json("src/cpu/tests/cases/2c.json");
    }

    #[test]
    fn test_2d_json() {
        run_json("src/cpu/tests/cases/2d.json");
    }

    #[test]
    fn test_2e_json() {
        run_json("src/cpu/tests/cases/2e.json");
    }

    #[test]
    fn test_2f_json() {
        run_json("src/cpu/tests/cases/2f.json");
    }

    #[test]
    fn test_30_json() {
        run_json("src/cpu/tests/cases/30.json");
    }

    #[test]
    fn test_31_json() {
        run_json("src/cpu/tests/cases/31.json");
    }

    #[test]
    fn test_32_json() {
        run_json("src/cpu/tests/cases/32.json");
    }

    #[test]
    fn test_33_json() {
        run_json("src/cpu/tests/cases/33.json");
    }

    #[test]
    fn test_34_json() {
        run_json("src/cpu/tests/cases/34.json");
    }

    #[test]
    fn test_35_json() {
        run_json("src/cpu/tests/cases/35.json");
    }

    #[test]
    fn test_36_json() {
        run_json("src/cpu/tests/cases/36.json");
    }

    #[test]
    fn test_37_json() {
        run_json("src/cpu/tests/cases/37.json");
    }

    #[test]
    fn test_38_json() {
        run_json("src/cpu/tests/cases/38.json");
    }

    #[test]
    fn test_39_json() {
        run_json("src/cpu/tests/cases/39.json");
    }

    #[test]
    fn test_3a_json() {
        run_json("src/cpu/tests/cases/3a.json");
    }

    #[test]
    fn test_3b_json() {
        run_json("src/cpu/tests/cases/3b.json");
    }

    #[test]
    fn test_3c_json() {
        run_json("src/cpu/tests/cases/3c.json");
    }

    #[test]
    fn test_3d_json() {
        run_json("src/cpu/tests/cases/3d.json");
    }

    #[test]
    fn test_3e_json() {
        run_json("src/cpu/tests/cases/3e.json");
    }

    #[test]
    fn test_3f_json() {
        run_json("src/cpu/tests/cases/3f.json");
    }

    #[test]
    fn test_40_json() {
        run_json("src/cpu/tests/cases/40.json");
    }

    #[test]
    fn test_41_json() {
        run_json("src/cpu/tests/cases/41.json");
    }

    #[test]
    fn test_42_json() {
        run_json("src/cpu/tests/cases/42.json");
    }

    #[test]
    fn test_43_json() {
        run_json("src/cpu/tests/cases/43.json");
    }

    #[test]
    fn test_44_json() {
        run_json("src/cpu/tests/cases/44.json");
    }

    #[test]
    fn test_45_json() {
        run_json("src/cpu/tests/cases/45.json");
    }

    #[test]
    fn test_46_json() {
        run_json("src/cpu/tests/cases/46.json");
    }

    #[test]
    fn test_47_json() {
        run_json("src/cpu/tests/cases/47.json");
    }

    #[test]
    fn test_48_json() {
        run_json("src/cpu/tests/cases/48.json");
    }

    #[test]
    fn test_49_json() {
        run_json("src/cpu/tests/cases/49.json");
    }

    #[test]
    fn test_4a_json() {
        run_json("src/cpu/tests/cases/4a.json");
    }

    #[test]
    fn test_4b_json() {
        run_json("src/cpu/tests/cases/4b.json");
    }

    #[test]
    fn test_4c_json() {
        run_json("src/cpu/tests/cases/4c.json");
    }

    #[test]
    fn test_4d_json() {
        run_json("src/cpu/tests/cases/4d.json");
    }

    #[test]
    fn test_4e_json() {
        run_json("src/cpu/tests/cases/4e.json");
    }

    #[test]
    fn test_4f_json() {
        run_json("src/cpu/tests/cases/4f.json");
    }

    #[test]
    fn test_50_json() {
        run_json("src/cpu/tests/cases/50.json");
    }

    #[test]
    fn test_51_json() {
        run_json("src/cpu/tests/cases/51.json");
    }

    #[test]
    fn test_52_json() {
        run_json("src/cpu/tests/cases/52.json");
    }

    #[test]
    fn test_53_json() {
        run_json("src/cpu/tests/cases/53.json");
    }

    #[test]
    fn test_54_json() {
        run_json("src/cpu/tests/cases/54.json");
    }

    #[test]
    fn test_55_json() {
        run_json("src/cpu/tests/cases/55.json");
    }

    #[test]
    fn test_56_json() {
        run_json("src/cpu/tests/cases/56.json");
    }

    #[test]
    fn test_57_json() {
        run_json("src/cpu/tests/cases/57.json");
    }

    #[test]
    fn test_58_json() {
        run_json("src/cpu/tests/cases/58.json");
    }

    #[test]
    fn test_59_json() {
        run_json("src/cpu/tests/cases/59.json");
    }

    #[test]
    fn test_5a_json() {
        run_json("src/cpu/tests/cases/5a.json");
    }

    #[test]
    fn test_5b_json() {
        run_json("src/cpu/tests/cases/5b.json");
    }

    #[test]
    fn test_5c_json() {
        run_json("src/cpu/tests/cases/5c.json");
    }

    #[test]
    fn test_5d_json() {
        run_json("src/cpu/tests/cases/5d.json");
    }

    #[test]
    fn test_5e_json() {
        run_json("src/cpu/tests/cases/5e.json");
    }

    #[test]
    fn test_5f_json() {
        run_json("src/cpu/tests/cases/5f.json");
    }

    #[test]
    fn test_60_json() {
        run_json("src/cpu/tests/cases/60.json");
    }

    #[test]
    fn test_61_json() {
        run_json("src/cpu/tests/cases/61.json");
    }

    #[test]
    fn test_62_json() {
        run_json("src/cpu/tests/cases/62.json");
    }

    #[test]
    fn test_63_json() {
        run_json("src/cpu/tests/cases/63.json");
    }

    #[test]
    fn test_64_json() {
        run_json("src/cpu/tests/cases/64.json");
    }

    #[test]
    fn test_65_json() {
        run_json("src/cpu/tests/cases/65.json");
    }

    #[test]
    fn test_66_json() {
        run_json("src/cpu/tests/cases/66.json");
    }

    #[test]
    fn test_67_json() {
        run_json("src/cpu/tests/cases/67.json");
    }

    #[test]
    fn test_68_json() {
        run_json("src/cpu/tests/cases/68.json");
    }

    #[test]
    fn test_69_json() {
        run_json("src/cpu/tests/cases/69.json");
    }

    #[test]
    fn test_6a_json() {
        run_json("src/cpu/tests/cases/6a.json");
    }

    #[test]
    fn test_6b_json() {
        run_json("src/cpu/tests/cases/6b.json");
    }

    #[test]
    fn test_6c_json() {
        run_json("src/cpu/tests/cases/6c.json");
    }

    #[test]
    fn test_6d_json() {
        run_json("src/cpu/tests/cases/6d.json");
    }

    #[test]
    fn test_6e_json() {
        run_json("src/cpu/tests/cases/6e.json");
    }

    #[test]
    fn test_6f_json() {
        run_json("src/cpu/tests/cases/6f.json");
    }

    #[test]
    fn test_70_json() {
        run_json("src/cpu/tests/cases/70.json");
    }

    #[test]
    fn test_71_json() {
        run_json("src/cpu/tests/cases/71.json");
    }

    #[test]
    fn test_72_json() {
        run_json("src/cpu/tests/cases/72.json");
    }

    #[test]
    fn test_73_json() {
        run_json("src/cpu/tests/cases/73.json");
    }

    #[test]
    fn test_74_json() {
        run_json("src/cpu/tests/cases/74.json");
    }

    #[test]
    fn test_75_json() {
        run_json("src/cpu/tests/cases/75.json");
    }

    #[test]
    fn test_77_json() {
        run_json("src/cpu/tests/cases/77.json");
    }

    #[test]
    fn test_78_json() {
        run_json("src/cpu/tests/cases/78.json");
    }

    #[test]
    fn test_79_json() {
        run_json("src/cpu/tests/cases/79.json");
    }

    #[test]
    fn test_7a_json() {
        run_json("src/cpu/tests/cases/7a.json");
    }

    #[test]
    fn test_7b_json() {
        run_json("src/cpu/tests/cases/7b.json");
    }

    #[test]
    fn test_7c_json() {
        run_json("src/cpu/tests/cases/7c.json");
    }

    #[test]
    fn test_7d_json() {
        run_json("src/cpu/tests/cases/7d.json");
    }

    #[test]
    fn test_7e_json() {
        run_json("src/cpu/tests/cases/7e.json");
    }

    #[test]
    fn test_7f_json() {
        run_json("src/cpu/tests/cases/7f.json");
    }

    #[test]
    fn test_80_json() {
        run_json("src/cpu/tests/cases/80.json");
    }

    #[test]
    fn test_81_json() {
        run_json("src/cpu/tests/cases/81.json");
    }

    #[test]
    fn test_82_json() {
        run_json("src/cpu/tests/cases/82.json");
    }

    #[test]
    fn test_83_json() {
        run_json("src/cpu/tests/cases/83.json");
    }

    #[test]
    fn test_84_json() {
        run_json("src/cpu/tests/cases/84.json");
    }

    #[test]
    fn test_85_json() {
        run_json("src/cpu/tests/cases/85.json");
    }

    #[test]
    fn test_86_json() {
        run_json("src/cpu/tests/cases/86.json");
    }

    #[test]
    fn test_87_json() {
        run_json("src/cpu/tests/cases/87.json");
    }

    #[test]
    fn test_88_json() {
        run_json("src/cpu/tests/cases/88.json");
    }

    #[test]
    fn test_89_json() {
        run_json("src/cpu/tests/cases/89.json");
    }

    #[test]
    fn test_8a_json() {
        run_json("src/cpu/tests/cases/8a.json");
    }

    #[test]
    fn test_8b_json() {
        run_json("src/cpu/tests/cases/8b.json");
    }

    #[test]
    fn test_8c_json() {
        run_json("src/cpu/tests/cases/8c.json");
    }

    #[test]
    fn test_8d_json() {
        run_json("src/cpu/tests/cases/8d.json");
    }

    #[test]
    fn test_8e_json() {
        run_json("src/cpu/tests/cases/8e.json");
    }

    #[test]
    fn test_8f_json() {
        run_json("src/cpu/tests/cases/8f.json");
    }

    #[test]
    fn test_90_json() {
        run_json("src/cpu/tests/cases/90.json");
    }

    #[test]
    fn test_91_json() {
        run_json("src/cpu/tests/cases/91.json");
    }

    #[test]
    fn test_92_json() {
        run_json("src/cpu/tests/cases/92.json");
    }

    #[test]
    fn test_93_json() {
        run_json("src/cpu/tests/cases/93.json");
    }

    #[test]
    fn test_94_json() {
        run_json("src/cpu/tests/cases/94.json");
    }

    #[test]
    fn test_95_json() {
        run_json("src/cpu/tests/cases/95.json");
    }

    #[test]
    fn test_96_json() {
        run_json("src/cpu/tests/cases/96.json");
    }

    #[test]
    fn test_97_json() {
        run_json("src/cpu/tests/cases/97.json");
    }

    #[test]
    fn test_98_json() {
        run_json("src/cpu/tests/cases/98.json");
    }

    #[test]
    fn test_99_json() {
        run_json("src/cpu/tests/cases/99.json");
    }

    #[test]
    fn test_9a_json() {
        run_json("src/cpu/tests/cases/9a.json");
    }

    #[test]
    fn test_9b_json() {
        run_json("src/cpu/tests/cases/9b.json");
    }

    #[test]
    fn test_9c_json() {
        run_json("src/cpu/tests/cases/9c.json");
    }

    #[test]
    fn test_9d_json() {
        run_json("src/cpu/tests/cases/9d.json");
    }

    #[test]
    fn test_9e_json() {
        run_json("src/cpu/tests/cases/9e.json");
    }

    #[test]
    fn test_9f_json() {
        run_json("src/cpu/tests/cases/9f.json");
    }

    #[test]
    fn test_a0_json() {
        run_json("src/cpu/tests/cases/a0.json");
    }

    #[test]
    fn test_a1_json() {
        run_json("src/cpu/tests/cases/a1.json");
    }

    #[test]
    fn test_a2_json() {
        run_json("src/cpu/tests/cases/a2.json");
    }

    #[test]
    fn test_a3_json() {
        run_json("src/cpu/tests/cases/a3.json");
    }

    #[test]
    fn test_a4_json() {
        run_json("src/cpu/tests/cases/a4.json");
    }

    #[test]
    fn test_a5_json() {
        run_json("src/cpu/tests/cases/a5.json");
    }

    #[test]
    fn test_a6_json() {
        run_json("src/cpu/tests/cases/a6.json");
    }

    #[test]
    fn test_a7_json() {
        run_json("src/cpu/tests/cases/a7.json");
    }

    #[test]
    fn test_a8_json() {
        run_json("src/cpu/tests/cases/a8.json");
    }

    #[test]
    fn test_a9_json() {
        run_json("src/cpu/tests/cases/a9.json");
    }

    #[test]
    fn test_aa_json() {
        run_json("src/cpu/tests/cases/aa.json");
    }

    #[test]
    fn test_ab_json() {
        run_json("src/cpu/tests/cases/ab.json");
    }

    #[test]
    fn test_ac_json() {
        run_json("src/cpu/tests/cases/ac.json");
    }

    #[test]
    fn test_ad_json() {
        run_json("src/cpu/tests/cases/ad.json");
    }

    #[test]
    fn test_ae_json() {
        run_json("src/cpu/tests/cases/ae.json");
    }

    #[test]
    fn test_af_json() {
        run_json("src/cpu/tests/cases/af.json");
    }

    #[test]
    fn test_b0_json() {
        run_json("src/cpu/tests/cases/b0.json");
    }

    #[test]
    fn test_b1_json() {
        run_json("src/cpu/tests/cases/b1.json");
    }

    #[test]
    fn test_b2_json() {
        run_json("src/cpu/tests/cases/b2.json");
    }

    #[test]
    fn test_b3_json() {
        run_json("src/cpu/tests/cases/b3.json");
    }

    #[test]
    fn test_b4_json() {
        run_json("src/cpu/tests/cases/b4.json");
    }

    #[test]
    fn test_b5_json() {
        run_json("src/cpu/tests/cases/b5.json");
    }

    #[test]
    fn test_b6_json() {
        run_json("src/cpu/tests/cases/b6.json");
    }

    #[test]
    fn test_b7_json() {
        run_json("src/cpu/tests/cases/b7.json");
    }

    #[test]
    fn test_b8_json() {
        run_json("src/cpu/tests/cases/b8.json");
    }

    #[test]
    fn test_b9_json() {
        run_json("src/cpu/tests/cases/b9.json");
    }

    #[test]
    fn test_ba_json() {
        run_json("src/cpu/tests/cases/ba.json");
    }

    #[test]
    fn test_bb_json() {
        run_json("src/cpu/tests/cases/bb.json");
    }

    #[test]
    fn test_bc_json() {
        run_json("src/cpu/tests/cases/bc.json");
    }

    #[test]
    fn test_bd_json() {
        run_json("src/cpu/tests/cases/bd.json");
    }

    #[test]
    fn test_be_json() {
        run_json("src/cpu/tests/cases/be.json");
    }

    #[test]
    fn test_bf_json() {
        run_json("src/cpu/tests/cases/bf.json");
    }

    #[test]
    fn test_c0_json() {
        run_json("src/cpu/tests/cases/c0.json");
    }

    #[test]
    fn test_c1_json() {
        run_json("src/cpu/tests/cases/c1.json");
    }

    #[test]
    fn test_c2_json() {
        run_json("src/cpu/tests/cases/c2.json");
    }

    #[test]
    fn test_c3_json() {
        run_json("src/cpu/tests/cases/c3.json");
    }

    #[test]
    fn test_c4_json() {
        run_json("src/cpu/tests/cases/c4.json");
    }

    #[test]
    fn test_c5_json() {
        run_json("src/cpu/tests/cases/c5.json");
    }

    #[test]
    fn test_c6_json() {
        run_json("src/cpu/tests/cases/c6.json");
    }

    #[test]
    fn test_c7_json() {
        run_json("src/cpu/tests/cases/c7.json");
    }

    #[test]
    fn test_c8_json() {
        run_json("src/cpu/tests/cases/c8.json");
    }

    #[test]
    fn test_c9_json() {
        run_json("src/cpu/tests/cases/c9.json");
    }

    #[test]
    fn test_ca_json() {
        run_json("src/cpu/tests/cases/ca.json");
    }

    #[test]
    fn test_cb_json() {
        run_json("src/cpu/tests/cases/cb.json");
    }

    #[test]
    fn test_cc_json() {
        run_json("src/cpu/tests/cases/cc.json");
    }

    #[test]
    fn test_cd_json() {
        run_json("src/cpu/tests/cases/cd.json");
    }

    #[test]
    fn test_ce_json() {
        run_json("src/cpu/tests/cases/ce.json");
    }

    #[test]
    fn test_cf_json() {
        run_json("src/cpu/tests/cases/cf.json");
    }

    #[test]
    fn test_d0_json() {
        run_json("src/cpu/tests/cases/d0.json");
    }

    #[test]
    fn test_d1_json() {
        run_json("src/cpu/tests/cases/d1.json");
    }

    #[test]
    fn test_d2_json() {
        run_json("src/cpu/tests/cases/d2.json");
    }

    #[test]
    fn test_d4_json() {
        run_json("src/cpu/tests/cases/d4.json");
    }

    #[test]
    fn test_d5_json() {
        run_json("src/cpu/tests/cases/d5.json");
    }

    #[test]
    fn test_d6_json() {
        run_json("src/cpu/tests/cases/d6.json");
    }

    #[test]
    fn test_d7_json() {
        run_json("src/cpu/tests/cases/d7.json");
    }

    #[test]
    fn test_d8_json() {
        run_json("src/cpu/tests/cases/d8.json");
    }

    #[test]
    fn test_d9_json() {
        run_json("src/cpu/tests/cases/d9.json");
    }

    #[test]
    fn test_da_json() {
        run_json("src/cpu/tests/cases/da.json");
    }

    #[test]
    fn test_dc_json() {
        run_json("src/cpu/tests/cases/dc.json");
    }

    #[test]
    fn test_de_json() {
        run_json("src/cpu/tests/cases/de.json");
    }

    #[test]
    fn test_df_json() {
        run_json("src/cpu/tests/cases/df.json");
    }

    #[test]
    fn test_e0_json() {
        run_json("src/cpu/tests/cases/e0.json");
    }

    #[test]
    fn test_e1_json() {
        run_json("src/cpu/tests/cases/e1.json");
    }

    #[test]
    fn test_e2_json() {
        run_json("src/cpu/tests/cases/e2.json");
    }

    #[test]
    fn test_e5_json() {
        run_json("src/cpu/tests/cases/e5.json");
    }

    #[test]
    fn test_e6_json() {
        run_json("src/cpu/tests/cases/e6.json");
    }

    #[test]
    fn test_e7_json() {
        run_json("src/cpu/tests/cases/e7.json");
    }

    #[test]
    fn test_e8_json() {
        run_json("src/cpu/tests/cases/e8.json");
    }

    #[test]
    fn test_e9_json() {
        run_json("src/cpu/tests/cases/e9.json");
    }

    #[test]
    fn test_ea_json() {
        run_json("src/cpu/tests/cases/ea.json");
    }

    #[test]
    fn test_ee_json() {
        run_json("src/cpu/tests/cases/ee.json");
    }

    #[test]
    fn test_ef_json() {
        run_json("src/cpu/tests/cases/ef.json");
    }

    #[test]
    fn test_f0_json() {
        run_json("src/cpu/tests/cases/f0.json");
    }

    #[test]
    fn test_f1_json() {
        run_json("src/cpu/tests/cases/f1.json");
    }

    #[test]
    fn test_f2_json() {
        run_json("src/cpu/tests/cases/f2.json");
    }

    #[test]
    fn test_f5_json() {
        run_json("src/cpu/tests/cases/f5.json");
    }

    #[test]
    fn test_f6_json() {
        run_json("src/cpu/tests/cases/f6.json");
    }

    #[test]
    fn test_f7_json() {
        run_json("src/cpu/tests/cases/f7.json");
    }

    #[test]
    fn test_f8_json() {
        run_json("src/cpu/tests/cases/f8.json");
    }

    #[test]
    fn test_f9_json() {
        run_json("src/cpu/tests/cases/f9.json");
    }

    #[test]
    fn test_fa_json() {
        run_json("src/cpu/tests/cases/fa.json");
    }

    #[test]
    fn test_fe_json() {
        run_json("src/cpu/tests/cases/fe.json");
    }

    #[test]
    fn test_ff_json() {
        run_json("src/cpu/tests/cases/ff.json");
    }
}
