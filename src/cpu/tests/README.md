# CPU Instruction Tests

This directory contains infrastructure for testing CPU instruction execution using JSON-based test cases.

## Overview

The test infrastructure allows you to:
- Load test cases from JSON files
- Set up initial CPU and memory state
- Execute a single instruction
- Verify the final CPU and memory state matches expectations

## Decode-Execute-Prefetch Architecture

The tests assume a **decode-execute-prefetch** architecture, which is how the Game Boy CPU operates:

1. **Fetch**: Read the instruction at the current PC
2. **Decode & Execute**: Decode and execute that instruction, updating PC
3. **Prefetch**: Read the next instruction byte (at the new PC)

This means:
- The **initial state** PC points to the instruction AFTER the one we want to test (because it was already prefetched by the previous instruction)
- The **final state** PC points to the instruction AFTER the one we just executed (because we prefetch it)

### Example

If testing a NOP instruction at address 0x1000:
- Initial PC: `0x1001` (the NOP at 0x1000 was already fetched)
- Execute the NOP at 0x1000 → PC advances to 0x1001
- Prefetch the next instruction at 0x1001 → PC advances to 0x1002
- Final PC: `0x1002`

The test runner automatically handles this by:
1. Setting PC to `initial.pc - 1` before execution
2. Executing the instruction
3. Simulating prefetch by reading the next byte and advancing PC

## Architecture

### Components

- **`basic.rs`**: Main test infrastructure
  - `TestCase`: Represents a single test case with initial state, expected final state, and cycles
  - `CpuState`: Represents the complete CPU state (registers, PC, SP, and RAM contents)
  - `run_test()`: Executes a single test case and returns pass/fail
  - `run_json()`: Loads and runs all tests from a JSON file

- **`cases/`**: Directory containing JSON test files
  - Each file (e.g., `00.json`, `01.json`) contains multiple test cases
  - Test files are typically named after opcodes in hexadecimal

### JSON Format

Each test case in a JSON file has the following structure:

```json
{
  "name": "00 22 11",
  "initial": {
    "a": 127,
    "b": 178,
    "c": 123,
    "d": 148,
    "e": 22,
    "f": 0,
    "h": 208,
    "l": 29,
    "pc": 31505,
    "sp": 40276,
    "ram": [
      [31504, 0],
      [31505, 34],
      [31506, 17]
    ]
  },
  "final": {
    "a": 127,
    "b": 178,
    "c": 123,
    "d": 148,
    "e": 22,
    "f": 0,
    "h": 208,
    "l": 29,
    "pc": 31506,
    "sp": 40276,
    "ram": [
      [31504, 0],
      [31505, 34],
      [31506, 17]
    ]
  },
  "cycles": [
    [31505, 34, "read"]
  ]
}
```

In this example:
- The instruction being tested is the NOP (0x00) at address 31504
- Initial PC is 31505 (pointing past the NOP, which was prefetched)
- Final PC is 31506 (after executing NOP and prefetching the next byte)
- The test name "00 22 11" lists the three bytes in memory

### Fields

- **`name`**: Descriptive name for the test case (often shows bytes in memory)
- **`initial`**: Initial CPU state before instruction execution
  - Register values (`a`, `b`, `c`, `d`, `e`, `f`, `h`, `l`)
  - Program counter (`pc`) - points AFTER the instruction to test (already prefetched)
  - Stack pointer (`sp`)
  - Memory contents (`ram`) as array of `[address, value]` pairs
- **`final`**: Expected CPU state after instruction execution and prefetch
  - Same structure as `initial`
  - PC points AFTER the next instruction (which was prefetched but not executed)
- **`cycles`**: Array of memory bus operations during execution
  - Each entry: `[address, value, operation_type]`
  - Operation types: `"read"`, `"write"`
  - Note: May not include all cycles for multi-cycle instructions

## Usage

### Running Tests

Run all tests (excluding ignored tests):
```bash
cargo test
```

Run a specific test:
```bash
cargo test test_manual_simple_case
```

Run ignored tests (like the JSON file tests):
```bash
cargo test -- --ignored
```

### Creating New Tests

#### Manual Test

You can create a test programmatically in Rust:

```rust
#[test]
fn test_my_instruction() {
    let test = TestCase {
        name: "My test".to_string(),
        initial: CpuState {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: 0,
            h: 0,
            l: 0,
            pc: 0x101,  // Points AFTER the instruction at 0x100
            sp: 0xFFFE,
            ram: vec![
                (0x100, 0x00),  // NOP instruction to test
                (0x101, 0x00),  // Next instruction (for prefetch)
            ],
        },
        final_state: CpuState {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: 0,
            h: 0,
            l: 0,
            pc: 0x102,  // After executing NOP and prefetching 0x101
            sp: 0xFFFE,
            ram: vec![(0x100, 0x00), (0x101, 0x00)],
        },
        cycles: vec![],
    };
    
    assert!(run_test(test), "Test should pass");
}
```

**Important**: Remember that initial PC should be `instruction_address + 1` and final PC should be `instruction_address + 2` for single-byte instructions.

#### JSON Test File

Create a new JSON file in the `cases/` directory and add test cases following the format above. Then create a test function:

```rust
#[test]
fn test_my_json_file() {
    run_json("src/cpu/tests/cases/my_file.json");
}
```

## Test Data Compatibility

The JSON test files in the `cases/` directory come from external test suites and follow the decode-execute-prefetch architecture described above. With the proper handling of PC offsets and prefetch simulation, these tests should work correctly with this emulator.

The infrastructure is valuable for:

1. Verifying individual instruction behavior
2. Testing edge cases and flag behavior
3. Ensuring compatibility with reference implementations
4. Regression testing
5. Creating custom test cases

## Example

The `test_manual_simple_case()` function demonstrates a simple passing test that verifies NOP instruction execution:

1. Sets up CPU with specific register values and initial PC at 0x1001 (meaning NOP at 0x1000 was already fetched)
2. Places NOP instructions (0x00) at memory addresses 0x1000 and 0x1001
3. Test runner sets actual CPU PC to 0x1000 (adjusting for prefetch)
4. Executes the NOP instruction at 0x1000 via `cpu.step()`
5. Simulates prefetch of instruction at 0x1001
6. Verifies final PC is 0x1002 and all registers remain unchanged

This demonstrates the decode-execute-prefetch flow where:
- Initial test PC (0x1001) → Actual execution PC (0x1000)
- After execution → PC at 0x1001
- After prefetch → Final PC at 0x1002

## Dependencies

- `serde` and `serde_json`: For parsing JSON test files
- CPU and MemoryBus implementations from parent modules

## Future Improvements

Possible enhancements to the test infrastructure:

- [x] Support for decode-execute-prefetch architecture
- [ ] Full cycle-accurate testing (matching all cycles in the `cycles` array)
- [ ] Better error reporting with diffs
- [ ] Test case generation tools
- [ ] Integration with other GameBoy test ROMs
- [ ] Performance benchmarking using test cases
- [ ] Filtering tests by opcode or instruction type
- [ ] Testing all JSON files (01.json, 02.json, etc.)
- [ ] Parallel test execution for faster runs