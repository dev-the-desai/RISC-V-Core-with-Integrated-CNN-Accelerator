# RISC-V-Core-with-Integrated-CNN-Accelerator

A high-performance 32-bit RISC-V processor core with an integrated CNN accelerator, implemented on Xilinx Nexys A7 FPGA. This project combines general-purpose computing capabilities with specialized neural network acceleration, optimized for embedded systems and edge computing applications.

## Core Features

### RISC-V Processor Implementation
- Complete RV32I ISA support
- Configurable pipeline stages
- Hardware-level hazard resolution
- Built-in branch prediction
- Custom instruction extensions for CNN operations

### CNN Accelerator Module
- Dedicated neural network processing unit
- Support for convolutional layers
- Max pooling capabilities
- Weight buffer management
- Configurable layer parameters

## Technical Specifications

### Processor Architecture
- Implementation Platform: Xilinx Nexys A7
- Core Components:
  - 5-stage pipeline
  - Register file (32 x 32-bit)
  - ALU with standard operations
  - Memory interface unit
- Clock Management:
  - Core clock: 40 MHz
  - CNN clock: 13 MHz

### Pipeline Implementation Details
- Stage Organization:
  - Instruction fetch
  - Decode
  - Execute
  - Memory access
  - Write-back
- Hazard Handling:
  - Data forwarding paths
  - Pipeline stall logic
  - Branch prediction unit
- Control Logic:
  - Instruction decoder
  - Pipeline controller
  - Memory access arbitration

### CNN Accelerator Details
- Processing Elements:
  - Convolution units
  - Pooling modules
  - Weight buffer
- Configuration Options:
  - Input size (32x32)
  - Kernel size (3x3)
  - Layer parameters
  - Activation functions
