# Daisy VIP Compiler

A Python-based compiler and flasher tool for Electro-Smith Daisy Seed board development. This tool helps streamline the process of compiling C++ code and flashing it to the Daisy Seed board.

## Prerequisites

1. ARM GCC Toolchain (arm-none-eabi-gcc)
2. dfu-util
3. Python 3.6 or higher

## Installation

1. Ensure you have the ARM GCC toolchain installed and in your PATH
2. Install dfu-util
3. Clone this repository

## Usage

Basic usage:
```bash
python main.py source.cpp
```

Options:
- `-o, --output`: Specify output hex file name
- `-I, --include`: Add include path (can be used multiple times)
- `-D, --define`: Add preprocessor definition (can be used multiple times)
- `--arm-gcc-path`: Manually specify ARM GCC toolchain path
- `--port`: Specify USB port for flashing
- `--flash`: Flash after compilation

Example with options:
```bash
python main.py source.cpp -o output.hex -I./include -DDEBUG --flash
```

## Development

The project structure is organized as follows:

```
daisy_vip_compiler/
├── src/
│   └── compiler.py     # Main compiler implementation
├── tests/              # Test files
├── main.py            # Command-line interface
└── README.md          # This file
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

MIT License