#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
from src.compiler import DaisyCompiler


# python main.py your_cpp_file.cpp --flash
# additionally add --port 0483:df11 for "more than one DFU capable USB device found"
def main():
    parser = argparse.ArgumentParser(description="Daisy Seed C++ Compiler and Flasher")
    parser.add_argument("source", help="Source C++ file to compile")
    parser.add_argument("-o", "--output", help="Output binary file name")
    parser.add_argument("-I", "--include", action="append", help="Include path")
    parser.add_argument("-D", "--define", action="append", help="Preprocessor definitions")
    parser.add_argument("--arm-gcc-path", help="Path to ARM GCC toolchain")
    parser.add_argument("--port", help="USB port for flashing")
    parser.add_argument("--flash", action="store_true", help="Flash after compilation")

    args = parser.parse_args()

    try:
        compiler = DaisyCompiler(args.arm_gcc_path)
        
        # Set default output name if not provided
        if not args.output:
            args.output = Path(args.source).stem + ".bin"

        # Create intermediate object file
        obj_file = Path(args.source).stem + ".o"
        
        # Compile
        print(f"Compiling {args.source}...")
        compiler.compile_cpp(
            args.source,
            obj_file,
            args.include,
            args.define
        )

        # Create binary file
        print(f"Creating binary file {args.output}...")
        compiler.create_binary(obj_file, args.output)

        # Flash if requested
        if args.flash:
            print("Flashing to Daisy board...")
            compiler.flash(args.output, args.port)
            print("Flashing complete!")

        # Clean up object file
        Path(obj_file).unlink()
        
        print("All operations completed successfully!")

    except Exception as e:
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()