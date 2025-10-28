import os
import subprocess
import platform
from pathlib import Path

class DaisyCompiler:
    def __init__(self, arm_gcc_path=None):
        """Initialize the Daisy compiler with optional ARM GCC toolchain path."""
        self.arm_gcc_path = arm_gcc_path or "armgcc/bin/"
        print(self.arm_gcc_path)
        if not self.arm_gcc_path:
            raise RuntimeError("ARM GCC toolchain not found. Please install it or provide the path.")

    def _find_arm_gcc(self):
        """Attempt to find ARM GCC toolchain in system PATH."""
        gcc_name = "arm-none-eabi-gcc.exe" if platform.system() == "Windows" else "arm-none-eabi-gcc"
        for path in os.environ["PATH"].split(os.pathsep):
            gcc_path = Path(path) / gcc_name
            if gcc_path.exists():
                return str(gcc_path.parent)
        return None

    def compile_cpp(self, source_file: str, output_file: str, include_paths=None, defines=None):
        """Compile C++ source file to object file."""
        if not os.path.exists(source_file):
            raise FileNotFoundError(f"Source file not found: {source_file}")

        cmd = [
            str(Path(self.arm_gcc_path) / "arm-none-eabi-g++"),
            "-c",
            "-mcpu=cortex-m7",
            "-mthumb",
            "-mfloat-abi=hard",
            "-mfpu=fpv5-d16",
            "-Os",
            "-Wall",
            source_file,
            "-o",
            output_file
        ]

        if include_paths:
            for path in include_paths:
                cmd.extend(["-I", path])

        if defines:
            for define in defines:
                cmd.extend(["-D", define])

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Compilation failed: {e.stderr}")

    def create_binary(self, object_file: str, bin_file: str):
        """Convert object file to binary format."""
        if not os.path.exists(object_file):
            raise FileNotFoundError(f"Object file not found: {object_file}")

        cmd = [
            str(Path(self.arm_gcc_path) / "arm-none-eabi-objcopy"),
            "-O",
            "binary",
            object_file,
            bin_file
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Binary file creation failed: {e.stderr}")

    def flash(self, bin_file: str, port: str = None):
        """Flash binary file to Daisy board using dfu-util."""
        if not os.path.exists(bin_file):
            raise FileNotFoundError(f"Binary file not found: {bin_file}")

        cmd = [
            "dfu\\dfu-util",
            "-a",
            "0",
            "-s",
            "0x08000000",
            "-D",
            bin_file
        ]

        if port:
            cmd.extend(["-d", port])

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Flashing failed: {e.stderr}")
        except FileNotFoundError:
            raise RuntimeError("dfu-util not found. Please install it and ensure it's in your PATH.")