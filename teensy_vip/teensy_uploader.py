import os
import subprocess

class TeensyUploader:
    def __init__(self, teensy_loader_cli_path):
        self.teensy_loader_cli_path = teensy_loader_cli_path
        self.arduino_cli_path = os.path.join(os.path.dirname(__file__), 'arduino-cli')
        self.board_fqbn = "teensy:avr:teensy41:usb=audio"  # Updated to include USB audio setting

    def compile_sketch(self, ino_file_path):
        """
        Compiles the .ino file using Arduino CLI.
        """
        try:
            print("Compiling the sketch for Teensy 4.1...")
            result = subprocess.run(
                [self.arduino_cli_path, "compile", "--fqbn", self.board_fqbn, ino_file_path],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("Compilation successful.")
            return True
        except subprocess.CalledProcessError as e:
            print("Compilation failed:", e.stderr.decode())
            return False
        except FileNotFoundError:
            print("Error: Arduino CLI not found. Ensure it is in the project directory.")
            return False

    def upload_to_teensy(self, hex_file_path):
        """
        Uploads the compiled .hex file to the Teensy board.
        """
        try:
            print("Uploading to Teensy 4.1...")
            result = subprocess.run(
                [self.teensy_loader_cli_path, "--mcu=TEENSY41", "-w", hex_file_path],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("Upload successful.")
            return True
        except subprocess.CalledProcessError as e:
            print("Upload failed:", e.stderr.decode())
            return False
        except FileNotFoundError:
            print("Error: Teensy Loader CLI not found. Ensure it is in the specified path.")
            return False

    def find_hex_file(self, ino_file_path):
        """
        Finds the .hex file generated after compilation in the specified build directory.
        """
        build_dir = os.path.join(os.path.dirname(__file__), 'hex')
        hex_files = [f for f in os.listdir(build_dir) if f.endswith('.hex')]
        if hex_files:
            return os.path.join(build_dir, hex_files[0])
        return None