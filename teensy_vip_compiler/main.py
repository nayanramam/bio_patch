import click
from teensy_uploader import TeensyUploader

@click.command()
@click.argument('ino_file_path', type=click.Path(exists=True))
@click.option('--teensy-loader-cli', default='teensy_loader_cli', help='Path to the Teensy Loader CLI executable.')
def main(ino_file_path, teensy_loader_cli):
    """
    Command-line interface to compile and upload .ino files to a Teensy 4.1 board.
    """
    uploader = TeensyUploader(teensy_loader_cli)

    # Compile the .ino file
    if not uploader.compile_sketch(ino_file_path):
        click.echo("Error: Compilation failed.")
        return

    # Find the generated .hex file
    hex_file = uploader.find_hex_file(ino_file_path)
    if not hex_file:
        click.echo("Error: Compiled .hex file not found.")
        return

    # Upload the .hex file to the Teensy board
    if not uploader.upload_to_teensy(hex_file):
        click.echo("Error: Upload failed.")
        return

    click.echo("Success: Sketch uploaded to Teensy 4.1.")

if __name__ == '__main__':
    main()
