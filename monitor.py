"""Flash inverted_pendulum_control.ino and/or watch its serial output.

Usage:
    python monitor.py                 # finds the Mega, 115200 baud
    python monitor.py --port COM4     # or name the port yourself
    python monitor.py --upload        # flash the sketch first, then monitor
    python monitor.py --upload-only   # just flash it
    python monitor.py --log run1.txt  # also append every line to a file

Opening the port resets the Mega, so the sketch re-zeroes both encoders on
connect: let the pendulum hang at rest with the arm centred when starting this.

Anything typed into this terminal is forwarded to the sketch. Ctrl-C to stop.

Needs: pip install pyserial
"""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

import serial
from serial.tools import list_ports

# USB (vendor, product) ids a Mega enumerates as: Arduino and Arduino SRL
# vendors, times the Mega 2560, Mega 2560 R3 and Mega ADK products
MEGA_USB_IDS = {
    (vendor, product)
    for vendor in (0x2341, 0x2A03)
    for product in (0x0010, 0x0042, 0x003F, 0x0044)
}

# the console build of the Arduino 1.8 IDE, used when arduino-cli isn't around
IDE_EXE = Path(r"C:\Program Files (x86)\Arduino\arduino_debug.exe")

# log4j lines the IDE prints on the way up; ~420 of them per invocation
NOISE_RE = re.compile(r"^(TRACE|DEBUG|INFO|WARN|ERROR|FATAL)\s+StatusLogger\b")


def describe(port):
    return f"{port.device} ({port.description})"


def find_mega_port(requested):
    """Locate the Mega by its USB ids, unless a port was named outright."""
    if requested:
        return requested

    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    megas = [port for port in ports if (port.vid, port.pid) in MEGA_USB_IDS]

    if len(megas) == 1:
        print(f"found Arduino Mega on {describe(megas[0])}")
        return megas[0].device

    if len(megas) > 1:
        sys.exit(
            "several Megas are attached, name one with --port:\n  "
            + "\n  ".join(describe(port) for port in megas)
        )

    # A clone with a CH340/CP210x bridge reports the bridge's ids, not
    # Arduino's, so it cannot be told apart from any other board. Take a lone
    # USB serial port as a guess, and say that is what happened.
    usb_ports = [port for port in ports if port.vid is not None]
    if len(usb_ports) == 1:
        print(
            f"no board with Arduino's USB ids; assuming the only USB serial "
            f"port, {describe(usb_ports[0])}"
        )
        return usb_ports[0].device

    listing = "\n  ".join(describe(port) for port in ports) or "(none)"
    sys.exit(f"could not find an Arduino Mega. Ports seen:\n  {listing}")


def upload_sketch(port, fqbn, sketch):
    """Build and flash the sketch, preferring arduino-cli and falling back to the IDE."""
    if not sketch.exists():
        sys.exit(f"no sketch at {sketch}")

    # Both tools insist the .ino lives in a folder of the same name, which this
    # one doesn't, so build from a copy in a correctly named temp folder.
    staging = Path(tempfile.mkdtemp(prefix="pendulum_upload_")) / sketch.stem
    staging.mkdir()
    staged = staging / sketch.name
    shutil.copy2(sketch, staged)

    cli = shutil.which("arduino-cli")
    if cli:
        commands = [
            [cli, "compile", "--fqbn", fqbn, str(staging)],
            [cli, "upload", "-p", port, "--fqbn", fqbn, str(staging)],
        ]
    elif IDE_EXE.exists():
        # arduino_debug.exe is the console build of the 1.8 IDE; arduino.exe is
        # the GUI wrapper and swallows its output on Windows
        commands = [
            [str(IDE_EXE), "--upload", "--port", port, "--board", fqbn, str(staged)]
        ]
    else:
        sys.exit(
            "no uploader found: install arduino-cli, or the Arduino 1.8 IDE at "
            f"{IDE_EXE}"
        )

    print(f"building {sketch.name} and uploading to {port}", flush=True)

    try:
        for command in commands:
            result = subprocess.run(
                command, capture_output=True, text=True, errors="replace"
            )
            if result.returncode == 0:
                continue

            # the 1.8 IDE puts everything on stderr, nearly all of it log4j
            # startup chatter, so keep only the lines that say something
            output = "\n".join(
                line
                for stream in (result.stderr, result.stdout)
                for line in stream.splitlines()
                if line.strip() and not NOISE_RE.match(line)
            )

            print(output, file=sys.stderr)
            sys.exit(f"upload failed ({' '.join(command)})")
    finally:
        shutil.rmtree(staging.parent, ignore_errors=True)

    print("upload finished")


def read_serial(ser, stop, log):
    """Echo every line to the terminal, and to the log file if there is one."""
    while not stop.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            print(f"serial error: {exc}", file=sys.stderr)
            break

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        print(line, flush=True)
        if log:
            log.write(line + "\n")
            log.flush()

    stop.set()


def forward_stdin(ser, stop):
    """Send whatever is typed here to the Arduino as a command line."""
    for line in sys.stdin:
        if stop.is_set():
            return
        try:
            ser.write(line.encode())
        except serial.SerialException:
            return


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--port",
        help="serial port; by default the Arduino Mega is found by its USB ids",
    )
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--log", help="also append every line to this file")
    parser.add_argument(
        "--upload",
        action="store_true",
        help="build and flash the sketch before opening the port",
    )
    parser.add_argument(
        "--upload-only",
        action="store_true",
        help="build and flash, then exit without monitoring",
    )
    parser.add_argument(
        "--sketch",
        type=Path,
        default=Path(__file__).with_name("inverted_pendulum_control.ino"),
        help="sketch to upload",
    )
    parser.add_argument(
        "--fqbn",
        default="arduino:avr:mega:cpu=atmega2560",
        help="board to build for",
    )
    args = parser.parse_args()
    args.port = find_mega_port(args.port)

    if args.upload or args.upload_only:
        upload_sketch(args.port, args.fqbn, args.sketch)
        if args.upload_only:
            return

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    print(f"reading {args.port} at {args.baud} baud, ctrl-c to stop")

    stop = threading.Event()
    log = open(args.log, "a", encoding="utf-8") if args.log else None

    reader = threading.Thread(target=read_serial, args=(ser, stop, log), daemon=True)
    reader.start()
    threading.Thread(target=forward_stdin, args=(ser, stop), daemon=True).start()

    try:
        # join with a timeout so Ctrl-C still gets through on Windows
        while reader.is_alive():
            reader.join(0.2)
    except KeyboardInterrupt:
        print("\nstopping", flush=True)
    finally:
        stop.set()
        ser.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
