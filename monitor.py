"""Flash testing_setup.ino and/or watch its serial output, optionally with new LQR gains.

Usage:
    python monitor.py                 # finds the Mega, 115200 baud
    python monitor.py --port COM4     # or name the port yourself
    python monitor.py --upload        # flash the sketch first, then monitor
    python monitor.py --upload-only   # just flash it
    python monitor.py --log run1.txt  # log to run1.txt instead of serial_log.txt
    python monitor.py --no-log        # terminal only

    python monitor.py --q 8 0 0.5 0           # K from Q = diag(q), R = 1, sent on serial
    python monitor.py --q 8 0 0.5 0 --r 2     # ... with R = 2
    python monitor.py --q 8 0 0.5 0 --upload  # write K into the sketch and flash it

Gains sent on serial last until the board resets; --upload makes them the
sketch's defaults. Either way the sketch echoes "K = [...]" once applied.

Opening the port resets the Mega, so the sketch re-zeroes both encoders on
connect: let the pendulum hang at rest with the arm centred when starting this.

Everything the sketch prints is also appended to serial_log.txt, beside this
script, under a header stamped with the time each run starts.

Anything typed into this terminal is forwarded to the sketch. Ctrl-C to stop.

Needs: pip install pyserial (and numpy scipy for --q)
"""

import argparse
import datetime
import re
import shutil
import subprocess
import sys
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

# the Q comment and the four gain defines in the sketch, which --upload rewrites
GAINS_BLOCK_RE = re.compile(
    r"^// Q = diag\(.*\n(?:#define K[1-4] .*\n){4}", re.MULTILINE
)

# the line the sketch prints its gains on, at startup and after a "k" command
GAINS_LINE = "K = ["

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

    cli = shutil.which("arduino-cli")
    if cli:
        commands = [
            [cli, "compile", "--fqbn", fqbn, str(sketch.parent)],
            [cli, "upload", "-p", port, "--fqbn", fqbn, str(sketch.parent)],
        ]
    elif IDE_EXE.exists():
        # arduino_debug.exe is the console build of the 1.8 IDE; arduino.exe is
        # the GUI wrapper and swallows its output on Windows
        commands = [
            [str(IDE_EXE), "--upload", "--port", port, "--board", fqbn, str(sketch)]
        ]
    else:
        sys.exit(
            "no uploader found: install arduino-cli, or the Arduino 1.8 IDE at "
            f"{IDE_EXE}"
        )

    print(f"building {sketch.name} and uploading to {port}", flush=True)

    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, errors="replace")
        if result.returncode == 0:
            continue

        # the 1.8 IDE puts everything on stderr, nearly all of it log4j startup
        # chatter, so keep only the lines that say something about the build
        output = "\n".join(
            line
            for stream in (result.stderr, result.stdout)
            for line in stream.splitlines()
            if line.strip() and not NOISE_RE.match(line)
        )

        print(output, file=sys.stderr)
        sys.exit(f"upload failed ({' '.join(command)})")

    print("upload finished")


def compute_gains(q, r_weight):
    """K for Q = diag(q) and R, from the manual's model."""
    # imported here so plain monitoring doesn't need numpy and scipy
    from lqr_pendulum import closed_loop_poles, lqr_gain

    k = lqr_gain(q, r_weight)
    poles = ", ".join(f"{p:.2f}" for p in closed_loop_poles(k))
    print(
        f"Q = diag([{' '.join(f'{v:g}' for v in q)}]), R = {r_weight:g}\n"
        f"K = [{', '.join(f'{v:.4f}' for v in k)}]\n"
        f"closed-loop poles: {poles}"
    )
    return k


def write_gains(sketch, q, r_weight, k):
    """Rewrite the sketch's Q comment and K1..K4 defines in place."""
    text = sketch.read_text(encoding="utf-8")
    blocks = GAINS_BLOCK_RE.findall(text)
    if len(blocks) != 1:
        sys.exit(
            f"expected one '// Q = diag(...)' line followed by the four K defines "
            f"in {sketch}, found {len(blocks)}"
        )

    names = ("theta", "alpha", "theta_dot", "alpha_dot")
    block = f"// Q = diag([{' '.join(f'{v:g}' for v in q)}]), R = {r_weight:g}\n" + "".join(
        f"#define K{i + 1} {value:.4f} // {name}\n"
        for i, (value, name) in enumerate(zip(k, names))
    )
    with open(sketch, "w", encoding="utf-8", newline="") as file:
        file.write(GAINS_BLOCK_RE.sub(lambda _match: block, text))
    print(f"wrote K into {sketch.name}")


def send_gains(ser, k, ready):
    """Send K once the sketch has printed its banner, so it isn't lost in the reset."""
    # opening the port resets the Mega, and the bootloader would swallow early
    # bytes. The banner's gains line means the sketch is up; anything sent now
    # waits in its receive buffer until setup() finishes zeroing. A board that
    # didn't reset prints no banner, so stop waiting after a few seconds.
    if not ready.wait(4.0):
        print("(no startup banner seen, sending the gains anyway)", flush=True)

    command = "k " + " ".join(f"{value:.4f}" for value in k) + "\n"
    try:
        ser.write(command.encode())
    except serial.SerialException as exc:
        print(f"serial write failed: {exc}", file=sys.stderr)
        return
    print(f"sent: {command.strip()}", flush=True)


def read_serial(ser, stop, log, ready):
    """Echo every line to the terminal, and to the log file if there is one."""
    # Lines are split here rather than with ser.readline(): on this laptop
    # (x64 Python emulated on ARM64 Windows) pyserial's readline() returns the
    # same stale fragment thousands of times a second, while read() is fine.
    pending = b""
    while not stop.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as exc:
            print(f"serial error: {exc}", file=sys.stderr)
            break

        if not chunk:
            continue

        pending += chunk
        *complete, pending = pending.split(b"\n")
        for raw in complete:
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            print(line, flush=True)
            if line.startswith(GAINS_LINE):
                ready.set()
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
    parser.add_argument(
        "--log",
        type=Path,
        default=Path(__file__).with_name("serial_log.txt"),
        help="file every line is appended to (default: serial_log.txt here)",
    )
    parser.add_argument(
        "--no-log", action="store_true", help="print to the terminal only"
    )
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
        "--q",
        type=float,
        nargs=4,
        metavar="Q",
        help="diag(Q) for the LQR; K is computed and sent on serial, or written "
        "into the sketch with --upload",
    )
    parser.add_argument("--r", type=float, default=1.0, help="R weight (default 1)")
    parser.add_argument(
        "--sketch",
        type=Path,
        default=Path(__file__).parent / "testing_setup" / "testing_setup.ino",
        help="sketch to upload",
    )
    parser.add_argument(
        "--fqbn",
        default="arduino:avr:mega:cpu=atmega2560",
        help="board to build for",
    )
    args = parser.parse_args()

    uploading = args.upload or args.upload_only
    gains = compute_gains(args.q, args.r) if args.q else None
    if gains is not None and uploading:
        write_gains(args.sketch, args.q, args.r, gains)

    args.port = find_mega_port(args.port)

    if uploading:
        upload_sketch(args.port, args.fqbn, args.sketch)
        if args.upload_only:
            return

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    print(f"reading {args.port} at {args.baud} baud, ctrl-c to stop")

    stop = threading.Event()
    log = None
    if not args.no_log:
        log = open(args.log, "a", encoding="utf-8")
        # the command line says which gains this run was meant to have; the
        # sketch's own "K = [...]" line below it says which it actually got
        started = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        command = " ".join(["monitor.py", *sys.argv[1:]])
        log.write(f"\n===== {started}  {args.port}  {command} =====\n")
        log.flush()
        print(f"logging to {args.log}")

    ready = threading.Event()

    reader = threading.Thread(
        target=read_serial, args=(ser, stop, log, ready), daemon=True
    )
    reader.start()
    threading.Thread(target=forward_stdin, args=(ser, stop), daemon=True).start()

    # after an upload the gains are already the sketch's defaults
    if gains is not None and not uploading:
        threading.Thread(
            target=send_gains, args=(ser, gains, ready), daemon=True
        ).start()

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
