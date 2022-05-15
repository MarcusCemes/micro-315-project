from math import asin, atan, pi, sqrt
from statistics import correlation
from matplotlib import animation
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
from msgpack import unpackb, UnpackValueError
import numpy as np
from rich.console import Console
from serial import Serial, SerialException
from scipy import correlate
from sys import argv, stdout
from threading import Lock, Thread
from time import sleep


INDEFINITE_TIME = 3600
UPDATE_INTERVAL = 20

# Constant character values used in the protocol
SFD = b'\x7e'  # ~
ETX = b'\x03'  # ETX
ESC = b'\x7d'  # }
XOR = b'\x20'  # SPACE

debug = "--debug" in argv

# Terminal conssole
console = Console()

# GUI plots
fig, axs = plt.subplots(5)

# Queued plot updates
updates = []

# Thread synchronisation
updates_lock = Lock()


class CancelledException(Exception):
    pass


class CancellationToken:
    def __init__(self) -> None:
        self.is_cancelled = False

    def cancel(self) -> None:
        self.is_cancelled = True

    def assert_ok(self) -> None:
        if self.is_cancelled:
            raise CancelledException()


def process_pcm(data: bytes) -> None:
    pcm = np.frombuffer(data, dtype=np.float32)

    pcms = np.split(pcm, 4)
    ffts = np.fft.fft(pcms)
    mags = np.abs(ffts)

    f_peak_index = np.argmax(mags[2])
    peaks = mags[:, f_peak_index]
    peaks *= [1.0, 1.02, 1.0, 0.95]  # Calibration
    peak_index = np.argmax(peaks)
    peak_args = np.angle(ffts[:, f_peak_index])

    align_lr = peak_args[0] - peak_args[1]
    align_fr = peak_args[0] - peak_args[3]
    align_fl = peak_args[1] - peak_args[3]
    align_fb = peak_args[2] - peak_args[3]
    align_f_lr = align_fr - align_fl

    if abs(align_lr) < 0.2:
        align_lr_text = "[green]"
    else:
        align_lr_text = "[red]"
    align_lr_text += "LR aligned: {:5.2f}".format(align_lr)

    if abs(align_f_lr) < 0.2:
        align_f_lr_text = "[green]"
    else:
        align_f_lr_text = "[red]"
    align_f_lr_text += "FL/FR aligned: {:5.2f}".format(align_f_lr)

    if align_fb < 0.7:
        align_fb_text = "[green]"
    else:
        align_fb_text = "[red]"
    align_fb_text += "FB aligned: {:5.2f}".format(align_fb)

    if peak_index == 3:
        peak_text = "[green]"
    else:
        peak_text = "[red]"
    peak_text += "Front"

    if peaks[0] > peaks[1]:
        peak_bias = "[green]R"
    else:
        peak_bias = "[red]L"

    console.log(
        f"{align_lr_text}  {align_f_lr_text}  {align_fb_text}  {peak_text}  {peak_bias}")

    updates.append((0, "PCM (left)", pcms[1]))
    updates.append((1, "PCM (right)", pcms[0]))
    updates.append((2, "FFT (front)", mags[3]))
    updates.append((3, "FFT (back)", mags[2]))


def process_pcm_back(data: bytes) -> None:
    pcm = np.frombuffer(data, dtype=np.float32)
    fft = np.fft.fft(pcm)
    mag = np.abs(fft)
    pha = np.angle(fft)

    updates.append((0, "PCM (back)", pcm))
    updates.append((1, "FFT magnitude (back)", mag))
    updates.append((2, "FFT phase (back)", pha))


def process_phases(data: bytes):
    try:
        phases = np.frombuffer(data, dtype=np.float32)
        assert(np.size(phases) == 4)

        right = phases[0]
        left = phases[1]
        back = phases[2]
        front = phases[3]

        console.log(f"Means: {right} {left} {back} {front}")

        freq = 1000.0
        c = 343.0
        delta_t = right / (freq * 2 * pi)
        delta_x = c * delta_t
        theta = pi/2 - atan(29/24) - asin(delta_x /
                                          sqrt(0.024 * 0.024 + 0.029 * 0.029))
        console.log("[orange bold]angle ", np.rad2deg(theta))

    except ValueError:
        console.log("[red]Could not calculate angle!")
        console.log(f"[yellow]Phase is {right}")


def decode_data(data: bytes) -> bytes:
    decoded_data = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESC[0]:
            decoded_data.append(data[i + 1] ^ XOR[0])
            i += 2
        else:
            decoded_data.append(data[i])
            i += 1
    return bytes(decoded_data)


def limit_data(data: bytes) -> bytes | str:
    if len(data) > 100:
        return f"{data[:100]}..."
    return data


def process_data(encoded_data: str):
    try:
        decoded_data = decode_data(encoded_data)
        unpacked_data = unpackb(decoded_data)

    except UnpackValueError as error:
        console.log("[red]Could not unpack data!")
        console.log("[yellow]The e-puck2 device may need to be restarted")
        console.log(f"[yellow]{error}")
        if debug:
            console.log(decoded_data)
        return

    try:
        updates_lock.acquire()
        type, data = unpacked_data

        if type == "PCM":
            process_pcm(data)
        elif type == "PCM_BACK":
            process_pcm_back(data)
        else:
            console.log(f"{type}: {limit_data(data)}")

    finally:
        updates_lock.release()


def read_from_port(port: Serial, token: CancellationToken):
    """
    Read data messages from the e-puck2, adhering to a custom protocol.
    The read call is blocking and problematic on Windows, requiring
    frequent checks to the cancellation token.
    """

    read_raw = "--raw" in argv
    if read_raw:
        console.log("[yellow]--raw is set, reading bytes only")

    while True:
        try:
            if read_raw:
                byte = port.read()
                token.assert_ok()
                if byte:
                    console.log(f"{hex(byte[0]).ljust(5)} {byte}")
                    stdout.flush()
                continue

            port.read_until(SFD)
            token.assert_ok()
            if debug:
                console.log("[cyan]Start of transmission")

            data = bytearray()
            size_bytes = port.read(3)
            size = int.from_bytes(size_bytes, byteorder="big")
            token.assert_ok()
            if debug:
                console.log(f"[cyan]Size hint: {size}")

            data.extend(port.read(size))
            token.assert_ok()

            data.extend(port.read_until(ETX)[:-1])
            token.assert_ok()
            if debug:
                console.log("[cyan]End of transmission")

            process_data(data)

        except (CancelledException, KeyboardInterrupt):
            console.log("[cyan]Closing serial port...")
            port.close()
            return


def rerender(_, token: CancellationToken):
    """
    Re-render the plots if new data has been received.
    """

    try:
        updates_lock.acquire()
        if len(updates) == 0:
            return

        for (index, title, data) in updates:
            ax: Axes = axs[index]

            lines = ax.get_lines()
            if len(lines) > 0:
                line = lines[0]
                x = np.linspace(0, len(data), len(data))
                line.set_xdata(x)
                line.set_ydata(data)
                ax.set_title(title)
                ax.set_ylim(np.min(data), np.max(data))
            else:
                ax.title.set_text(title)
                x = np.linspace(0, len(data), len(data))
                ax.plot(x, data)

        updates.clear()

    except KeyboardInterrupt:
        token.cancel()
        plt.close()

    finally:
        updates_lock.release()


def main():

    if len(argv) < 2:
        console.print(f"Usage: {argv[0]} <com_port>")
        return

    # Try to connect to the e-puck2 device
    console.print("")
    with console.status("Connecting to the e-puck...", spinner_style="cyan"):
        try:
            com_port = argv[1]
            port = Serial(com_port)
            console.log("[green]✅ Connected to the e-puck device")
        except SerialException:
            console.log("[red]‼️ Could not connect to the e-puck!")
            console.log("🤚 Perhaps try a different COM port?\n")
            return

    # Start a background thread to receive data
    token = CancellationToken()
    thread = Thread(target=read_from_port, args=(port, token))
    thread.start()

    # Run the matplotlib event loop and update with new data if necessary
    try:
        if not "--raw" in argv and not "--no-gui" in argv:
            _anim = animation.FuncAnimation(
                fig, rerender, fargs=(token,), interval=UPDATE_INTERVAL)
            plt.tight_layout()
            plt.show()
            token.assert_ok()
        while True:
            sleep(INDEFINITE_TIME)  # Sleep can be interrupted on Windows
    except (KeyboardInterrupt, CancelledException):
        console.log("[cyan]Interrupt detected!")
        console.log("[cyan]Waiting for communication thread to terminate...")

        token.cancel()
        port.cancel_read()
        port.cancel_write()

    thread.join()
    console.log("[cyan]Done\n")


if __name__ == '__main__':
    main()
