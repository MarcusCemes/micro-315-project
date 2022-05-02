from matplotlib import animation
import matplotlib.pyplot as plt
from msgpack import unpackb, UnpackValueError
import numpy as np
from rich.console import Console
from serial import Serial, SerialException
from sys import argv, stdout
from threading import Thread
from time import sleep


INDEFINITE_TIME = 3600
SFD = b'\x7e'  # ~
ETX = b'\x03'  # ETX
ESC = b'\x7d'  # }
XOR = b'\x20'  # SPACE

console = Console()

fig, axx = plt.subplots(5)
plot_needs_update = False
plot_pcm = None
plot_fft_re = None
plot_fft_im = None
plot_mag_pha_re = None
plot_mag_pha_im = None


class CancelledException(Exception):
    pass


class CancellationToken:
    def __init__(self):
        self.is_cancelled = False

    def cancel(self):
        self.is_cancelled = True

    def assert_ok(self):
        if self.is_cancelled:
            raise CancelledException()


class ProtocolException(Exception):
    pass


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


def process_data(encoded_data: str):
    try:
        decoded_data = decode_data(encoded_data)
        data = unpackb(decoded_data)
    except UnpackValueError as error:
        console.log("[red]Could not unpack data")
        console.log(f"[red]{error}")
        console.log(decoded_data)
        return

    global plot_needs_update, plot_pcm, plot_fft_re, plot_fft_im, plot_mag_pha_re, plot_mag_pha_im

    if data[0] == "PCM":
        plot_pcm = np.frombuffer(data[1], dtype=np.float32)[::2]
        plot_needs_update = True
    elif data[0] == "FFT":
        re_im = np.frombuffer(data[1], dtype=np.float32)
        plot_fft_re = re_im[::2]
        plot_fft_im = re_im[1::2]
        plot_needs_update = True
    elif data[0] == "MAG/PHA":
        re_im = np.frombuffer(data[1], dtype=np.float32)
        plot_mag_pha_re = re_im[::2]
        plot_mag_pha_im = re_im[1::2]
        plot_needs_update = True
    else:
        console.log(data)


# def warn_protocol_error(byte: bytes) -> None:
#     console.log(
#         f"[bold red]Protocol error, unexpected byte received: {byte}")
#     console.log("[bold red]Perhaps the device reset?")
#     console.log("[cyan]Waiting for new SDF byte...")


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
            console.log("[cyan]TRANSMISSION START")

            data = bytearray()
            size_bytes = port.read(3)
            size = int.from_bytes(size_bytes, byteorder="big")
            token.assert_ok()

            console.log(f"[cyan]SIZE HINT {size}")
            console.log("Reading up to size...")
            data.extend(port.read(size))
            token.assert_ok()

            console.log("Reading until EXT...")
            data.extend(port.read_until(ETX)[:-1])
            token.assert_ok()

            console.log("[cyan]TRANSMISSION END")
            process_data(data)

        except CancelledException:
            console.log("[cyan]Closing serial port...")
            port.close()
            return


# update the plots
def update_plot(_args):
    global plot_needs_update

    if not plot_needs_update:
        return

    plot_needs_update = False

    if plot_pcm is not None:
        axx[0].clear()
        axx[0].title.set_text("PCM data")

        axx[0].plot(np.linspace(0, 1, len(plot_pcm)), plot_pcm)

    if plot_fft_re is not None:
        axx[1].clear()
        axx[1].title.set_text("FFT real/imaginary")
        axx[1].plot(np.linspace(0, 1, len(plot_fft_re)), plot_fft_re)

    if plot_fft_im is not None:
        axx[2].clear()
        axx[2].plot(np.linspace(0, 1, len(plot_fft_im)), plot_fft_im)

    if plot_mag_pha_re is not None:
        axx[3].clear()
        axx[3].title.set_text("Magnitude/Phase")
        axx[3].plot(np.linspace(
            0, 1, len(plot_mag_pha_re)), plot_mag_pha_re)

    if plot_mag_pha_im is not None:
        axx[4].clear()
        axx[4].plot(np.linspace(
            0, 1, len(plot_mag_pha_im)), plot_mag_pha_im)


def main():

    if len(argv) < 2:
        console.print(f"Usage: {argv[0]} <com_port>")
        return

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

    token = CancellationToken()
    thread = Thread(target=read_from_port, args=(port, token))
    thread.start()

    try:
        while True:
            _anim = animation.FuncAnimation(fig, update_plot, interval=200)
            plt.show()
            sleep(INDEFINITE_TIME)  # Sleep can be interrupted on Windows
    except KeyboardInterrupt:
        console.log("[cyan]Interrupt detected!")
        console.log("[cyan]Waiting for communication thread to terminate...")

        token.cancel()
        port.cancel_read()
        port.cancel_write()

    thread.join()
    console.log("[cyan]Done\n")


if __name__ == '__main__':
    main()
