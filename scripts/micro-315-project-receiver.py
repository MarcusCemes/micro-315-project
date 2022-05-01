from sys import argv
from threading import Thread
from time import sleep

from msgpack import unpackb
from rich.console import Console
from serial import Serial, SerialException

INDEFINITE_TIME = 3600
SFD = b'\x7e'  # ~
ETX = b'\x03'  # ETX
ESC = b'\x7d'  # }
XOR = b'\x20'  # SPACE

console = Console()


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
    data = unpackb(decode_data(encoded_data))
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

    while True:
        try:
            port.read_until(SFD)
            token.assert_ok()
            console.log("[cyan]TRANSMISSION START")

            data = bytearray()
            size_bytes = port.read(3)
            size = int.from_bytes(size_bytes, byteorder="big")
            token.assert_ok()

            console.log(f"[cyan]SIZE HINT {size}")
            data.extend(port.read(size))
            token.assert_ok()

            data.extend(port.read_until(ETX)[:-1])
            token.assert_ok()

            console.log("[cyan]TRANSMISSION END")
            process_data(data)

        except CancelledException:
            console.log("[cyan]Closing serial port...")
            port.close()
            return


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
