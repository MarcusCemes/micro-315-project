from sys import argv
from threading import Thread
from time import sleep

from msgpack import unpackb
from rich.console import Console
from serial import Serial, SerialException

INDEFINITE_TIME = 3600
SFD = b"\x7e"

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


def process_data(serialised_data: str):
    data = unpackb(serialised_data)
    console.log(data)


def read_from_port(port: Serial, token: CancellationToken):
    """
    Read data messages from the e-puck2, adhering to a custom protocol.
    The read call is blocking and problematic on Windows, requiring
    frequent checks to the cancellation token.
    """

    while True:
        try:
            # The initial data may contain random garbage, the first
            # read has to be handled differently.
            port.read_until(SFD)
            token.assert_ok()
            console.log("[bold green]Received initial SFD byte")

            size = int.from_bytes(port.read(2), byteorder="big")
            token.assert_ok()

            data = port.read(size)
            token.assert_ok()

            process_data(data)

            # Continue to read and process subsequent data frames
            while True:
                sfd = port.read()
                token.assert_ok()

                if sfd != SFD:
                    console.log(
                        "[bold red]Protocol error, unexpected byte received")
                    console.log("[bold red]Perhaps the device reset?")
                    console.log("[cyan]Waiting for new SDF byte...")
                    break

                size = int.from_bytes(port.read(2), byteorder="big")
                token.assert_ok()

                data = port.read(size)
                token.assert_ok()

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
