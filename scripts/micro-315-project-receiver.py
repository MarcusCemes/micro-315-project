from sys import argv
from threading import Thread
from time import sleep

from serial import Serial, SerialException

# Global variable to signal the serial port
# thread to return after the blocking call
# has timed out
stop = False

READ_TIMEOUT = 2
INDEFINITE_TIME = 3600


def process_data(data: str):
    print(data)


def read_from_port(port: Serial):
    global stop

    while True:
        data = port.readline()

        if stop == True:
            port.close()
            print("Serial port closed")
            return

        if len(data) > 0:
            process_data(data)


def main():
    global stop

    if len(argv) < 2:
        print(f"Usage: {argv[0]} <com_port>")
        return

    print("\nConnecting to the e-puck...")

    try:
        com_port = argv[1]
        port = Serial(com_port, timeout=READ_TIMEOUT)
        print(f"Connected on {port.name}!")
    except SerialException:
        print("Could not connect to the e-puck2 robot.")
        print("Maybe try a different COM port?")
        return

    thread = Thread(target=read_from_port, args=(port,))
    thread.start()

    try:
        while True:
            # Sleep is able to receive KeyboardInterrupt on Windows
            sleep(INDEFINITE_TIME)
    except KeyboardInterrupt:
        print("Closing the serial port...")
        stop = True


main()
