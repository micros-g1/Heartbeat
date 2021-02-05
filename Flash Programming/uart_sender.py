import serial
import numpy
import os


def _wait_4_rdy(s, num):
    ready = False
    while not ready:
        try:
            response = s.readline().decode('ascii')
            if response == "OK\n":
                ready = True
                print('OK: ', num)

        except:
            pass


def send_file(filename):

    with serial.Serial('COM5', 115200, timeout=100) as s:
        respose = ""
        while respose != "Go\n":
            respose = s.readline().decode('ascii')
            print(respose)
    # bytes_per_transfer = 0x1000
    bytes_per_data = 2
    bytes_per_sector = 4096

    amount_bytes = os.path.getsize("data.bin")
    amount_bytes -= 1
    with open(filename, 'rb') as file:
        _ = file.read(1)

        amount_sectors = amount_bytes // bytes_per_sector
        if amount_bytes % bytes_per_sector == 0:
            augment = 0
        else:
            augment = 1
        amount_sectors += augment

        with serial.Serial('COM5', 115200, timeout=100) as s:

            s.write(amount_sectors.to_bytes(length=1, byteorder='little'))

            for sector in range(amount_sectors - augment):
                for _ in range(bytes_per_sector // bytes_per_data):
                    s.write(file.read(bytes_per_data))
                _wait_4_rdy(s, sector+1)

            # missing one (incomplete) sector?
            if augment == 1:
                # send bytes that we DO have...
                for _ in range((amount_bytes % bytes_per_sector) // bytes_per_data):
                    s.write(file.read(bytes_per_data))
                # send missing bytes as b'\x00'
                for _ in range(bytes_per_sector - amount_bytes % bytes_per_sector):
                    for i in range(bytes_per_data):
                        s.write(b'\x00')
                _wait_4_rdy(s, amount_sectors)