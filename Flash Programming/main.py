# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import random
from vals_2_write import values_2_write, dummy
from uart_sender import send_file
import serial

def array_to_int(values, bitwidth):
    mask = 2**bitwidth - 1
    shift = bitwidth * (len(values)-1)
    integer = 0
    for value in values:
        integer |= (value & mask) << shift
        shift -= bitwidth
    return integer


def run_bin_writer(vals, bitwidth, filename):

    # just for debug/information: bit string representation
    bitstring = "".join(map(lambda x: bin(x)[2:].zfill(bitwidth), vals))
    print(bitstring)

    bigint = array_to_int(vals, bitwidth)
    width = bitwidth * len(vals)
    print('{:0{width}b}'.format(bigint, width=width))  # show integer's value in binary

    num_bytes = (width+8 - (width % 8)) // 8  # round to whole number of 8-bit bytes
    with open(filename, 'wb') as file:
        file.write(bigint.to_bytes(num_bytes, byteorder='big'))


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    values = values_2_write
    # values = dummy
    # values = [0xAA for _ in range(4096*2)]  # create fixed pattern for debugging
    # values = [0b000001 for _ in range(10)]  # create fixed pattern for debugging
    # values[9] = 0b011111  # make last one different so it can be spotted
    bit_w = 8
    f_name = 'data.bin'
    # print(len(values))
    # print(values[0:4096])
    # print(values[4096:])
    run_bin_writer(vals=values, bitwidth=bit_w, filename=f_name)
    send_file(filename=f_name)
