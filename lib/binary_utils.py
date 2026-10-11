import binascii

def hex_from_bytes(byte_string, spaced=False):
    """Return the hexadecimal representation of the binary data. Every byte of
    data is converted into the corresponding 2-digit hex representation. The
    resulting string is therefore twice as long as the length of data.
    """
    hex_string = binascii.hexlify(byte_string).decode('ascii')

    if spaced:
        hex_string = ' '.join([hex_string[i:i+2] for i in range(0, len(hex_string), 2)])

    return hex_string

def bytes_from_hex(hex_string, spaced=False):
    """Return the binary data represented by the hexadecimal string. Every 2-digit hex representation
    is converted into the corresponding byte of data. The resulting byte string is therefore half as long as the hex string.
    """
    if spaced:
        hex_string = ''.join(hex_string.split(' '))

    byte_string = binascii.unhexlify(hex_string)

    return byte_string

def shift_bytes(data, shift=1):
    return data[-shift:] + data[:-shift]


if __name__ == '__main__':
    # # EXAMPLE DATA
    hex_list = ['10', '0e', '53', '2b', '04', '01', 'ec', '13',
                '01', 'ea', '1a', '01', 'ea', '1e', '01', 'ea', '26', '01',
                'e9', '31', '01', 'e8', '29', '01', 'eb', '2c', '01', 'e9',
                '2d', '01', 'eb', '34', '01', 'e8', '38', '01', 'e5', '4e',
                '01', 'd9', 'e4', '2e', '57', '0b', 'd3', '54', '2c']
    #len(hex_list) = 47

    hex_string_spaced = ' '.join(hex_list)
    byte_string = bytes_from_hex(hex_string_spaced, spaced=True)
    print("\nbyte_string:\n", byte_string, "\n")
    # byte_string = b'T,\x10\x0eS+\x04\x01\xec\x13\x01\xea\x1a\x01\xea\x1e\x01\xea&\x01\xe91\x01\xe8)\x01\xeb,\x01\xe9-\x01\xeb4\x01\xe88\x01\xe5N\x01\xd9\xe4.W\x0b\xd3'

    hex_string_spaced = hex_from_bytes(byte_string, spaced=True)
    hex_list = hex_string_spaced.split(' ')
    print("hex_list:\n", hex_list, "\n")
