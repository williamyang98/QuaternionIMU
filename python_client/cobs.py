# Source: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
# COBS: consistent overhead byte stuffing

# decode COBS encoded packets
# delimited by zero byte
def cobs_decode(encode):
    encode_i = 0

    code = 0xff
    block = 0
    
    decode = []

    while encode_i < len(encode):
        if block:
            decode.append(encode[encode_i])
            encode_i += 1
        else:
            # encoded 0x00
            if code != 0xff:
                decode.append(0x00)

            block = encode[encode_i]
            code = encode[encode_i]
            encode_i += 1
            # delimiter
            if code == 0x00:
                break

        block -= 1
    
    return decode 

# encode an arbitary packet which could contain a zero byte
# into a COBS encoded zero byte encoded packet
# all our original zero data bytes are replaced with non-zero encoded bytes
def cobs_encode(data):
    N = len(data)

    encode = [0x00 for _ in range(N + N//254 + 1)]

    data_i = 0

    code_i = 0 
    encode_i = 1
    code = 0x01

    while N > 0:
        N -= 1

        if data[data_i] != 0x00:
            encode[encode_i] = data[data_i]
            encode_i += 1
            code += 1
        
        if data[data_i] == 0x00 or code == 0xFF:
            encode[code_i] = code
            code = 0x01
            code_i = encode_i 
            if data[data_i] == 0x00 or N > 0:
                encode_i += 1

        data_i += 1
    
    encode[code_i] = code
    encode = encode[:encode_i]
    encode.append(0x00)

    return encode

if __name__ == '__main__':
    print(cobs_decode([0x00]))

    # def print_test(input):
    #     output = cobs_encode(input)
    #     print(' '.join((f'{o:02x}' for o in output)))

    # print_test([0x00])
    # print_test([0x00, 0x00])
    # print_test([0x11,0x22,0x00,0x33])
    # print_test([0x11,0x22,0x33,0x44])
    # print_test([0x11,0x00,0x00,0x00])
