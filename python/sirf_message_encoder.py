pack = lambda x: chr(x>>8)+chr(x&0xff)
start = "\xa0\xa2"
end = "\xb0\xb3"
payload = "\x81\x02\x01\x01\x00\x01\x01\x01\x05\x01\x01\x01\x01\x01\x01\x01\x00\x01\x00\x01\x00\x00\x25\x80"
#                   GGA     GLL     GSA     GSV     RMC     VTG     MSS     EPE     ZDA     NA      9600
checksum = sum(map(ord, list(payload))) & 0x7fff
length = len(payload)
message = start + pack(length) + payload + pack(checksum) + end
print(''.join(hex(ord(x)) + "," for x in message))
