# this one needs to be a multiple of 8
bitsString = "1111111101111111001111110001111100001111000001110000001100000001"
# store here the hex values
bhex = list()
# check the size of bitsString
if len(bitsString) % 8 != 0 : print("Invalid string len provided")
# split, convert to 8 and append to a string (make this more pythonable)
for i in range(0, len(bitsString), 8):
    bhex.append(hex(int(bitsString[i:i+8], 2)))
print(bhex)
# move to bytes
for i in range(len(bhex)):
    c = bytearray(bhex[i],encoding='utf8')
    print(c, end='\t')
    print(bin(int(bhex[i], 16))[2:].zfill(8))