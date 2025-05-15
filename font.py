# python font.py 8x8font.png > CGAFont.h

import sys
from PIL import Image

img = Image.open(sys.argv[1]).convert('1')
w, h = img.size

rows = h // 8
cols = w // 8

data = []

for row in range(rows):
    for col in range(cols):

        char_data = []

        for y in range(8):
            byte = 0

            for x in range(8):
                if img.getpixel((x + col * 8, y + row * 8)) != 0:
                    byte |= 1 << x

            char_data.append(byte)

        data.append(char_data)


print('const uint8_t cgaFont[]\n{')
print(',\n'.join(['    ' + ', '.join([f'0x{b:02X}' for b in char]) for char in data]))
print('};')