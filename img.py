from PIL import Image

img = Image.open("panther.png").convert("1")  # 1-bit monochrome
width, height = img.size
data = []

for y in range(0, height, 8):
    for x in range(width):
        byte = 0
        for bit in range(8):
            if y + bit < height:
                pixel = img.getpixel((x, y + bit))
                if pixel == 0:  # black pixel
                    byte |= (1 << bit)
        data.append(byte)

print(data)  # This can go directly into your `uint8_t buffer[]`
