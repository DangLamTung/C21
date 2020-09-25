# vim: set ai et ts=4 sw=4:

import cv2

img = cv2.imread("flappy_bird.png")
img = cv2.resize(img,(128,128))

f= open("image.h","w+")
f.write("const uint16_t test_img_128x128[][128] = {")

for y in range(0, img.shape[0]):
    s = "{"
    for x in range(0, img.shape[1]):
        (r, g, b) = img[x,y,:]
        color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3)
        # for right endiness, so ST7735_DrawImage would work
        color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8)
        s += "0x{:04X},".format(color565)
    s += "},"
    f.write(s)

f.write("};")
f.close()