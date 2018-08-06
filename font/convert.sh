#!/bin/sh
convert -crop 2048x8+0+8 -rotate 90 -alpha off -depth 1 duck_font_grafx2.gif duck_font_1_%d.png
convert -crop 2048x8+0+16 -rotate 90 -alpha off -depth 1 duck_font_grafx2.gif duck_font_2_%d.png
convert -crop 2048x8+0+24 -rotate 90 -alpha off -depth 1 duck_font_grafx2.gif duck_font_3_%d.png
convert -crop 2048x8+0+32 -rotate 90 -alpha off -depth 1 duck_font_grafx2.gif duck_font_4_%d.png
montage duck_font_1_1.png duck_font_2_1.png duck_font_3_1.png duck_font_4_1.png -geometry 8x2048+0+0 -tile 1x4 duck_font.png
convert duck_font.png -depth 1 -colors 2 gray:duck_font.raw
xxd > ../duck_font.h -i duck_font.raw
rm duck_font.raw duck_font_1_0.png duck_font_1_1.png duck_font_2_0.png duck_font_2_1.png duck_font_3_0.png duck_font_3_1.png duck_font_4_0.png duck_font_4_1.png duck_font.png
