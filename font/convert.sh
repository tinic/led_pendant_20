convert -crop 2048x8+0+8 -rotate 90 -alpha off -depth 1 duck_font_grafx2.gif duck_font_%d.png
convert duck_font_1.png -depth 1 -colors 2 gray:duck_font.raw
xxd > ../duck_font.h -i duck_font.raw
rm duck_font.raw duck_font_0.png duck_font_1.png