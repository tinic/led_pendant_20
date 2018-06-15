
#include <stdio.h>
#include <stdint.h>

float easeOut(float t,float b , float c, float d) {
	if ((t/=d) < (1/2.75f)) {
		return c*(7.5625f*t*t) + b;
	} else if (t < (2/2.75f)) {
		float postFix = t-=(1.5f/2.75f);
		return c*(7.5625f*(postFix)*t + .75f) + b;
	} else if (t < (2.5/2.75)) {
			float postFix = t-=(2.25f/2.75f);
		return c*(7.5625f*(postFix)*t + .9375f) + b;
	} else {
		float postFix = t-=(2.625f/2.75f);
		return c*(7.5625f*(postFix)*t + .984375f) + b;
	}
}

float easeIn (float t,float b , float c, float d) {
	return c - easeOut (d-t, 0, c, d) + b;
}

int main() {
	int32_t c=0; 
	for (uint32_t c=0; c<64; c++) {
//		printf("%g\n", easeOut(t,0.0f,1.0f,1.0f));
		if ((c&7) == 0) printf("\n");
		printf("0x%02x, ", 31-int32_t(easeOut(float(c)/63.0f,0.0f,1.0f,1.0f)*31.0f));
	}

};
