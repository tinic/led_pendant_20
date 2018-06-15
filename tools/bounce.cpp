
#include <stdio.h>
#include <stdint.h>

float bounceEaseOut(float t,float b , float c, float d) {
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

float bounceEaseIn (float t,float b , float c, float d) {
	return c - bounceEaseOut (d-t, 0, c, d) + b;
}

float quadEaseIn (float t,float b , float c, float d) {
	return c*(t/=d)*t + b;
}
float quadEaseOut(float t,float b , float c, float d) {
	return -c *(t/=d)*(t-2) + b;
}

int main() {
	int32_t c=0; 

#if 0
	for (uint32_t c=0; c<64; c++) {
		if ((c&7) == 0) printf("\n");
		printf("0x%02x, ", 31-int32_t(bounceEaseOut(float(c)/63.0f,0.0f,1.0f,1.0f)*31.0f));
	}
#endif //#if 0

#if 1
	for (uint32_t c=0; c<32; c++) {
		if ((c&7) == 0) printf("\n");
		printf("0x%02x, ", int32_t(quadEaseIn(float(c)/31.0f,0.0f,1.0f,1.0f)*31.0f));
	}
#endif  // #if 0
};
