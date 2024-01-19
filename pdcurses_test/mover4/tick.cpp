#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <conio.h>
#include <windows.h>

static LARGE_INTEGER li;
static long Freq = 0, curTime = 0;

void tick_init(void){
	QueryPerformanceFrequency(&li);
	Freq = li.QuadPart;
	QueryPerformanceCounter(&li);
	curTime = li.QuadPart / (Freq / 1000);// convert to sec
}

long tick_get(void) {
	/* asses time */
	QueryPerformanceCounter(&li);
	curTime = li.QuadPart / (Freq / 1000);
	return curTime;
}

long tick_diff(long stamp) {
	 long diff;
	if (tick_get() >= stamp) diff = tick_get() - stamp;
	//else {
	//	diff = 0x100000000 + tick_get() - stamp;
	//}
	diff = tick_get() - stamp;
	return diff;
}