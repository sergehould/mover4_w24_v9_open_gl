#ifndef TICK_H
#define	TICK_H
#include <stdint.h>

void tick_init(void);
long tick_get(void);
long tick_diff(long stamp);


#endif

