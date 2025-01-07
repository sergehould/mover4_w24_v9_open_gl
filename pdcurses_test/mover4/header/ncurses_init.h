 /****************************************************************************
 *	ncurses_init.h
 * 	Contains useful public functions for the ncurses library
 *
 *	Author				Date			Version
 *	Serge Hould			27 Dec. 2020	1.0.0
 *
 *****************************************************************************/

#ifndef	NCURSE_INIT_H
#define NCURSE_INIT_H


#define	WHITE_BLACK		COLOR_PAIR(1)
#define	RED_BLACK		COLOR_PAIR(2)
#define	GREEN_BLACK		COLOR_PAIR(3)
#define	BLUE_BLACK		COLOR_PAIR(4)
#define	YELLOW_BLACK	COLOR_PAIR(5)
#define	MAGENTA_BLACK	COLOR_PAIR(6)
#define	CYAN_BLACK		COLOR_PAIR(7)
#define	BLACK_WHITE		COLOR_PAIR(8)
#define	RED_WHITE		COLOR_PAIR(9)
#define	GREEN_WHITE		COLOR_PAIR(10)
#define	BLUE_WHITE		COLOR_PAIR(11)
#define YELLOW_WHITE	COLOR_PAIR(12)
#define	MAGENTA_WHITE	COLOR_PAIR(13)
#define	CYAN_WHITE		COLOR_PAIR(14)
#define	BLACK_YELLOW	COLOR_PAIR(15)
#define	RED_YELLOW		COLOR_PAIR(16)
#define	GREEN_YELLOW	COLOR_PAIR(17)
#define	BLUE_YELLOW		COLOR_PAIR(18)
#define	WHITE_YELLOW	COLOR_PAIR(19)
#define	MAGENTA_YELLOW	COLOR_PAIR(20)
#define	CYAN_YELLOW		COLOR_PAIR(21)


/*Prototype Area*/
void ncurses_init(void);
int width_get(void);
int height_get(void);
void screen_init(void);

#endif
