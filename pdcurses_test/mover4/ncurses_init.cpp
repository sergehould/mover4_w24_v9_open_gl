/****************************************************************************
*	ncurses_init.c
* 	Contains definitions of useful  ncurses library functions
*
*	Author				Date			Version
*	Serge Hould			27 Dec. 2020	1.0.0
*	Serge Hould			May 2023		1.1.0	Add height_get(), width_get() and screen_init()
*
*****************************************************************************/
 
 #ifdef _WIN32
#include <Windows.h>

#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#endif

#include "header/public.h"
#include "header/ncurses_init.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>

/* Initializes Ncurses*/
void ncurses_init(void) {
	/*Ncurse config */
	initscr();			/* Start curses mode 		  */
	//use_default_colors();
	//start_color();
	keypad(stdscr, TRUE);  // enable keyboard mapping
	nonl();         // tell curses not to do NL->CR/NL on output
	cbreak();       // take input chars one at a time, no wait for \n ; necessary for moving the robot
	noecho();        // do not echo input
	start_color();
	init_pair(1, COLOR_WHITE, COLOR_BLACK);
	init_pair(2, COLOR_RED, COLOR_BLACK);
	init_pair(3, COLOR_GREEN, COLOR_BLACK);
	init_pair(4, COLOR_BLUE, COLOR_BLACK);
	init_pair(5, COLOR_YELLOW, COLOR_BLACK);
	init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
	init_pair(7, COLOR_CYAN, COLOR_BLACK);

	init_pair(8, COLOR_BLACK, COLOR_WHITE);
	init_pair(9, COLOR_RED, COLOR_WHITE);
	init_pair(10, COLOR_GREEN, COLOR_WHITE);
	init_pair(11, COLOR_BLUE, COLOR_WHITE);
	init_pair(12, COLOR_YELLOW, COLOR_WHITE);
	init_pair(13, COLOR_MAGENTA, COLOR_WHITE);
	init_pair(14, COLOR_CYAN, COLOR_WHITE);

	init_pair(15, COLOR_BLACK, COLOR_YELLOW);
	init_pair(16, COLOR_RED, COLOR_YELLOW);
	init_pair(17, COLOR_GREEN, COLOR_YELLOW);
	init_pair(18, COLOR_BLUE, COLOR_YELLOW);
	init_pair(19, COLOR_WHITE, COLOR_YELLOW);
	init_pair(20, COLOR_MAGENTA, COLOR_YELLOW);
	init_pair(21, COLOR_CYAN, COLOR_YELLOW);
	//attron(COLOR_PAIR(3)); // BLUE
}

/* Returns the width of the screen */
int width_get(void) {
	/*keyboard*/
	initscr();
	WINDOW* stdstr;

	int width, height;
	CONSOLE_SCREEN_BUFFER_INFOEX consolesize;
	consolesize.cbSize = sizeof(consolesize);
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfoEx(hConsole, &consolesize);
	auto largest_size{ GetLargestConsoleWindowSize(hConsole) };
	return largest_size.X - 1;
}

/* Returns the heigth of the screen */
int height_get(void) {
	/*keyboard*/
	initscr();
	WINDOW* stdstr;

	int width, height;
	CONSOLE_SCREEN_BUFFER_INFOEX consolesize;
	consolesize.cbSize = sizeof(consolesize);
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfoEx(hConsole, &consolesize);
	auto largest_size{ GetLargestConsoleWindowSize(hConsole) };
	return largest_size.Y - 1;
}

//#define FULL_SCREEN

/* Initializes the screen to its maximum size*/
void screen_init(void) {
	initscr();
	WINDOW* stdstr;

	system("mode 650");
	int width, height;
	CONSOLE_SCREEN_BUFFER_INFOEX consolesize;
	consolesize.cbSize = sizeof(consolesize);
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfoEx(hConsole, &consolesize);
	auto largest_size{ GetLargestConsoleWindowSize(hConsole) };
	if (!largest_size.X && !largest_size.Y) {
		//std::cerr << "GetLargestConsoleWindowSize() failed with \"" << get_last_error_msg() << "\" :(\n\n";
		//return false;
	}


	--largest_size.X;
	--largest_size.Y;

	COORD c;
	c.X = largest_size.X;
	c.Y = largest_size.Y;
	////c.X = width;
	////c.Y = height;
	consolesize.dwSize = c;
	width = largest_size.X;;
	height = largest_size.Y;

	//SetConsoleScreenBufferInfoEx(hConsole, &consolesize);
	SetConsoleScreenBufferSize(hConsole, largest_size);
	//SetConsoleScreenBufferSize(hConsole, c);
	//SetConsoleDisplayMode(hConsole, CONSOLE_FULLSCREEN_MODE, &c);
	//keypad(stdstr, TRUE);


}