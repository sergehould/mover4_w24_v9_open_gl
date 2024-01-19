/*
*	Description: Controller for Mover4 robot arm.
*
*	Author				Date			Version
* SH					2 June	2023	v5.4.0	
*												
*		
*********************************************************************************************************/

#ifdef _WIN32
#include <Windows.h>
#endif
#include "header/task_controller.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <curses.h>
#include <stdint.h>
#include <ctype.h> 
#include <errno.h>
#include <sys/types.h>
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>

#include "header/config.h"


#if defined UDP
#include "header/udp.h"
#else 
#include "header/can.h"
#endif

/*	By defining LOCK it sets the workspace limits.
	Sets the angle's boundaries
	Sets the x-y-z's boundaries
*/
#define		LOCK


#define		PI	3.1416
#define		NUM_JOINTS	4

/* Defines the minimum distance to the floor for motor 3, motor 4 and the tip */
#define 	Z_TIP_MIN	2.5	// sets the minimum z position of the tip
#define   	Z_M3_MIN	2.5	// sets the minimum z position of the elbow motor
#define 	Z_M4_MIN	2.5	// sets the minimum z position of the wrist motor
/* Sets the maximum reach of the arm*/
#define 	R_MAX		32	//no limit
//#define 	R_MAX		24	//limited

/* link length in inches.  Must also be modified in kinematic.c and animation.c */
#define BASE_HGT 	8.625      //base height   
#define HUMERUS 	7.375      //shoulder-to-elbow "bone" 
#define ULNA		8.625      //elbow-to-wrist "bone" 
#define GRIPPER		6.0           //gripper
#ifdef _WIN32
#define DELAY_LOOP	10		// control loop delay in mS . The actual loop delay is 4* DELAY_LOOP. Must never exceed 100 mS - see set_all_sp_angles()

#else
#define DELAY_LOOP	12		// control loop delay in mS . The actual loop delay is 4* DELAY_LOOP
#endif

#define	WARNING_DELAY	150	// Warning display delay of about 3 seconds for DELAY_LOOP of 5mS
#define	SLOW_DEGREES	130 // number of tics where the speed slows down

#define	TRAJ_BUFFER_SIZE 20000

#ifdef LOCK
	/* sets the limits in lock mode */
	#define		BASE_MIN	-150
	#define		BASE_MAX	150
	#define		SHLD_MIN	-50
	#define		SHLD_MAX	65
	#define		ELB_MIN		-110
	#define		ELB_MAX		140
	#define		WRIST_MIN	-140
	#define		WRIST_MAX	140
#else
	/* Remove limits by setting to +/- 360 */
	#define		BASE_MIN	-360
	#define		BASE_MAX	360
	#define		SHLD_MIN	-360
	#define		SHLD_MAX	360
	#define		ELB_MIN		-360
	#define		ELB_MAX		360
	#define		WRIST_MIN	-360
	#define		WRIST_MAX	360
#endif

/* static local function prototypes */
static void set_velocity(int id, int vel);
static void set_max_lag(int low, int high);
static void maxMissedCom(int low, int high);
static void reset_error(void);
static int get_sp_tics_mem(int joint);
static void set_sp_tics_mem(int joint, int val);
//static void resetJointsToZero(void);
static void disable_motor(void);
static int enable_motors(void);
static double computeJointPos(int, int);
static int computeTics(int, double);
static kin_f to_cart(kin_f angle);
static void init_KinematicMover(void);
static double to_radians2(double degrees);
static void* pTask_Controller(void* ptr);
static void* pTask_Rx(void* ptr);
static kin_f to_3zs(kin_f angle);
static double to_r(kin_f angle);
static void set_error_f(int);
static int get_error_f(void);
#ifdef _WIN32
static int gettimeofday(struct timeval* p, void* tz);
#endif
static void set_sp_tics(int nb, int tics);
static int get_sp_tics(int joint);
static kin_i get_all_sp_tics(void);
static void set_all_sp_tics(kin_i tics);
static void set_control_mode(int j, int m);
static int get_control_mode(int n);
static void all_pv_angles_set(kin_f _angles);
static int check_sp_angle(int i, double _angle);
int traj_max_get();
void traj_max_set(int);
int traj_cnt_get();
void traj_cnt_inc(void);
void traj_cnt_clear(void);


//Global variables
static pthread_t thread_controller;
static pthread_t thread_Rx;
static kin_i sp_tics_mem;
static int jointIDs[4] = { 16,32,48,64 };	// CAN IDs of the robots joints
static kin_f sp_angles = { 0 };
static kin_f curr_angles;   // AKA pv angle
static int gripper = GRIP_OPEN;
static double gearScale[4];
static int reset_err = 0, en_motor_ = 0;
static int error_f = 0;
static FILE* fd_s;	// state file
static FILE* fp;	//log file
static char buf_w1[250] = { 0 };	//warning messages
static char buf_err[250] = { 0 };	//error messages
static char buf_temp[250];	// temporary buffer
static char buf_temp2[250];	// temporary buffer
static int adc_value = 0x1;
/* display flags*/
static  int		w1_f = 0, wb_f = 0, ws_f = 0, we_f = 0, ww_f = 0;

static int flag_debug = 0; //debug
/* skip counter to erase warning messages after a delay*/
static int cnt = 0;
// Motor characteristics

double _speed[4] = { SPEED_100 , SPEED_100, SPEED_100, SPEED_100 };
int time_diff;

static double* traj_ptr; // to point the trajectory buffer
static double traj_buf[TRAJ_BUFFER_SIZE];
static int 	traj_max, traj_cnt = 0;
static int control_state[4] = { IDLE,IDLE,IDLE,IDLE };
kin_i _sp_tics = { 0x7d00,0x7d00 ,0x7d00 ,0x7d00 };
//kin_i _sp_tics = { 31350,31350,31350,31350 };

/*	Mutexes */
/* Note: scope of variables and mutexes are the same */
static pthread_mutex_t mutex_sp_tics_mem = PTHREAD_MUTEX_INITIALIZER; //sp_tics_mem
static pthread_mutex_t mutex_grip = PTHREAD_MUTEX_INITIALIZER;	// gripper
static pthread_mutex_t mutex_sp = PTHREAD_MUTEX_INITIALIZER;  //sp
static pthread_mutex_t mutex_err_f = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_curr = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_w1 = PTHREAD_MUTEX_INITIALIZER; // warning messages
static pthread_mutex_t mutex_err = PTHREAD_MUTEX_INITIALIZER; // error messages
static pthread_mutex_t mutex_test = PTHREAD_MUTEX_INITIALIZER; // debug
static pthread_mutex_t mutex_skip = PTHREAD_MUTEX_INITIALIZER; // skip counter
static pthread_mutex_t mutex_motor_charact = PTHREAD_MUTEX_INITIALIZER; // motor speed, acceleration and slow_degree
static pthread_mutex_t mutex_time = PTHREAD_MUTEX_INITIALIZER; // loop time measure
static pthread_mutex_t mutex_ctl_st = PTHREAD_MUTEX_INITIALIZER;//control_state
static pthread_mutex_t mutex_traj_buf = PTHREAD_MUTEX_INITIALIZER;//set_all_sp_angles()

void startTasksControllerRx(void) {
	/* Thread Area	*/
	const char* message = "Thread Task";
	traj_ptr = &traj_buf[0];
	int  iret1, iret2;
	//init_files();

	/* Create independent threads each of which will execute function */
	iret1 = pthread_create(&thread_controller, NULL, pTask_Controller, NULL);
	if (iret1)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret1);
		exit(EXIT_FAILURE);
	}
	Sleep(400); // Delay to make sure that the controller is fully running before spawning other tasks
	//iret2 = pthread_create(&thread_Rx, NULL, pTask_Rx, NULL);
	//if (iret2)
	//{
	//	fprintf(stderr, "Error - pthread_create() return code: %d\n", iret2);
	//	exit(EXIT_FAILURE);
	//}

	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
}

void pthread_joinControllerRx() {
	pthread_join(thread_controller, NULL);
	pthread_join(thread_Rx, NULL);
}



/*******Tasks area *********/

/*
	Task_Controller
	Task that sends command to the robot
*/

static void* pTask_Controller(void* ptr)
{
	double temp_r, elb_wrist_angle, shld_elbow_angle;
	double diff[4], step[4] = { SPEED_SLOW };
	int reset_step_neg_f[4] = { 1 }, reset_step_pos_f[4] = { 1 };
	kin_i prev_sp_tics, _extrapol_tics, sp_tics_memo;
	//kin_i _sp_tics = { 31350,31350,31350,31350 }, prev_sp_tics, _extrapol_tics, sp_tics_memo;
	int j, i, byte_low = 0x80, byte_high = 0x7d, tics = 0, grip = GRIP_OPEN, cnt2, cnt3=0;
	kin_f temp_sp_angles, _pos, zs, temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	char buffer[1] = { '0' };
	char debug_buf[1] = { '1' };
	struct timeval start;	/* starting time */
	struct timeval end;	/* ending time */
	unsigned long e_usec;	/* elapsed microseconds */
	int check_angle = 0;
	int extrapol_over_f = 0;
	int tics_buf;
	int prev_sp_tics_buff;
	/*Temp variable menat for debugging purpose only*/
	double* traj_ptr_debug; // to convert into a pointer
	int control_state_debug = IDLE;
	int  iret2;
	int rx_buf[NUM_JOINTS] = { 0 };
#if defined UDP
	udp_init();
	//Sleep(10);
#endif
	/* Open state file for reading*/
	//fseek(fd_s, 0, SEEK_SET);
	/* reads state file in case an error is received */
	//fread(buffer, 1, 1, fd_s);
	/* Needs to be closed after reading.  Will be re-open for writing in Rx task */
	//fclose(fd_s);

	/* If the system previously had an error*/
	//if(1){  // for debug
	//if (buffer[0] == '1') {
	//	strcat(buf_temp, "The system cannot restart - Call the teacher to clear the errors -Ctrl C to quit  ");
	//	set_errors(buf_temp);
	//	while (1); // Grind the system to a halt
	//}

	// init all gearScale values
	init_KinematicMover();

#if defined _WIN32 && !defined UDP
	// init all values to 0 degree
	for (j = 0; j < 4; j++) {
		//_sp_tics.data[j] = 0x7d00;
		set_sp_tics(j, 0x7d00);
		temp_sp_angles.data[j] = computeJointPos(j, 0x7d00); // converts tics to angles
	}
#else
	/***********Fetches the robot positions by sending 4 packets ***************/

	/***************************OLD VERSION ************************************/
	//cnt2 = 2;
	///* Loops 8 times. More packets than needed to make up for lost packets*/
	//while (cnt2--) {
	//	/* Send 4 bytes to read the current position of the robot*/
	//	for (j = 0; j < 4; j++) {
	//			setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, 0x0);
	//			Sleep(5);
	//			//_sp_tics.data[j] = 0x7d00;
	//			set_sp_tics(j, 0x7d00);
	//			_extrapol_tics.data[j] = 0x7d00;
	//	}
	//}
	//Sleep(100); // delay to make sure all packets get through before reading the packet feedbacks
	///* Loops 32 times to retrieve the robot response*/
	///* More packets than needed to make up for lost packets*/
	//cnt2 = 8;
	//while (cnt2--) {
	//		//mvwprintw(menu_win, 22, 0, "cnt2 right before get_can %d           ", cnt2);
	//		//set_sock_timeout(0);// 0mS timeout
	//		canframe = get_can_mess();
	//		Sleep(5);
	//		//mvwprintw(menu_win, 23, 0, "cnt2 right after get_can %d           ", cnt2);
	//		/* Reads frame response in proper order*/
	//		for (j = 0; j < 4; j++) {
	//			if (canframe.id == (jointIDs[j] + 1)) {
	//				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
	//				//mvwprintw(menu_win, 17 + j, 0, "Debug tics %d:%d															*",j, tics);
	//				//refresh();
	//				// sets sp tic values to current pv tic values
	//				//_sp_tics.data[j] = tics;
	//				set_sp_tics(j, tics);
	//				// sets temp_sp angles to current value
	//				temp_sp_angles.data[j] = computeJointPos(j, tics); // converts tics to angles
	//			}
	//		}
	//		//mvwprintw(menu_win, 25, 0, "cnt2 %d           ",cnt2);
	//}

	/*************************NEW VERSION*******************************************/
	/* Clears buffer */
	for (j = 0; j < NUM_JOINTS; j++) rx_buf[j] = 0;

	/* Keep looping if some packet were not received */
	while (rx_buf[0] == 0 || rx_buf[1] == 0 || rx_buf[2] == 0 || rx_buf[3] == 0) {
		/* Send 4 bytes to force read the current position of the robot*/
		for (j = 0; j < NUM_JOINTS; j++) {
			if (rx_buf[j] ==0) {
				setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, 0x0);
				Sleep(50); // 50 mS
				//_sp_tics.data[j] = 0x7d00;
				set_sp_tics(j, 0x7d00);
				_extrapol_tics.data[j] = 0x7d00;
			}
		}

		Sleep(100); // delay to make sure all packets get through before reading the packet feedbacks
		/* Loops 4 times to retrieve the robot response*/
		cnt2 = 4;
		while (cnt2--) {
			//mvwprintw(menu_win, 22, 0, "cnt2 right before get_can %d           ", cnt2);
			set_sock_timeout(100);// forces a timeout in mS
			canframe = get_can_mess();
			Sleep(5);
			//mvwprintw(menu_win, 23, 0, "cnt2 right after get_can %d           ", cnt2);
			/* Reads frame response in proper order*/
			for (j = 0; j < NUM_JOINTS; j++) {
				if (canframe.id == (jointIDs[j] + 1)) {
					rx_buf[j] = 1; // means a reception
					tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
					//mvwprintw(menu_win, 17 + j, 0, "Debug tics %d:%d															*",j, tics);
					//refresh();
					// sets sp tic values to current pv tic values
					//_sp_tics.data[j] = tics;
					set_sp_tics(j, tics);
					// sets temp_sp angles to current value
					temp_sp_angles.data[j] = computeJointPos(j, tics); // converts tics to angles
				}
			}
			//mvwprintw(menu_win, 25, 0, "cnt2 %d           ",cnt2);
		}
	}
	//mvwprintw(menu_win, 22, 0, "rx_buf: %d %d %d %d          ", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
	refresh();
	set_sock_timeout(0);// no timeout for the rest of the time
	/*************************************************************************************************/

#endif  // _WIN32

	/* Spawns receive task */
	iret2 = pthread_create(&thread_Rx, NULL, pTask_Rx, NULL);
	if (iret2)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret2);
		exit(EXIT_FAILURE);
	}

	/* Memorizes sp tics in case it goes beyond the limits */
	sp_tics_memo = get_all_sp_tics();

	//set_all_sp_angles(temp_sp_angles);
	//set_velocity(0x30,0x80);  // TESTED BUT DOES NOT WORK
	enable_motors(); // turns on all motors
	gettimeofday(&start, 0);	// marks the start time 

	//mvwprintw(menu_win, 25, 0, "entering infinite loop");
	while (1) {
		
		//printf("cnt: %d traj_cnt: %d elbow: %3.2f\n", cnt3++, traj_cnt, sp_angle_get(1)); // for debugging - display task must be disabled
		//mvprintw(26, 0, "looping:%d															*", e_usec);
		//refresh();
		gettimeofday(&end, 0);		/* mark the end time */
		/* now we can do the math. timeval has two elements: seconds and microseconds */
		e_usec = ((end.tv_sec * 1000000) + end.tv_usec) - ((start.tv_sec * 1000000) + start.tv_usec);
		set_time(e_usec);
		gettimeofday(&start, 0);	/* mark the start time */

		temp_curr_angles = all_pv_angles_get(); // NOT TESTED YET

		/* reads sp  angle */
		/* Converts encoder tics set point into angles set point in degrees */
		for (i = 0; i < 4; i++) {
			sp_angles.data[i] = sp_angle_get(i); // sp_angles is used to set the boundaries of the robot 
		}

#ifdef LOCK
		/* Shoulder angle - Elbow angle must be between 150 and 570 degrees */
		/* to avoid the Elbow hitting the body of the robot  */
		shld_elbow_angle = 360 - sp_angles.data[1] - sp_angles.data[2]; // (180- shoulder) + (180 - elbow) = 360 - shoulder - elbow

		/* Elbow angle - Wrist angle must be between 120 and 600 degrees */
		/* to avoid the grip hitting the body of the robot  */
		elb_wrist_angle = 360 - sp_angles.data[2] - sp_angles.data[3];

		// Finds out all zs 
		zs = to_3zs(all_sp_angles_get());
		temp_r = to_r(all_sp_angles_get());

		// if out of range restore sp tics
		if (!((zs.data[2] > Z_TIP_MIN) && (zs.data[0] > Z_M3_MIN) && (zs.data[1] > Z_M4_MIN) && (temp_r < R_MAX) && (elb_wrist_angle > 135 && elb_wrist_angle < 585) && (shld_elbow_angle > 170 && shld_elbow_angle < 720 - 170))) {
			set_all_sp_tics(sp_tics_memo);
			pthread_mutex_lock(&mutex_skip);
			if (w1_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- z or r or angle limits exceeded");
				w1_f = 1;
				set_warnings(buf_temp);
			}
			pthread_mutex_unlock(&mutex_skip);
		}
#endif

		// memorizes sp tics
		sp_tics_memo = get_all_sp_tics();

		//control loop starts here
		for (j = 0; j < 4; j++) {
			switch (get_control_mode(j)) {
			case EXTRAPOL:
				if (traj_max_get() < 1) set_control_mode(j, IDLE);
				/* Extrapolates when too fast */
				/* Slowly increases or decreases sp until it almost match extrapol_tics*/
				diff[j] = (double)(_extrapol_tics.data[j] - get_sp_tics(j)); // difference between desired value - _extrapol_tics- and current _sp_tics
				step[j] = _speed[j];
				if (diff[j] < -SLOW_DEGREES) 	set_sp_tics(j, get_sp_tics(j) - (int)step[j]); //updates sp. The step value is set by set_speed()
				else if (diff[j] >= SLOW_DEGREES)	set_sp_tics(j, get_sp_tics(j) + (int)step[j]);
				else if (fabs(diff[j]) <= SLOW_DEGREES) { 	//when approaching destination must reduce step
					if (diff[j] > 0.0)	set_sp_tics(j, get_sp_tics(j) + SPEED_5); //small step to reduce diff to almost zero
					else set_sp_tics(j, get_sp_tics(j) - SPEED_5);
				}
				if (fabs(diff[j]) <= SPEED_20) { 	//when the destination is almost reached, change state
					set_control_mode(j, TRAJECT);
				}
				break;

			case TRAJECT:

				// reads the next point from the trajectory buffer
				if (isnan(*(traj_ptr + (traj_cnt_get() * 4 + j)))) {
					strcat(buf_temp, "Stopped-NaN");
					set_warnings(buf_temp);
					set_control_mode(0, IDLE);
					set_control_mode(1, IDLE);
					set_control_mode(2, IDLE);
					set_control_mode(3, IDLE);
					traj_max_set(0);
					break;
				}
				tics_buf = computeTics(j, *(traj_ptr + (traj_cnt_get() * 4 + j)));
				// reads the previous trajectory buffer point. Used to detect whether extrapolation is needed.
				prev_sp_tics_buff = prev_sp_tics.data[j];
				// check for angle max. Returns 0 if the angle is within the range or return a cap if angle exceeds.
				check_angle = check_sp_angle(j, computeJointPos(j, tics_buf));
				if (check_angle != 0) {
					tics_buf = check_angle;// too large value, cap tics_buf 	
				}
				//if the user specified 0 point
				if (traj_max_get() == 0) {
					strcat(buf_temp, "stopped-empty-array ");
					set_sp_tics(j, tics_buf);
					set_warnings(buf_temp);
					set_control_mode(j, IDLE);
					break;
				}
				//if the user specified one or more point
				else if (traj_max_get() >= 1) {
					//if it needs extrapolation
					if (fabs((double)tics_buf - (double)prev_sp_tics_buff) > 4 * ONE_DEGREE) {
						//strcat(buf_temp, "- a point must be extrapolated");
						strcat(buf_temp, "-too large, gen. its own via ");
						set_warnings(buf_temp);
						set_control_mode(j, EXTRAPOL);
						_extrapol_tics.data[j] = tics_buf;
					}
					else {
						if (traj_max_get() == 1) { // single point
							//_sp_tics.data[j] = tics_buf;
							set_sp_tics(j, tics_buf);
							set_control_mode(j, IDLE);
						}
						else { //multiple points

							set_sp_tics(j, tics_buf);
							// If no more extrapolation for the specific angle then raise a specific flag
							extrapol_over_f |= ((1 << j) & 0xf);
							//All 4 flags are high. It means that we move on to the next point
							if (extrapol_over_f == 0xf) {
								traj_cnt_inc();
								extrapol_over_f = 0x0;
								// if the end of the array
								if (traj_cnt_get() >= traj_max_get()) {
									traj_cnt_clear();
									set_control_mode(0, IDLE);
									set_control_mode(1, IDLE);
									set_control_mode(2, IDLE);
									set_control_mode(3, IDLE);
								}
							}
						}
					}
				}

				break;
			case IDLE:
				//waits for the user to process a new trajectory buffer
				traj_cnt_clear();
				break;


			} // switch

			// memorizes current _sp_tics
			prev_sp_tics.data[j] = get_sp_tics(j);
		}// end of controller for loop
		//mvwprintw(menu_win, 22, 0,"traj_cnt: %d, traj_max %d          ", traj_cnt, traj_max);

		if (get_keyb_f(RESET_ERROR) == 1) {  // do a reset error if requested
			reset_error();
			set_keyb_f(RESET_ERROR, 0);
			//Copy the memorized position into the _sp_tics setPoint position
			for (j = 0; j < 4; j++) {
				set_sp_tics(j, get_sp_tics_mem(j));
			}
		}
		if (get_keyb_f(EN_MOTOR) == 1) { // do a motor enable if requested
			enable_motors();
			set_keyb_f(EN_MOTOR, 0); // resets flag
		}

		if (get_error_f() == 1) { // do a motor enable if requested
			strcat(buf_temp, "Call the teacher to reset the errors.  The teacher will recalibrate the robot. Ctrl C to quit     ");
			set_errors(buf_temp);
			while (1); // Grind the system to a halt
		}

		grip = gripper_get();
#if defined _WIN32 && !defined UDP
		delay_ms(4 * DELAY_LOOP); // Windows simulation delay
#else				
		/* Loops to send CAN frames to all motors*/
		for (j = 0; j < 4; j++) {
			byte_low = _sp_tics.data[j] & 0x000000ff;
			byte_high = _sp_tics.data[j] & 0x0000ff00;
			byte_high = _sp_tics.data[j] / 256;
			setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, grip);
			delay_ms(DELAY_LOOP);
		}

#endif //_WIN32 and NOT UDP

		//Erase error messages after 3 seconds
		pthread_mutex_lock(&mutex_skip);
		if (cnt++ > WARNING_DELAY) {
			sprintf(buf_temp, "");
			set_warnings(buf_temp);
			cnt = 0;
			// resets warning flags 
			w1_f = 0;
			wb_f = 0;
			ws_f = 0;
			we_f = 0;
			ww_f = 0;
		}
		pthread_mutex_unlock(&mutex_skip);

	}// while 1
}

#if defined _WIN32 && !defined UDP
/*
	Task_Rx Simulated
	Task that normally receives command from the CAN bus.
	It then sets parameter for Task_Controller
*/
#define KI      0.5     // Capacitor
#define KR      1       // Resistor

void* pTask_Rx(void* ptr1) {
	static double i[4] = { 0 }, ITerm2[4] = { 0 };
	int i_pid;
	kin_f temp_curr_angles;

#define		_ERROR	1.25		// current angle has a small error to match real robot
//#define		_ERROR	0.1		// current angle has a small error to match real robot
	while (1) {
		/* Model to simulate all 4 motors */
		for (i_pid = 0; i_pid < 4; i_pid++) {
			//usleep(10000); // 
			delay_ms(10); // Windows
				 /* Vs = Vr + Vc                                         */
			/* Vc represents speed in RPM - integral term           */
			/* Vs/R represents the pwm input -  initial current     */
			/* Vs = Ri + i/C * integral of ic * dt                  */
			/* Vs - integral of ic * dt = Ri                        */
			/* (Vs - integral  of ic * dt) / R = i                  */
			/* KR* Vs - KI *integral  of ic * dt = i                */
			ITerm2[i_pid] = ITerm2[i_pid] + KI * i[i_pid];  // ITerm2 represents the speed in RPM - Vc voltage
			i[i_pid] = (sp_angle_get(i_pid) * KR) - ITerm2[i_pid];     // current in a RC circuit  
			temp_curr_angles.data[i_pid] = ITerm2[i_pid] + _ERROR;
		}
		all_pv_angles_set(temp_curr_angles);

	}
	//close(s[0]);
	exit;
}
#else
/*
	Task_Rx
	Task that receive command from the CAN bus.
	It then sets parameter for Task_Controller
*/
void* pTask_Rx(void* ptr1) {
	int j;
	int tics;
	kin_i temp_tics;
	kin_f temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	int adc_skip = 0, adc_value_temp;
	delay_ms(100);
	char buf[1] = { '1' };  // to be written to state file if error
	/* state file in case an error is received* /
	/* Was closed right before the main loop after reading.  Must be re-open for wriring */
	//fd_s = fopen("../state", "r+");

	while (1) {

		/* Blocks and waits for a frame*/
		canframe = get_can_mess();
		for (j = 0; j < 4; j++) {
			if (canframe.id == (jointIDs[j] + 1)) {
				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
				set_sp_tics_mem(j, tics);  // Used only for reset zeros
				/*NOT TESTED YET*/
				//temp_tics.data[j]=tics;	
				temp_curr_angles.data[j] = computeJointPos(j, tics); // converts current tics to current angles
				all_pv_angles_set(temp_curr_angles);  // FUNCTION TO BE DEFINED

				// 0x02 Velocity lag,  0x08  Comm WDog, 0x10 Pos Lag  0x40 Over Current
				if (canframe.data[0] == 0x02) {
					sprintf(buf_temp2, "Motor %d Vel. lag     \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					//fprintf(fp, "Motor %d Vel. lag     \n", j); // logs up
					set_error_f(1);
					//fseek(fd_s, 0, SEEK_SET);
					//fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if (canframe.data[0] == 0x08) {
					sprintf(buf_temp2, "Motor %d Comm. WDog   \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					//fprintf(fp, "Motor %d Comm. WDog   \n", j);
					set_error_f(1);
					//fseek(fd_s, 0, SEEK_SET);
					//fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if (canframe.data[0] == 0x10) {
					sprintf(buf_temp2, "Motor %d Pos. lag     ", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					//fprintf(fp, "Motor %d Pos. lag     \n", j);
					set_error_f(1);
					//fseek(fd_s, 0, SEEK_SET);
					//fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if (canframe.data[0] == 0x40) {
					sprintf(buf_temp2, "Motor %d Over current \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					//fprintf(fp, "Motor %d Over current \n", j);
					set_error_f(1);
					//fseek(fd_s, 0, SEEK_SET);
					//fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
#if defined UDP
				if (adc_skip++ > 20) { // reads every 12mS * 20 = 240mS
					adc_skip = 0;
					adc_value_temp = (canframe.data[9] & 0xff) << 8 | (canframe.data[8] & 0xff);
					adc_value = adc_value_temp; // to prevent atomicity
				}
#endif

			}
		}
		/*External CAN messages*/
		// if(frame.can_id == 0x0A0){
			// //mvwprintw(menu_win, 17,0,"A0 id received \n");
			// if(frame.data[0] == 0xF0) set_angle1(0x89ff);
			// if(frame.data[0] == 0xF1) set_angle1(0x7D00);
		// } 	

//out_fflush:fflush(stdout);

	}
	//close(s[0]);
	exit;
}
#endif // WIN32 and not UDP

/***** Private definitions ************/

//disable motors
static void disable_motor(void) {
#if !defined _WIN32 || defined UDP
	int i;
	for (i = 0; i < 4; i++) {
		setFrame2(jointIDs[i], 0x01, 0x0a);
		delay_ms(50);
		//usleep(50000);
	}
#endif
}

//enable motors
static int enable_motors(void) {
#if !defined _WIN32 || defined UDP
	int i;
	for (i = 0; i < 4; i++) {

		setFrame2(jointIDs[i], 0x01, 0x09);
		delay_ms(50);
		//usleep(50000);

	}
#endif
	return 1;
}
/*
Send posH in byte 3, posL in byte 4
Typical: 0x01 0x08 0x00 0x00
Currently the provided position data are not used, the joint is set to zero (0x7D00
for CPR-CAN, 0x0000 for CPR-CAN-V2)
To ensure data integrity this command has to be send twice within the time of 50
ms to take effect.
Length has to be 4.
On some boards: Two acknowledge message are sent: 0x0208
*/
#if defined _WIN32 && !defined UDP
static void set_zeros(void) {
	// no hardware involved in WIN32
}
/*
Reset Error
Sets Error Code to 0x04 (Motor not enabled)
Length has to be 2.
Acknowlede message 0x0106 0x001 is sent.
Also set reference
*/
static void reset_error(void) {
	// no hardware involved in WIN32
}

void resetJointsToZero(void) {
	int i;
	disable_motor();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	// no hardware involved in WIN32

	pthread_mutex_lock(&mutex_skip);
	cnt = 0;
	strcat(buf_temp, "- set joint pos. to zero ");
	set_warnings(buf_temp);
	pthread_mutex_unlock(&mutex_skip);

}


static void set_velocity(int id, int vel) {
	// no hardware involved in WIN32
}

/*
Allowed distance between current
position and setpoint position in
encoder tics.
When value is 0 then this test is
switched of.
Value is saved in EEPROM
Standard Value 1200
*/
static void set_max_lag(int low, int high) {
	// no hardware involved in WIN32
}
/*
0x02 0x30 data-H data-L
Number of cycles without incoming CAN
message before COM error
When value is 0 then this test is
switched of.
Value ist saved in EEPROM
Standard Value 1000
*/
static void maxMissedCom(int low, int high) {
	// no hardware involved in WIN32
}
#else
static void set_zeros(void) {
	//Set to zero
	setFrame4(0x10, 0x01, 0x08, 0x0, 0x0);
	delay_ms(1000);
	//sleep(1);
}
/*
Reset Error
Sets Error Code to 0x04 (Motor not enabled)
Length has to be 2.
Acknowlede message 0x0106 0x001 is sent.
Also set reference
*/
static void reset_error(void) {
	int i;
	//reset zeros p.8 UserGuide
	for (i = 0; i < 4; i++) setFrame2(jointIDs[i], 0x01, 0x06);
	delay_ms(1000);
	//sleep(1);
}

void resetJointsToZero(void) {
	int i;
	disable_motor();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	for (i = 0; i < 4; i++) {
		setFrame4(jointIDs[i], 0x01, 0x08, 0x0, 0x0);	// first reset command.. but thats not sufficient
		delay_ms(5000);
		//sleep(5);
		// the command has to be sent twice in the time of two seconds to take effect
		setFrame4(jointIDs[i], 0x01, 0x08, 0x0, 0x0);
		//sleep(5);
		delay_ms(5000);
	}

}


static void set_velocity(int id, int vel) {
	setFrame3(id, 0x05, vel, 0x05);
}

/*
Allowed distance between current
position and setpoint position in
encoder tics.
When value is 0 then this test is
switched of.
Value is saved in EEPROM
Standard Value 1200
*/
static void set_max_lag(int low, int high) {
	//0x02 0x31 data-H data-L
	setFrame3(0x2, 0x31, high, low);
}
/*
0x02 0x30 data-H data-L
Number of cycles without incoming CAN
message before COM error
When value is 0 then this test is
switched of.
Value ist saved in EEPROM
Standard Value 1000
*/
static void maxMissedCom(int low, int high) {
	//0x02 0x31 data-H data-L
	setFrame3(0x2, 0x30, high, low);
}
#endif



static int get_sp_tics_mem(int joint) {
	int tempo;
	pthread_mutex_lock(&mutex_sp_tics_mem);
	tempo = sp_tics_mem.data[joint];
	pthread_mutex_unlock(&mutex_sp_tics_mem);
	return tempo;
}

static void set_sp_tics_mem(int joint, int val) {
	pthread_mutex_lock(&mutex_sp_tics_mem);
	sp_tics_mem.data[joint] = val;
	pthread_mutex_unlock(&mutex_sp_tics_mem);
}



//***************************************************************
// int ticks:		joint encoder tics
// return value: 	joint position in degrees
static double computeJointPos(int joint, int ticks) {
	double gearZero = 32000.0;
	double p = 0;
	p = (ticks - gearZero) / gearScale[joint];
	return p;
}


//***************************************************************
// double pos:		joint position in degree
// return value: 	joint encoder ticks
static int computeTics(int joint, double pos) {
	double gearZero = 32000.0;
	int t = ((int)(pos * gearScale[joint])) + gearZero;
	return t;
}


//***************************************************************
// init all values
static void init_KinematicMover(void)
{
	gearScale[0] = 65.0;		// reduction ratio of the joints: 1° = 65 encoder tics
	gearScale[1] = -65.0;
	gearScale[2] = 65.0;
	gearScale[3] = -65.0;
	//gearScale[3] = -90.0;	// needed some fine tuning

}

static kin_f to_cart(kin_f angle) {
	kin_f cart;
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER*cos(90-elb-sld-wris);
	z=BASE_HGT+
	y= r*sin(to_radians2(base_angle));
	x= r*cos(to_radians2(base_angle));*/
	r = HUMERUS * cos(to_radians2(90 - angle.data[1])) + ULNA * cos(to_radians2(90 - angle.data[1] - angle.data[2])) + GRIPPER * cos(to_radians2(90 - angle.data[1] - angle.data[2] - angle.data[3]));
	// z
	cart.data[2] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.data[1])) + ULNA * sin(to_radians2(90 - angle.data[1] - angle.data[2])) + GRIPPER * sin(to_radians2(90 - angle.data[1] - angle.data[2] - angle.data[3]));
	// y
	cart.data[1] = r * sin(to_radians2(angle.data[0]));
	// x
	cart.data[0] = r * cos(to_radians2(angle.data[0]));
	return cart;

}

/* returns distance from base to tip on the x-y plane*/
static double to_r(kin_f angle) {
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER*cos(90-elb-sld-wris);*/
	r = HUMERUS * cos(to_radians2(90 - angle.data[1])) + ULNA * cos(to_radians2(90 - angle.data[1] - angle.data[2])) + GRIPPER * cos(to_radians2(90 - angle.data[1] - angle.data[2] - angle.data[3]));
	return r;
}

/* returns distance all 3 z: z motor3, z motor4 and z tip*/
static  kin_f to_3zs(kin_f angle) {
	kin_f cart;
	double r;
	// z tip
	cart.data[2] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.data[1])) + ULNA * sin(to_radians2(90 - angle.data[1] - angle.data[2])) + GRIPPER * sin(to_radians2(90 - angle.data[1] - angle.data[2] - angle.data[3]));

	// z motor3
	cart.data[0] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.data[1]));

	// z motor4
	cart.data[1] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.data[1])) + ULNA * sin(to_radians2(90 - angle.data[1] - angle.data[2]));
	return cart;

}

static double to_radians2(double degrees) {
	return (degrees * PI) / 180;
}

static void set_error_f(int m) {
	pthread_mutex_lock(&mutex_err_f);
	error_f = m;
	pthread_mutex_unlock(&mutex_err_f);
}

static int get_error_f(void) {
	int temp;
	pthread_mutex_lock(&mutex_err_f);
	temp = error_f;
	pthread_mutex_unlock(&mutex_err_f);
	return temp;
}

/***** _sp_tics private access ********/

static void set_sp_tics(int nb, int tics) {
	pthread_mutex_lock(&mutex_sp); //sp mutex
	_sp_tics.data[nb] = tics;
	pthread_mutex_unlock(&mutex_sp);
}

static void set_all_sp_tics(kin_i tics) {
	pthread_mutex_lock(&mutex_sp); //sp mutex
	_sp_tics = tics;
	pthread_mutex_unlock(&mutex_sp);
}

static int get_sp_tics(int joint) {
	int tempo;
	pthread_mutex_lock(&mutex_sp); //sp mutex
	tempo = _sp_tics.data[joint];
	pthread_mutex_unlock(&mutex_sp);
	return tempo;
}

static kin_i get_all_sp_tics(void) {
	kin_i tempo;
	pthread_mutex_lock(&mutex_sp); //sp mutex
	tempo = _sp_tics;
	pthread_mutex_unlock(&mutex_sp);
	return tempo;
}


#ifdef _WIN32
// Windows version of gettimeofday()
static int gettimeofday(struct timeval* p, void* tz) {
	ULARGE_INTEGER ul; // As specified on MSDN.
	FILETIME ft;

	// Returns a 64-bit value representing the number of
	// 100-nanosecond intervals since January 1, 1601 (UTC).
	GetSystemTimeAsFileTime(&ft);

	// Fill ULARGE_INTEGER low and high parts.
	ul.LowPart = ft.dwLowDateTime;
	ul.HighPart = ft.dwHighDateTime;
	// Convert to microseconds.
	ul.QuadPart /= 10ULL;
	// Remove Windows to UNIX Epoch delta.
	ul.QuadPart -= 11644473600000000ULL;
	// Modulo to retrieve the microseconds.
	p->tv_usec = (long)(ul.QuadPart % 1000000LL);
	// Divide to retrieve the seconds.
	p->tv_sec = (long)(ul.QuadPart / 1000000LL);

	return 0;
}
#endif


/* control the 3 modes of the state machine */
static void set_control_mode(int j, int m) {
	pthread_mutex_lock(&mutex_ctl_st);
	control_state[j] = m;
	pthread_mutex_unlock(&mutex_ctl_st);
}

static int get_control_mode(int n) {
	int temp;
	pthread_mutex_lock(&mutex_ctl_st);
	temp = control_state[n];;
	pthread_mutex_unlock(&mutex_ctl_st);
	return temp;
}

/************** Public definitions *****************/

/* Sets the gripper:
	GRIP_CLOSE
	GRIP_OPEN
*/
void gripper_set(int val) {
	pthread_mutex_lock(&mutex_grip);
	gripper = val;
	pthread_mutex_unlock(&mutex_grip);

}

int gripper_get(void) {
	int tempo;
	pthread_mutex_lock(&mutex_grip);
	tempo = gripper;
	pthread_mutex_unlock(&mutex_grip);
	return tempo;

}


void set_keyb_f(int s, int state) {
	// add mutex
	switch (s) {
	case RESET_ERROR: reset_err = state;
		break;
	case EN_MOTOR: en_motor_ = state;
		break;
	}
}

void clear_error(void) {
	set_keyb_f(RESET_ERROR, 1);
	pthread_mutex_lock(&mutex_skip);
	cnt = 0;
	strcat(buf_temp, "- error cleared          ");
	set_warnings(buf_temp);
	//mvwprintw(menu_win, 20,0, "error cleared          ");
	pthread_mutex_unlock(&mutex_skip);
}
void enable_motor(void) {
	set_keyb_f(EN_MOTOR, 1);
	//mvwprintw(menu_win, 20,0,"motor enable           ");
	pthread_mutex_lock(&mutex_skip);
	cnt = 0;
	strcat(buf_temp, "- motor enabled          ");
	set_warnings(buf_temp);
	pthread_mutex_unlock(&mutex_skip);
}


int get_keyb_f(int s) {
	// add mutex
	switch (s) {
	case RESET_ERROR: return reset_err;
		break;
	case EN_MOTOR: return en_motor_;
		break;
	}
}




/*
* 	Function to set up a trajectory.
*	The function transmit a new value to the robot every 50mS
*	until the destination is reached.
*	The destination is reached at the last point of the trajectory buffer.
*	The function returns only when the destination is reached.
*
*	Parameters:
*		The first parameter is the pointer that points the trajectory buffer.
* 		The second parameter is the number of via points contained inside
*		the trajectory buffer.
*		The third parameter sets it as a blocking/non-blocking function
*		using BLOCKING or NON_BLOCKING macros
*
*   The trajectory buffer is a 2D array. Each line of the array has four
*	values representing the	four joint angles.
*	There are as many lines as there are via points	in the trajectory.
*	E.g., a 4 by 100 array 	contains a 100-via-point-trajectory.
*
*	The angles must be within the following range:
*		Base (0) BASE_MIN to +BASE_MAX degrees
*		Shoulder(1)SHLD_MIN to +SHLD_MAX
*		Elbow (2) ELB_MIN to +ELB_MAX
*		Wrist (3) WRIST_MIN to +WRIST_MAX
*/
void traj_set(double* ptr, int max, int blocking) {
	// a delay longer than the loop overhead is needed to allow the loop to reach IDLE state
	//delay_ms(100);

	if (max >= TRAJ_BUFFER_SIZE / 4) {
		strcat(buf_temp, "too large trajectory buffer");
		set_errors(buf_temp);
		return;
	}
	// protects traj_cnt and traj_max
	pthread_mutex_lock(&mutex_traj_buf);
	if (max > 0) {
		memcpy(traj_buf, ptr, max * 4 * sizeof(double)); // 8 bytes for a double
		traj_max = max;
		traj_cnt = 0;
	}
	//	To prevent warning, generate one point  whenever start 
	//	and final are the same. 
	else {
		*(traj_buf + 0) = pv_angle_get(0);
		*(traj_buf + 1) = pv_angle_get(1);
		*(traj_buf + 2) = pv_angle_get(2);
		*(traj_buf + 3) = pv_angle_get(3);
		traj_max = 1;
		traj_cnt = 0;
	}
	pthread_mutex_unlock(&mutex_traj_buf);

	set_control_mode(0, TRAJECT);
	set_control_mode(1, TRAJECT);
	set_control_mode(2, TRAJECT);
	set_control_mode(3, TRAJECT);
	if (blocking) {
		// waits for the trajectory to finish
		while (get_control_mode(0) != IDLE || get_control_mode(1) != IDLE || get_control_mode(2) != IDLE || get_control_mode(3) != IDLE) {
			delay_ms(100);
		}
	}
	// returns to idle mode. The above delay is needed to actually reach the state machine idle mode
	//set_control_mode(0, IDLE);
	//set_control_mode(1, IDLE);
	//set_control_mode(2, IDLE);
	//set_control_mode(3, IDLE);
	//pthread_mutex_unlock(&mutex_traj_buf);

}

int traj_max_get() {
	int temp;
	pthread_mutex_lock(&mutex_traj_buf);
	temp = traj_max;
	pthread_mutex_unlock(&mutex_traj_buf);
	return temp;
}

void traj_max_set(int n) {
	pthread_mutex_lock(&mutex_traj_buf);
	traj_max = n;
	pthread_mutex_unlock(&mutex_traj_buf);
}

int traj_cnt_get() {
	int temp;
	pthread_mutex_lock(&mutex_traj_buf);
	temp = traj_cnt;
	pthread_mutex_unlock(&mutex_traj_buf);
	return temp;
}

void traj_cnt_inc(void) {
	pthread_mutex_lock(&mutex_traj_buf);
	traj_cnt++;
	pthread_mutex_unlock(&mutex_traj_buf);
}

void traj_cnt_clear(void) {
	pthread_mutex_lock(&mutex_traj_buf);
	traj_cnt = 0;
	pthread_mutex_unlock(&mutex_traj_buf);
}

/* returns the SP angle in degrees for the specified joint
	A value from 0 to 3 must be provided.
*/

double sp_angle_get(int nb) {
	double tempo;
	pthread_mutex_lock(&mutex_sp); //sp mutex
	tempo = computeJointPos(nb, _sp_tics.data[nb]);
	pthread_mutex_unlock(&mutex_sp);
	return tempo;
}

/* Returns all four SP angles in degrees
	The return values are inside a kin_f structure
*/
kin_f all_sp_angles_get(void) {
	kin_f tempo;
	int i;
	pthread_mutex_lock(&mutex_sp);
	for (i = 0; i < 4; i++) {
		tempo.data[i] = computeJointPos(i, _sp_tics.data[i]);
	}
	pthread_mutex_unlock(&mutex_sp);
	return tempo;
}


/* 	check and cap angles for joints 0 to 3 */
/*	Base (0) BASE_MIN to +BASE_MAX degrees	*/
/*	Shoulder(1)SHLD_MIN to +SHLD_MAX*/
/*	Elbow (2) ELB_MIN to +ELB_MAX */
/*	Wrist (3) WRIST_MIN to +WRIST_MAX*/
/* Returns 0 if within range */
/* Otherwise return a cap value*/

static int check_sp_angle(int i, double _angle) {

	if (i == 0) {
		if (_angle > BASE_MAX || _angle < BASE_MIN)
		{
			pthread_mutex_lock(&mutex_skip);
			if (wb_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle base ");
				set_warnings(buf_temp);
				//mvwprintw(menu_win, 21, 0, "exceeded angle  motor base  at %4.2fdeg.         ", _angle);
				wb_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
			if (_angle > BASE_MAX) return computeTics(i, BASE_MAX); // exceeds angle range
			if (_angle < BASE_MIN) return computeTics(i, BASE_MIN);
		}

	}
	else if (i == 1) {
		if (_angle > SHLD_MAX || _angle < SHLD_MIN)
		{
			pthread_mutex_lock(&mutex_skip);
			if (ws_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle shoulder ");
				set_warnings(buf_temp);
				//mvwprintw(menu_win, 22, 0, "exceeded angle  motor shoulder  at %4.2fdeg.         ", _angle);
				ws_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
			if (_angle > SHLD_MAX) return computeTics(i, SHLD_MAX); // exceeds angle range
			if (_angle < SHLD_MIN) return computeTics(i, SHLD_MIN);
		}

	}
	else if (i == 2) {
		if (_angle > ELB_MAX || _angle < ELB_MIN)
		{
			pthread_mutex_lock(&mutex_skip);
			if (we_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle elbow ");
				set_warnings(buf_temp);
				we_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
			if (_angle > ELB_MAX) return computeTics(i, ELB_MAX); // exceeds angle range
			if (_angle < ELB_MIN) return computeTics(i, ELB_MIN);
		}
	}
	else if (i == 3) {
		if (_angle > WRIST_MAX || _angle < WRIST_MIN)
		{
			pthread_mutex_lock(&mutex_skip);
			if (ww_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle wrist ");
				set_warnings(buf_temp);
				ww_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
			if (_angle > WRIST_MAX) return computeTics(i, WRIST_MAX); // exceeds angle range
			if (_angle < WRIST_MIN) return computeTics(i, WRIST_MIN);
		}
	}
	return 0; // angle within its range
}


/************* curr pv angles*******************/

static void all_pv_angles_set(kin_f _angles) {
	int i;
	pthread_mutex_lock(&mutex_curr);

	for (i = 0; i < 4; i++) {
		curr_angles.data[i] = _angles.data[i];
	}
	pthread_mutex_unlock(&mutex_curr);
}

/* 	Gets all current PV angles in degrees for joints 0 to 3	*/
/*  	The return values are returned a kin_f structure		*/
kin_f all_pv_angles_get(void) {
	kin_f tempo;
	int i;
	pthread_mutex_lock(&mutex_curr);
	for (i = 0; i < 4; i++) {
		tempo.data[i] = curr_angles.data[i];
		/* If out of range return 0.0*/
		if (tempo.data[i] > 1000 || tempo.data[i] < -1000) tempo.data[i] = 0.0;
	}
	pthread_mutex_unlock(&mutex_curr);
	return tempo;
}
/* returns the current PV angle in degrees for the specified joint
	A value from 0 to 3 must be provided.
*/
double pv_angle_get(int nb) {
	double tempo=0.0;
	pthread_mutex_lock(&mutex_curr);
	tempo = curr_angles.data[nb];
	pthread_mutex_unlock(&mutex_curr);
	/* If out of range return 0.0*/
	if (tempo < 1000.0 && tempo > -1000.0) return tempo;
	else return 0.0;
	//return tempo;
}


/******* Warning and error messages setter-getter section **********/

void set_warnings(char* str) {
	pthread_mutex_lock(&mutex_w1);
	//mvwprintw(menu_win, 22, 0, " set warmoings ");
	strcpy(buf_w1, str);
	pthread_mutex_unlock(&mutex_w1);
}

int get_warnings(char* str) {
	pthread_mutex_lock(&mutex_w1);
	strcpy(str, buf_w1);
	pthread_mutex_unlock(&mutex_w1);
	if (str[0] != 0)return 0; // valid string if none null
	else return -1;
}

/* Prints all current warnings to the line and column specified*/
void print_warnings(int v, int h ) {
	pthread_mutex_lock(&mutex_w1);
	mvprintw(v, h, buf_w1);
	pthread_mutex_unlock(&mutex_w1);
}

void set_errors(char* str) {
	pthread_mutex_lock(&mutex_err);
	strcpy(buf_err, str);
	pthread_mutex_unlock(&mutex_err);
}

int get_errors(char* str) {
	pthread_mutex_lock(&mutex_err);
	strcpy(str, buf_err);
	pthread_mutex_unlock(&mutex_err);
	if (str[0] != 0)return 0; // valid string if none null
	else return -1;
}

/* Prints all current errors to the line and column specified*/
void print_errors( int v, int h) {
	pthread_mutex_lock(&mutex_err);
	mvprintw( v, h, buf_err);
	pthread_mutex_unlock(&mutex_err);
}

/* Memorizes the loop time inside time_diff */
void set_time(int t) {
	pthread_mutex_lock(&mutex_time);
	time_diff = t;
	pthread_mutex_unlock(&mutex_time);
}

/* Prints the loop time in uS*/
/* The first parameter is the line and the second is the row */
void print_time(int v, int h) {
	pthread_mutex_lock(&mutex_time);
	mvprintw( v, h, "%d us    ", time_diff);
	pthread_mutex_unlock(&mutex_time);
}

/*****************************************************/

void init_files(void) {
#if defined _WIN32
	//fp = fopen("mover4_v6/log", "w+"); // w+ erases the file if already existing!
	//fd_s = fopen("mover4_v6/state", "w+");

	//fp = fopen("mover4/log", "r+"); // created inside mover4_v5.  Path required
	fd_s = fopen("mover4/state", "r+");
#else
	//fp = fopen("log", "w+"); // created inside mover4_v6.  
	//fd_s = fopen("state", "w");
	fp = fopen("../log", "r+"); // created inside mover4_v6.  Cannot specify a path otherwise does not work
	fd_s = fopen("../state", "r+");
#endif
	//test for debug only
	//char buf[] = { '1' };
	//fseek(fd_s, 0, SEEK_SET);
	//fwrite(buf, sizeof(char), sizeof(buf), fd_s);  // writes 1 to the state file
	//fprintf(fp, "test log file %d   \n",4);
	//fclose(fp);
}


void close_files(void) {
	//fclose(fp); //log
	fclose(fd_s); // state
}

/* Sets a blocking delay in mS*/
void delay_ms(int d) {
#ifdef	WIN32
	Sleep(d);
#else
	usleep(1000 * d);
#endif
}


/* Set the extrapolation speed of the joints.	*/
/* The first parameter is the base, the second parameter is the shoulder, 	*/
/* the third is the elbow and the last parameter is the wrist.	*/
/* Example: speed_set(SPEED_100, SPEED_90, SPEED_10, SPEED_100)	*/

void speed_set(double base, double shld, double elbow, double wrist) {

	pthread_mutex_lock(&mutex_motor_charact);
	_speed[0] = base;
	_speed[1] = shld;
	_speed[2] = elbow;
	_speed[3] = wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}


int readADC_udp(void) {
	return adc_value;
}

/*
	Stops generating via points
*/
void traj_stop(void) {
	traj_max_set(0);
	Sleep(5);
}