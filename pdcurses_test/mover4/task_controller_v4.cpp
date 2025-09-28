/*
*	Description: Igus Rebel 4
*
*	Author				Date				Version		Description
*	Serge Hould			20 Sept-2024		1.0
*						2 December 2024		1.1			Tested ok - with the elbow only
*						3 December 2024		1.2			Sets up all 4 joints - Tested ok
*														Sets up joints set zeros - Tested ok
*						6 December 2024		1.3			Tested OK on both robots and in both CAN and UDP mode
*						17 Sept. 2025					Does not work propely with Visual Studio 2022.
*														It cannot read the received packet properly.
*						18 Sept. 2025					Found out that  CAN get_packet_mess() is not blocking.
*						19 Sept. 2025					Added  sync_f flag to synchronize Rx CAN packet with
*														when transmitting CAN packets. Also packet.cpp was modified 
*											2.0			to clear the CAN queue before a new CAN frame transmission.
*	TODO:
*
*
*********************************************************************************************************/


#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libgen.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include "../can-utils/terminal.h"
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "../can-utils/lib.h"
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
#include <stdarg.h>
#include "../freeglut/include/GL/freeglut.h"
#include "../freeglut/include/GL/glut.h"

#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>

#include "header/config.h"
#include "header/ncurses_init.h"
#include "header/public.h"
#include "../../../../../common/tick.h"
#include "header/packet.h"

//#if!defined SIMULATION
//#include "header/packet.h"
//#else 
////#include "header/can.h"
//#endif

/*	By defining LOCK it sets the workspace limits.
	Sets the angle's boundaries
	Sets the x-y-z's boundaries
*/
//#define		LOCK
//#define		DEBUG

#define		PI	3.1416
#define		NUM_JOINTS	4

/* Defines the minimum distance to the floor for motor 3, motor 4 and the tip */
#define 	Z_TIP_MIN	2.5	// sets the minimum z position of the tip
#define   	Z_ELBOW_MIN	2.5	// sets the minimum z position of the elbow motor
#define 	Z_WRIST_MIN	2.5	// sets the minimum z position of the wrist motor


/* link length in inches.  Must also be modified in kinematic.c and animation.c */
//#define BASE_HGT 	8.625      //base height   
//#define HUMERUS 	7.375      //shoulder-to-elbow "bone" 
//#define ULNA		8.625      //elbow-to-wrist "bone" 
//#define GRIPPER		6.0        //wrist to gripper's tip

#define	WARNING_DELAY	60	// Warning display delay of about 3 seconds for DELAY_LOOP of 5mS
//#define	TWO_DEGREES		130 // number of tics where the speed slows down
#define	MAX_STEP_DEG		4
#define	EXTRA_DEGREES		1
#define	TRAJ_BUFFER_SIZE 20000

#ifdef LOCK
	/* sets the limits in lock mode */
	#define		BASE_MIN	-150
	#define		BASE_MAX	150
	#define		SHLD_MIN	-50
	//#define		SHLD_MAX	65
	#define		SHLD_MAX	75
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

/* Warning message bits */
#define	WARN_RANGE		0X1
#define	WARN_BASE		0X2
#define	WARN_SHLD		0X4
#define	WARN_ELB		0X8
#define	WARN_WRST		0X10
#define	WARN_TRAJ		0X20
#define	WARN_OTHER		0x40

/* Error message bits */
#define	ERR_OTHER		0X2


/* static local function prototypes */
static void set_velocity(int id, int vel);
static void set_max_lag(int low, int high);
static void maxMissedCom(int low, int high);

static int get_sp_tics_mem(int joint);
static void set_sp_tics_mem(int joint, int val);
static void disable_motor(void);
//static int enable_motors(void);
static double computeJointPos(int, int);
static int computeTics(int, double);
static data_f to_cart(data_f angle);
static void init_KinematicMover(void);
static double to_radians2(double degrees);
static void* pTask_Controller(void* ptr);
static void* pTask_Rx(void* ptr);
static data_f to_3zs(data_f angle);
static double to_r(data_f angle);
static void set_error_f(int);
static int get_error_f(void);
static void set_sp_tics(int nb, int tics);
static int get_sp_tics(int joint);
static kin_i get_all_sp_tics(void);
static void set_all_sp_tics(kin_i tics);
static void set_control_mode(int j, int m);
static int get_control_mode(int n);
static void all_pv_angles_set(data_f _angles);
static int cap_sp_angle(int i, double _angle);
static void buf_warn_clear();
static void buf_warn_fill(int, char*);
static int traj_max_get();
static void traj_max_set(int);
static int traj_cnt_get();
static void traj_cnt_inc(void);
static void traj_cnt_clear(void);

static void buf_err_clear(void);
void controller_stop();
void controller_start();
int controller_get();
static void reset_error(void);


//Global variables
static pthread_t thread_controller;
static pthread_t thread_Rx;
static kin_i sp_tics_mem;
static int jointIDs[4] = { 16,32,48,64 };	// CAN IDs of the robots joints
static data_f sp_angles = { 0 };
static data_f curr_angles;   // AKA pv angle
static int gripper = GRIP_OPEN;

static double gearScale[4];

static int reset_err = 0, en_motor_ = 0, reset_angles = 0;
static int error_f = 0;
static char buf_w1[250] = { 0 };	//warning messages
static char buf_err[250] = { 0 };	//error messages
static char buf_temp[250];	// temporary buffer
static char buf_warn[250];	// warning buffer
static char buf_temp2[250];	// temporary buffer
static int adc_value = 0x1;
static int enable_start = 1; // enable motor only when receiving packets
/* display flags*/
static  int	warn_f = 0;
static  int	err_f = 0;

static int flag_debug = 0; //debug
/* skip counter to erase warning messages after a delay*/
static int warn_skip_cnt = 0;

// Motor characteristics
//double _speed[4] = { SPEED_100 , SPEED_100, SPEED_100, SPEED_100 };
int time_diff;
int controller_cmd=1;

static double* traj_ptr; // to point the trajectory buffer
static double traj_buf[TRAJ_BUFFER_SIZE];
static int 	traj_max, traj_cnt = 0;
static int control_state[4] = { IDLE,IDLE,IDLE,IDLE };
kin_i _sp_tics = { 0x7d00,0x7d00 ,0x7d00 ,0x7d00 };
int sync_f = 0; // flag to synchronize CAN packet reception right after transmission

/*	Mutexes */
/* Note: scope of variables and mutexes are the same */
static pthread_mutex_t mutex_sp_tics_mem = PTHREAD_MUTEX_INITIALIZER; //sp_tics_mem
static pthread_mutex_t mutex_grip = PTHREAD_MUTEX_INITIALIZER;	// gripper
static pthread_mutex_t mutex_sp = PTHREAD_MUTEX_INITIALIZER;  //sp
static pthread_mutex_t mutex_err_f = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_curr = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_err = PTHREAD_MUTEX_INITIALIZER; // error messages
static pthread_mutex_t mutex_warning = PTHREAD_MUTEX_INITIALIZER; // skip counter
static pthread_mutex_t mutex_time = PTHREAD_MUTEX_INITIALIZER; // loop time measure
static pthread_mutex_t mutex_ctl_st = PTHREAD_MUTEX_INITIALIZER;//control_state
static pthread_mutex_t mutex_traj_buf = PTHREAD_MUTEX_INITIALIZER;//set_all_sp_angles()
static pthread_mutex_t mutex_print = PTHREAD_MUTEX_INITIALIZER; // warning messages

void startTasksControllerRx(void) {
	/* Thread Area	*/
	//const char* message = "Thread Task";
	traj_ptr = &traj_buf[0];
	int  iret1, iret2;

	/* Create independent threads each of which will execute function */
	iret1 = pthread_create(&thread_controller, NULL, pTask_Controller, NULL);
	if (iret1)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret1);
		exit(EXIT_FAILURE);
	}
	Sleep(400); // Delay to make sure that the controller is fully running before spawning other tasks


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

static void* pTask_Controller(void* ptr){
	double temp_r, elb_wrist_angle, shld_elbow_angle, angle_buf[4];
	double diff[4];
	int reset_step_neg_f[4] = { 1 }, reset_step_pos_f[4] = { 1 };
	kin_i prev_sp_tics, _extrapol_tics, sp_tics_memo;
	int j, i, pos0 = 0x0, pos1 = 0x0, pos2 = 0x7d, pos3 = 0x0, tics = 0, grip = GRIP_OPEN, cnt2, cnt3 = 0, byte_low = 0x80, byte_high = 0x7d;
	data_f temp_sp_angles, _pos, zs, temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	char buffer[1] = { '0' };
	char debug_buf[1] = { '1' };
	double cap_angle = 0;
	int extrapol_over_f = 0;
	int tics_buf;
	int  iret2,rx_f=0x00;
	long sample; // for time stamp
	uint16_t timer=0;
#if!defined SIMULATION
	if (packet_init() == -1) { //UDP and PCAN mode
		sprintf(buf_temp2, "- Error cannot connect");
		buf_err_fill(ERR_ROBOT, buf_temp2);
	}
#endif
	// init all gearScale values
	init_KinematicMover();
	tick_init();

#if defined _WIN32 && defined SIMULATION
	// init all values to 0 degree
	for (j = 0; j < 4; j++) {
		//_sp_tics.value[j] = 0x7d00;
		set_sp_tics(j, 0x7d00);
		temp_sp_angles.value[j] = computeJointPos(j, 0x7d00); // converts tics to angles
	}
#else

//	/***********Reset **********************************************************/
//#if !defined _WIN32 ||!defined SIMULATION
//	reset_error(); 
//#endif
//
//
//	/***********Enable  **********************************************************/
//	enable_motors(); // turns on all motors

//	Sleep(3000);
		/* Spawns receive task */
	//iret2 = pthread_create(&thread_Rx, NULL, pTask_Rx, NULL);
	//if (iret2)
	//{
	//	fprintf(stderr, "Error - pthread_create() return code: %d\n", iret2);
	//	exit(EXIT_FAILURE);
	//}

	/***********Fetches the robot positions by sending 4 packets ***************/
	timer = 0; // make sure timer is 0 for the UDP server to detect a boot
	/* Keep looping if some packet were not received */
	while (rx_f != 0xf) {
		/* Send 4 bytes to force read the current position of the robot*/
		for (j = 0; j < NUM_JOINTS; j++) {
			if ((rx_f & ((1 << j) & 0xf)) ==0) {
#ifdef REBEL4
				//setFrame8(jointIDs[j], 0x14, 0x80, pos0, pos1, pos2, pos3, 0x23, 0x0); // pos0 MSB
				setFrame8(jointIDs[j], 0x14, timer & 0xff, pos0, pos1, pos2, pos3, (timer >> 8) & 0xff, 0x0); // pos0 MSB	
#else
				//setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, 0x0); // MOVER4
				setFrame6(jointIDs[j], 0x04, timer & 0xff, byte_high, byte_low, (timer >> 8) & 0xff, 0x0);
#endif
				Sleep(50); // 50 mS
				//set_sp_tics(j, 0x7d00);
				//_extrapol_tics.data_i[j] = 0x7d00;
			}
		}


		Sleep(100); // delay to make sure all packets get through before reading the packet feedbacks
		/* Loops 4 times to retrieve the robot response*/
		cnt2 = 4;
		while (cnt2--) {
			set_packet_timeout(100);// forces a timeout in mS in case of no response - UDP mode only
			canframe = get_packet_mess();
			Sleep(5);
			/* Reads frame response in proper order*/
			for (j = 0; j < NUM_JOINTS; j++) {
				if (canframe.id == (jointIDs[j] + 1)) {
					rx_f = rx_f | 1 << j; // sets bit if packet reception
#ifdef	REBEL4
					tics = (16777216 * ((int)((unsigned char)canframe.data[1]))) + (65536 * ((int)((unsigned char)canframe.data[2]))) + (256 * ((int)((unsigned char)canframe.data[3]))) + ((unsigned int)((unsigned char)canframe.data[4]));	// combine the 4 bytes to the position in encoder tics	
#else
					tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
#endif
					// sets sp tic values to current pv tic values
					set_sp_tics(j, tics);
					// sets temp_sp angles to current value
					temp_sp_angles.value[j] = computeJointPos(j, tics); // converts tics to angles
				}
			}
		}
	}
	refresh();
	set_packet_timeout(0);// no timeout for the rest of the time
	timer++; // next timer should never be 0
//
//	for (j = 0; j < NUM_JOINTS; j++) {
//		//setFrame2(jointIDs[j], 0x01, 0x06); // reset 
//		reset_error();
//#ifdef REBEL4
//		setFrame2(0x80, 0x01, 0x06); // reset gripper
//#endif
//	}
//	Sleep(10); // 10 mS
//	for (j = 0; j < NUM_JOINTS; j++) {
//		//setFrame2(jointIDs[j], 0x01, 0x09); // enable motor
//		enable_motors();
//#ifdef REBEL4
//		setFrame2(0x80, 0x01, 0x09); // enable gripper
//#endif
//	}

	/*************************************************************************************************/

#endif  // _WIN32
	sync_f = 1; // enables pTask_Rx to read CAN packets

	/* Spawns receive task */
	iret2 = pthread_create(&thread_Rx, NULL, pTask_Rx, NULL);
	if (iret2)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret2);
		exit(EXIT_FAILURE);
	}

	/* Memorizes sp tics in case it goes beyond the limits */
	sp_tics_memo = get_all_sp_tics();


		/***********Reset **********************************************************/
#if !defined _WIN32 ||!defined SIMULATION
	//reset_error();
#endif


	/***********Enable  **********************************************************/
	//enable_motors(); // turns on all motors



	sample = tick_get();
	while (1) {
		while (tick_diff(sample) < 50); // waits 50mS before looping again
		sample = tick_get(); 

		while (controller_get() == 0) {// grinds to a halt
			Sleep(100);
		} 
		//control loop starts here
		for (j = 0; j < NUM_JOINTS; j++) {
			switch (get_control_mode(j)) {
				case EXTRAPOL:
					if (traj_max_get() <= 0) set_control_mode(j, IDLE);
					/* Extrapolates when too fast */
					/* Slowly increases or decreases sp until it almost match angle_buf[j] */
					diff[j] = angle_buf[j] - sp_angle_get(j);
					//when the destination is almost reached, change state
					if (fabs(diff[j]) <= 2* EXTRA_DEGREES) set_control_mode(j, TRAJECT);
					else if (diff[j] < 0) 	sp_angle_set(j, sp_angle_get(j) - EXTRA_DEGREES); //updates sp. The step value is set by set_speed()
					else if (diff[j] > 0)	sp_angle_set(j, sp_angle_get(j) + EXTRA_DEGREES);
					break;
				case TRAJECT:
					// reads the next point from the trajectory buffer
					if (isnan(*(traj_ptr + (traj_cnt_get() * 4 + j)))) {
						buf_warn_fill(WARN_TRAJ, " - Stopped-NaN");
						for (int k = 0; k < NUM_JOINTS; k++) set_control_mode(k, IDLE); // set IDLE mode
						traj_max_set(0);
						break;
					}
					angle_buf[j] =  *(traj_ptr + (traj_cnt_get() * 4 + j)); 
					cap_angle = cap_sp_angle(j, angle_buf[j]);
					if (cap_angle != 0) {
						angle_buf[j] = cap_angle;// too large value, cap tics_buf 	
					}
					//if the user specified 0 point
					if (traj_max_get() == 0) {
						buf_warn_fill(WARN_TRAJ, " - stopped-empty-array ");
						set_control_mode(j, IDLE);
						break;
					}
					//if the user specified one or more point
					else if (traj_max_get() > 1) {
						//if it needs extrapolation
						if (fabs(angle_buf[j] - sp_angle_get(j)) > MAX_STEP_DEG) { 
							buf_warn_fill(WARN_TRAJ, " - too large, extrapolates to generate its own vias ");
							set_control_mode(j, EXTRAPOL);
							//_extrapol_tics.data_i[j] = tics_buf;
						}
						else {
							sp_angle_set(j,angle_buf[j]);
							// If no more extrapolation for the specific angle then raise a specific flag
							extrapol_over_f |= ((1 << j) & 0xf);
							//All 4 flags are high. It means that we move on to the next point
							if (extrapol_over_f == 0xf) {
								traj_cnt_inc();
								extrapol_over_f = 0x0;
								// if the end of the array
								if (traj_cnt_get() >= traj_max_get()) {
									traj_cnt_clear();
									for (int k = 0; k < NUM_JOINTS; k++) set_control_mode(k, IDLE); // set IDLE mode
								}
							}	
						}
					}
					else { // last via point
						if (fabs(angle_buf[j] - sp_angle_get(j)) > MAX_STEP_DEG) { 
							buf_warn_fill(WARN_TRAJ, " - too large, extrapolates to generate its own vias ");
							set_control_mode(j, EXTRAPOL);
						}
						else {
							sp_angle_set(j, angle_buf[j]);
							set_control_mode(j, IDLE);
						}
					}

					break;
				case IDLE:
					//waits for the user to process a new trajectory buffer
					traj_cnt_clear();
					break;
			} // switch
		}// end of controller for loop

#ifdef LOCK
		/* Shoulder angle - Elbow angle must be between 170 and 550 degrees */
		/* to avoid the Elbow hitting the body of the robot  */
		shld_elbow_angle = 360 - sp_angle_get(1) - sp_angle_get(2); // (180- shoulder) + (180 - elbow) = 360 - shoulder - elbow

		/* Elbow angle - Wrist angle must be between 135 and 585 degrees */
		/* to avoid the grip hitting the body of the robot  */
		elb_wrist_angle = 360 - sp_angle_get(2) - sp_angle_get(3);

		// Returns z position for the elbow joint (0), the wrist joint (1) and the tip of the gripper (2)
		zs = to_3zs(all_sp_angles_get());
		//printf("z elb %3.2f, z wrist %3.2f, z tip %3.2f", zs.value[0], zs.value[1], zs.value[2]);
		// if out of range restore sp tics
		if (!((zs.value[2] > Z_TIP_MIN) && (zs.value[0] > Z_ELBOW_MIN) && (zs.value[1] > Z_WRIST_MIN) && (shld_elbow_angle > 170 && shld_elbow_angle < 720 - 170) && (elb_wrist_angle > 135 && elb_wrist_angle < 585))) {
			//set_all_sp_tics(sp_tics_memo); // return to the previous value 
			//traj_stop();
			//buf_warn_fill(WARN_RANGE, "- z or angle limits exceeded");
		}
#endif
		// memorizes sp tics in case it went to far
		sp_tics_memo = get_all_sp_tics();

#if!defined SIMULATION
		if (get_keyb_f(RESET_ERROR) == 1) {  // do a reset error if requested
			
			for (j = 0; j < NUM_JOINTS; j++) {
				setFrame2(jointIDs[j], 0x01, 0x06); // resets errors
				//reset_error(); // BETTER
			}
			//Copy the memorized position into the _sp_tics setPoint position
			for (j = 0; j < 4; j++) {
				set_sp_tics(j, get_sp_tics_mem(j));
			}

			set_keyb_f(RESET_ERROR, 0);
		}

		if (get_keyb_f(EN_MOTORS) == 1) { // do a motor enable if requested
			for (j = 0; j < NUM_JOINTS; j++) {
				setFrame2(jointIDs[j], 0x01, 0x09); // enable motorS
			}
			//enable_motors(); TO TEST
			set_keyb_f(EN_MOTORS, 0); // resets flag
		}

		if (get_keyb_f(RESET_MOTORS) == 1) {  // do a reset all motor angles to zero

			for (j = 0; j < NUM_JOINTS; j++) {
				setFrame4(jointIDs[j], 0x01, 0x08, 0x0, 0x0); // sets joints to zero
			}
			//set_zeros(); // TO TEST
			Sleep(1);
			// has to be send twice to take effect; measure to avoid unwanted reset
			for (j = 0; j < NUM_JOINTS; j++) {
				setFrame4(jointIDs[j], 0x01, 0x08, 0x0, 0x0); // sets joints to zero
			}
			//set_zeros(); // TO TEST

			Sleep(5);// wait for a short moment especially to allow Joint4 to catch up

			//sets sp tics to 0
			for (j = 0; j < 4; j++) {
				set_sp_tics(j, 0);
			}
			set_keyb_f(RESET_MOTORS, 0); // resets flag
		}
#endif // !defined SIMULATION

		if (get_error_f() == 1) { 
			while (1) {
				//buf_err_fill(ERR_ROBOT, "Error - You must exit      ");
				Sleep(100);
			} // Grinds the system to a halt
		}


		grip = gripper_get();
#if defined _WIN32 && defined SIMULATION
		Sleep(4 * 5); // Windows simulation delay
#else	
#ifdef REBEL4
		//j = 2; // elbow only 0x30 - 0d48
		/* Loops to send CAN frames to all motors*/
		for (j = 0; j < NUM_JOINTS; j++) {
			pos3== (byte)(_sp_tics.data_i[j] & 0x000000ff);
			pos2 = (byte)((_sp_tics.data_i[j]>>8) & 0x000000ff);
			//pos2 = _sp_tics.data_i[j] / 256; // 0x100
			pos1 = (byte)((_sp_tics.data_i[j]>>16) & 0x000000ff);
			//pos1 = _sp_tics.data_i[j] / 65536; // 0x10000
			pos0 = (byte)((_sp_tics.data_i[j]>>24) & 0x000000ff);
			//pos0 = _sp_tics.data_i[j] / 16777216; // 0x1000000
			//setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, grip);
			setFrame8(jointIDs[j], 0x14, timer & 0xff,pos0, pos1,pos2, pos3, (timer >> 8) & 0xff, grip); // pos0 MSB	
			sync_f = 1; // unblocks pTask_Rx when in CAN mode
			Sleep(5);
		}
		setFrame8(0x80, 0x14, timer & 0xff, pos0, pos1, pos2, pos3, (timer >> 8) & 0xff, grip); //gripper
#else //MOVER4
		/* Loops to send CAN frames to all motors*/
		for (j = 0; j < NUM_JOINTS; j++) {
			byte_low = _sp_tics.data_i[j] & 0x000000ff;
			byte_high = _sp_tics.data_i[j] & 0x0000ff00;
			byte_high = _sp_tics.data_i[j] / 256;
			setFrame6(jointIDs[j], 0x04, timer & 0xff, byte_high, byte_low, (timer >> 8) & 0xff, grip);
			sync_f = 1; // unblocks pTask_Rx when in CAN mode
			Sleep(5);
		}
#endif
		//mvprintw_m(GREEN_WHITE, 23, 2, "tics rx: %d      ", timer);
		timer++;
		if (timer == 0) timer = 1; // timer should never be 0

#endif //_WIN32 and SIMULATION

		//Erase error messages after 3 seconds
		if (warn_skip_cnt++ > WARNING_DELAY) {
				buf_warn_clear(); // fills with spaces
				buf_err_clear();
				warn_skip_cnt = 0; // reset skip counter
				// resets flags to allow printing again
				warn_f = 0;
				err_f = 0;
			}
	}// while 1
}

#if defined _WIN32 && defined SIMULATION
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
	data_f temp_curr_angles;
#ifdef REBEL4
#define		_ERROR	0		// current angle has a small error to match real robot
#else
#define		_ERROR	1.25		// current angle has a small error to match real robot
#endif
	while (1) {
		/* Model to simulate all 4 motors */
		for (i_pid = 0; i_pid < NUM_JOINTS; i_pid++) {
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
			temp_curr_angles.value[i_pid] = ITerm2[i_pid] + _ERROR;
		}
		all_pv_angles_set(temp_curr_angles);
		//sprintf(buf_temp2, "- Error: 0x%x  You must exit      ", 10);
		//buf_err_fill(buf_temp2);
		//set_error_f(1); // to grind to a halt
	}
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
	int cnt2;
	kin_i temp_tics;
	data_f temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	int adc_skip = 0, adc_value_temp;
	int rx_f = 0x00;
	delay_ms(100);
	while (1) {
#ifdef UDP
		/* Blocks and waits for a frame when in UDP mode */
		canframe = get_packet_mess();
#else
		/* Waits for sync flag in CAN mode */
		while (sync_f== 0);
		sync_f = 0;
		Sleep(2); // waits 2mS to make sure the packet gets through
		canframe = get_packet_mess();
#endif
		for (j = 0; j < NUM_JOINTS; j++) {
			if (canframe.id == (jointIDs[j] + 1)) {
#ifdef REBEL4
				tics = (16777216 * ((int)((unsigned char)canframe.data[1]))) + (65536 * ((int)((unsigned char)canframe.data[2]))) + (256 * ((int)((unsigned char)canframe.data[3]))) + ((unsigned int)((unsigned char)canframe.data[4]));	// combine the 4 bytes to the position in encoder tics
				///tics = (16777216 * ((int)((canframe.data[1]))) + (65536 * ((int)(canframe.data[2]))) + (256 * ((int)(canframe.data[3]))) + ((canframe.data[4])));	// combine the 4 bytes to the position in encoder tics
#else  MOVER4
				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
#endif
				/* Executes at start up only */
				if (enable_start == 1) {
					enable_start = 0;

					for (int j = 0; j < NUM_JOINTS; j++) {
						//setFrame2(jointIDs[j], 0x01, 0x06); // reset 
						reset_error();
#ifdef REBEL4
						setFrame2(0x80, 0x01, 0x06); // reset gripper
#endif
					}
					Sleep(10); // 10 mS
					for (int j = 0; j < NUM_JOINTS; j++) {
						//setFrame2(jointIDs[j], 0x01, 0x09); // enable motor
						enable_motors();
#ifdef REBEL4
						setFrame2(0x80, 0x01, 0x09); // enable gripper
#endif
					}

				}// end of startup execution
				set_sp_tics_mem(j, tics);  // Used only for reset zeros
				/*NOT TESTED YET*/	
				//mvprintw_m(GREEN_WHITE, 23, 2, "tics rx: %d      ", tics);
				temp_curr_angles.value[j] = computeJointPos(j, tics); // converts current tics to current angles
				all_pv_angles_set(temp_curr_angles);  


#ifdef REBEL4

				if ((canframe.data[0] & 0xFF) != 0x0) {
					//sprintf(buf_temp2, "- Error: 0x%x  (6-OC 5-EncErr 4-PosLag 3-CommWD 2-MotorDis 1-VeloLag 0-BO-WD)                  ", canframe.data[0]);
					//buf_err_fill(ERR_ROBOT,buf_temp2);
					if ((canframe.data[0] & 0xFF) < 0x80) { // If it is not and Over current error, try to restart
						sprintf(buf_temp2, "- Error joint %d: 0x%02x  (7-OC 6-DRV 5-EncErr 4-PosLag 3-Comm 2-MotorDis 1-EStop 0-OTemp)    ", j, canframe.data[0]);
						buf_err_fill(ERR_ROBOT, buf_temp2);
						//controller_stop();
						//disable_motor();
						//Sleep(2000);
						/* Keep looping if some packet were not received */
						//while (rx_f != 0xf) {
						//	/* Send 4 bytes to force read the current position of the robot*/
						//	for (j = 0; j < NUM_JOINTS; j++) {
						//		if ((rx_f & ((1 << j) & 0xf)) == 0) {
						//			//setFrame6(jointIDs[j], 0x04, 0x80, 0x7d,0x80, 0x23, 0x0);
						//			setFrame8(jointIDs[j], 0x14, 0x80, 0x0, 0x0, 0x7d, 0x80, 0x23, 0x0); // pos0 MSB
						//			Sleep(50); // 50 mS
						//			set_sp_tics(j, 0x7d00);
						//		}
						//	}
						//	Sleep(100); // delay to make sure all packets get through before reading the packet feedbacks
						//	/* Loops 4 times to retrieve the robot response*/
						//	cnt2 = 4;
						//	while (cnt2--) {
						//		set_packet_timeout(100);// forces a timeout in mS in case of no response
						//		canframe = get_packet_mess();
						//		Sleep(5);
						//		/* Reads frame response in proper order*/
						//		for (j = 0; j < NUM_JOINTS; j++) {
						//			if (canframe.id == (jointIDs[j] + 1)) {
						//				rx_f = rx_f | 1 << j; // sets bit if packet reception
						//				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
						//				// sets sp tic values to current pv tic values
						//				set_sp_tics(j, tics);
						//				// sets temp_sp angles to current value
						//			}
						//		}
						//	}
						//}

						//set_packet_timeout(0);// no timeout for the rest of the time
						//enable_motor();
						//controller_start();
					}
					else {
						sprintf(buf_temp2, "- Error: 0x%02x  (7-OC 6-DRV 5-EncErr 4-PosLag 3-Comm 2-MotorDis 1-EStop 0-OTemp)      ", canframe.data[0]);
						buf_err_fill(ERR_ROBOT, buf_temp2);
						set_error_f(1); // to grind to a halt when over current
					}
				}
#else  // MOVER4
				if ((canframe.data[0] & 0x7F) != 0x0) {
					//sprintf(buf_temp2, "- Error: 0x%x  (6-OC 5-EncErr 4-PosLag 3-CommWD 2-MotorDis 1-VeloLag 0-BO-WD)                  ", canframe.data[0]);
					//buf_err_fill(ERR_ROBOT,buf_temp2);
					if ((canframe.data[0] & 0x7F) < 0x40) { // If it is not and Over current error, try to restart
						sprintf(buf_temp2, "j: %d - Error: 0x%02x  (6-OC 5-EncErr 4-PosLag 3-CommWD 2-MotorDis 1-VeloLag 0-BO-WD)     ",j, canframe.data[0]);
						buf_err_fill(ERR_ROBOT, buf_temp2);
#ifdef FORCED_RESET
						controller_stop();
						//disable_motor();
						Sleep(2000);
						/* Keep looping if some packet were not received */
						while (rx_f != 0xf) {
							/* Send 4 bytes to force read the current position of the robot*/
							for (j = 0; j < NUM_JOINTS; j++) {
								if ((rx_f & ((1 << j) & 0xf)) == 0) {
									setFrame6(jointIDs[j], 0x04, 0x80, 0x7d, 0x80, 0x23, 0x0);
									Sleep(50); // 50 mS
									set_sp_tics(j, 0x7d00);
								}
							}
							Sleep(100); // delay to make sure all packets get through before reading the packet feedbacks
							/* Loops 4 times to retrieve the robot response*/
							cnt2 = 4;
							while (cnt2--) {
								set_packet_timeout(100);// forces a timeout in mS in case of no response
								canframe = get_packet_mess();
								Sleep(5);
								/* Reads frame response in proper order*/
								for (j = 0; j < NUM_JOINTS; j++) {
									if (canframe.id == (jointIDs[j] + 1)) {
										rx_f = rx_f | 1 << j; // sets bit if packet reception
										tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
										// sets sp tic values to current pv tic values
										set_sp_tics(j, tics);
										// sets temp_sp angles to current value
									}
								}
							}
						}

						set_packet_timeout(0);// no timeout for the rest of the time
						enable_motor();
						controller_start();
#endif
					}
					else {
						sprintf(buf_temp2, "- Error: 0x%02x  (6-OC 5-EncErr 4-PosLag 3-CommWD 2-MotorDis 1-VeloLag 0-BO-WD) - You must exit                       ", canframe.data[0]);
						buf_err_fill(ERR_ROBOT, buf_temp2);
						set_error_f(1); // to grind to a halt when over current
					}
			}
#endif // MOVER4
				//print_warnings(RED_WHITE, 21, 2);
				//print_errors(RED_WHITE, 22, 2);
				
#if!defined SIMULATION
				if (adc_skip++ > 20) { // reads every 12mS * 20 = 240mS
					adc_skip = 0;
					adc_value_temp = (canframe.data[9] & 0xff) << 8 | (canframe.data[8] & 0xff);
					adc_value = adc_value_temp; // to prevent atomicity
				}
#endif
			}

		} // if CAN ID
	} // while(1)
	exit;
}
#endif // WIN32 and SIMULATION

/***** Private definitions ************/

//disable motors
static void disable_motor(void) {
#if !defined _WIN32 ||!defined SIMULATION
	int i;
	for (i = 0; i < 4; i++) {
		setFrame2(jointIDs[i], 0x01, 0x0a);
		delay_ms(50);
		//usleep(50000);
	}
#endif
}

//enable motors
 int enable_motors(void) {
#if !defined _WIN32 ||!defined SIMULATION
	int i;
	for (i = 0; i < 4; i++) {
		setFrame2(jointIDs[i], 0x01, 0x09);
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
#if defined _WIN32 && defined SIMULATION
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

	//pthread_mutex_lock(&mutex_warning);
	//warn_skip_cnt = 0;
	//strcat(buf_temp, "- set joint pos. to zero ");
	//set_warnings(buf_temp);
	//pthread_mutex_unlock(&mutex_warning);
	buf_warn_fill(WARN_OTHER, "- set joint pos. to zero ");


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
	for (int j = 0; j < NUM_JOINTS; j++) {
		setFrame4(jointIDs[j], 0x01, 0x08, 0x0, 0x0); // sets joints to zero
	}
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
	tempo = sp_tics_mem.data_i[joint];
	pthread_mutex_unlock(&mutex_sp_tics_mem);
	return tempo;
}

static void set_sp_tics_mem(int joint, int val) {
	pthread_mutex_lock(&mutex_sp_tics_mem);
	sp_tics_mem.data_i[joint] = val;
	pthread_mutex_unlock(&mutex_sp_tics_mem);
}



//***************************************************************
// int ticks:		joint encoder tics
// return value: 	joint position in degrees
static double computeJointPos(int joint, int ticks) {
	double p = 0;
#ifdef REBEL4
	double gearZero = 0;
	p = (ticks - gearZero) / gearScale[joint];
	return p;
#else
	double gearZero = 32000.0;
	p = (ticks - gearZero) / gearScale[joint];
	return p;
#endif

}


//***************************************************************
// double pos:		joint position in degree
// return value: 	joint encoder ticks
static int computeTics(int joint, double pos) {
#ifdef REBEL4
	double gearZero = 0;
	int t = ((int)(pos * gearScale[joint])) + gearZero;
	return t;
#else
	double gearZero = 32000.0;
	int t = ((int)(pos * gearScale[joint])) + gearZero;
	return t;

#endif
}


//***************************************************************
#ifdef REBEL4
// init all values
static void init_KinematicMover(void)
{
	gearScale[0] = 1400;		// reduction ratio of the joints: 1° to encoder tics
	gearScale[1] = 1400;
	gearScale[2] = 1400;
	gearScale[3] = 1031.5;	

}
#else
// init all values
static void init_KinematicMover(void)
{
	gearScale[0] = 65.0;		// reduction ratio of the joints: 1° = 65 encoder tics
	gearScale[1] = -65.0;
	gearScale[2] = 65.0;
	//gearScale[3] = -65.0;
	gearScale[3] = -100.0;	// After test, it was determined that the wrist has 100 tics/degree

}
#endif

static data_f to_cart(data_f angle) {
	data_f cart;
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER_TIP*cos(90-elb-sld-wris);
	z=BASE_HGT+
	y= r*sin(to_radians2(base_angle));
	x= r*cos(to_radians2(base_angle));*/
	r = HUMERUS * cos(to_radians2(90 - angle.value[1])) + ULNA * cos(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER_TIP * cos(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));
	// z
	cart.value[2] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.value[1])) + ULNA * sin(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER_TIP * sin(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));
	// y
	cart.value[1] = r * sin(to_radians2(angle.value[0]));
	// x
	cart.value[0] = r * cos(to_radians2(angle.value[0]));
	return cart;
}

/* returns distance from base to tip on the x-y plane*/
static double to_r(data_f angle) {
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER_TIP*cos(90-elb-sld-wris);*/
	r = HUMERUS * cos(to_radians2(90 - angle.value[1])) + ULNA * cos(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER_TIP * cos(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));
	return r;
}

/* returns distance all 3 z: z motor3, z motor4 and z tip*/
static  data_f to_3zs(data_f angle) {
	data_f cart;
	double r;
	// z tip
	cart.value[2] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.value[1])) + ULNA * sin(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER_TIP * sin(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));

	// z motor3 -elbow
	cart.value[0] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.value[1]));

	// z motor4 - wrist
	cart.value[1] = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.value[1])) + ULNA * sin(to_radians2(90 - angle.value[1] - angle.value[2]));
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
	_sp_tics.data_i[nb] = tics;
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
	tempo = _sp_tics.data_i[joint];
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
	case EN_MOTORS: en_motor_ = state;
		break;
	case RESET_MOTORS: reset_angles = state;
		break;
	}
}

void clear_error(void) {
	set_keyb_f(RESET_ERROR, 1);
	buf_warn_fill(WARN_OTHER, "- error cleared          ");
}
void enable_motor(void) {
	set_keyb_f(EN_MOTORS, 1);
	buf_warn_fill(WARN_OTHER, "- motor enabled          ");
}


int get_keyb_f(int s) {
	// add mutex
	switch (s) {
	case RESET_ERROR: return reset_err;
		break;
	case EN_MOTORS: return en_motor_;
		break;
	case RESET_MOTORS: return reset_angles;
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
		warn_skip_cnt = 0;
		buf_err_fill(ERR_OTHER,"too large trajectory buffer");
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
	for (int k = 0; k < NUM_JOINTS; k++) set_control_mode(k, TRAJECT); // set TRAJECT mode
	if (blocking) {
		// waits for the trajectory to finish
		while (get_control_mode(0) != IDLE || get_control_mode(1) != IDLE || get_control_mode(2) != IDLE || get_control_mode(3) != IDLE) {
			delay_ms(100);
		}
	}
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
	tempo = computeJointPos(nb, _sp_tics.data_i[nb]);
	pthread_mutex_unlock(&mutex_sp);
	return tempo;
}

void sp_angle_set(int nb, double value) {
	pthread_mutex_lock(&mutex_sp); //sp mutex
	//tempo = computeJointPos(nb, _sp_tics.data_i[nb]);
	_sp_tics.data_i[nb] = computeTics(nb,value);
	pthread_mutex_unlock(&mutex_sp);
}

/* Returns all four SP angles in degrees
	The return values are inside a data_f structure
*/
data_f all_sp_angles_get(void) {
	data_f tempo;
	int i;
	pthread_mutex_lock(&mutex_sp);
	for (i = 0; i < 4; i++) {
		tempo.value[i] = computeJointPos(i, _sp_tics.data_i[i]);
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

static int cap_sp_angle(int i, double _angle) {

	if (i == 0) {
		if (_angle > BASE_MAX || _angle < BASE_MIN)
		{
			buf_warn_fill(WARN_BASE, "- exceeded angle base ");
			if (_angle > BASE_MAX) return BASE_MAX; // exceeds angle range
			if (_angle < BASE_MIN) return BASE_MIN;
		}

	}
	else if (i == 1) {
		if (_angle > SHLD_MAX || _angle < SHLD_MIN)
		{
			buf_warn_fill(WARN_SHLD, "- exceeded angle shoulder ");
			if (_angle > SHLD_MAX) return SHLD_MAX; // exceeds angle range
			if (_angle < SHLD_MIN) return SHLD_MIN;
		}

	}
	else if (i == 2) {
		if (_angle > ELB_MAX || _angle < ELB_MIN)
		{
			buf_warn_fill(WARN_ELB, "- exceeded angle elbow ");
			if (_angle > ELB_MAX) return ELB_MAX; // exceeds angle range
			if (_angle < ELB_MIN) return ELB_MIN;
		}
	}
	else if (i == 3) {
		if (_angle > WRIST_MAX || _angle < WRIST_MIN)
		{
			buf_warn_fill(WARN_WRST, "- exceeded angle wrist ");
			if (_angle > WRIST_MAX) return WRIST_MAX; // exceeds angle range
			if (_angle < WRIST_MIN) return WRIST_MIN;
		}
	}
	return 0; // angle within its range
}


/************* curr pv angles*******************/

static void all_pv_angles_set(data_f _angles) {
	int i;
	pthread_mutex_lock(&mutex_curr);

	for (i = 0; i < 4; i++) {
		curr_angles.value[i] = _angles.value[i];
	}
	pthread_mutex_unlock(&mutex_curr);
}

/* 	Gets all current PV angles in degrees for joints 0 to 3	*/
/*  	The return values are returned a data_f structure		*/
data_f all_pv_angles_get(void) {
	data_f tempo;
	int i;
	pthread_mutex_lock(&mutex_curr);
	for (i = 0; i < 4; i++) {
		tempo.value[i] = curr_angles.value[i];
		/* If out of range return 0.0*/
		if (tempo.value[i] > 1000 || tempo.value[i] < -1000) tempo.value[i] = 0.0;
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
	tempo = curr_angles.value[nb];
	pthread_mutex_unlock(&mutex_curr);
	/* If out of range return 0.0*/
	if (tempo < 1000.0 && tempo > -1000.0) return tempo;
	else return 0.0;
	//return tempo;
}


/******* Warning and error messages setter-getter section **********/

/* Memorizes the loop time inside time_diff */
void set_time(int t) {
	pthread_mutex_lock(&mutex_time);
	time_diff = t;
	pthread_mutex_unlock(&mutex_time);
}

/* Prints the loop time in uS*/
/* The first parameter is the line and the second is the row */
void print_time(int v, int h) {
	int t;
	pthread_mutex_lock(&mutex_time);
	t = time_diff;
	pthread_mutex_unlock(&mutex_time);

	pthread_mutex_lock(&mutex_print);
	mvprintw( v, h, "%d us    ", t);
	pthread_mutex_unlock(&mutex_print);
}
void buf_err_clear() {
	pthread_mutex_lock(&mutex_err);
	sprintf(buf_err, "                                                                                                                             ");
	pthread_mutex_unlock(&mutex_err);
}
//void buf_err_fill(char* mess) {
//	pthread_mutex_lock(&mutex_err);
//	sprintf(buf_err, mess); 
//	pthread_mutex_unlock(&mutex_err);
//}

void buf_err_fill(int err, char* mess) {
	int debug = err_f;
	pthread_mutex_lock(&mutex_err);
	if ((err_f & err) == 0) {
		warn_skip_cnt = 0;
		if (err_f == 0) sprintf(buf_err, mess); // first warning
		else strcat(buf_err, mess); // concatenate more warnings
		err_f = err_f | err;
		debug = err_f;
	}
	pthread_mutex_unlock(&mutex_err);
}
/* Prints all current errors to the line and column specified*/
void print_errors(int color, int v, int h) {
	char temp[250];
	pthread_mutex_lock(&mutex_err);
	sprintf(temp, buf_err);
	pthread_mutex_unlock(&mutex_err);

	pthread_mutex_lock(&mutex_print);
	attron(color);
	mvprintw(v, h, temp);
	attroff(color);
	pthread_mutex_unlock(&mutex_print);
}

void gl_print_errors(double x, double y, double z) {
	int i = 0;
	char temp[250];
	pthread_mutex_lock(&mutex_err);
	sprintf(temp, buf_err);
	pthread_mutex_unlock(&mutex_err);
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
	glPushMatrix();
	glRasterPos3f(y, z, x);// position
	while (temp[i] != '\0') {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, temp[i++]);
	}
	glPopMatrix();
}

void buf_warn_fill(int warn, char* mess) {
	int debug = warn_f;
	pthread_mutex_lock(&mutex_warning);
	if ((warn_f & warn) == 0) {
		warn_skip_cnt = 0;
		if(warn_f ==0) sprintf(buf_warn, mess); // first warning
		else strcat(buf_warn, mess); // concatenate more warnings
		warn_f = warn_f | warn ;
		debug = warn_f;
	}
	pthread_mutex_unlock(&mutex_warning);
}

void buf_warn_clear() {
	pthread_mutex_lock(&mutex_warning);
	sprintf(buf_warn, "                                                                                            ");
	pthread_mutex_unlock(&mutex_warning);
}

/* Prints all current warnings to the line and column specified*/
void print_warnings(int color, int v, int h) {
	char temp[250];
	pthread_mutex_lock(&mutex_warning);
	sprintf(temp, buf_warn);
	pthread_mutex_unlock(&mutex_warning);

	pthread_mutex_lock(&mutex_print);
	attron(color);
	mvprintw(v, h, temp);
	attroff(color);
	pthread_mutex_unlock(&mutex_print);
}

void gl_print_warnings(double x, double y, double z) {
	int i = 0;
	//char text[] = "this is a test";
	char temp[250];
	pthread_mutex_lock(&mutex_warning);
	sprintf(temp, buf_warn);
	pthread_mutex_unlock(&mutex_warning);
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
	glPushMatrix();
	glRasterPos3f(y, z, x);// position
	while (temp[i] != '\0') {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, temp[i++]);
	}
	glPopMatrix();
}
/*****************************************************/


/* Sets a blocking delay in mS*/
void delay_ms(int d) {
#ifdef	WIN32
	Sleep(d);
#else
	usleep(1000 * d);
#endif
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

void print(int l, int c, char* str){
	pthread_mutex_lock(&mutex_print);
	mvprintw(l, c, str);
	pthread_mutex_unlock(&mutex_print);
}


void mvprintw_m(int col, int l, int c, const char* format, ...) {
	char buf[512];
	va_list vl;
	va_start(vl, format);
	vsnprintf(buf, 511, format, vl);
	va_end(vl);
	//mutex lock
	pthread_mutex_lock(&mutex_print);
	attron(col);
	mvprintw(l, c, "%s", buf);
	attroff(col);
	//mutex unlock
	pthread_mutex_unlock(&mutex_print);
}

void controller_stop() {
	controller_cmd = 0;
}

void controller_start() {
	controller_cmd = 1;
}

int controller_get() {
	return controller_cmd;

}
