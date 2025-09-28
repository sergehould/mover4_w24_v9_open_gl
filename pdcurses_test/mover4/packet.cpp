/****************************************************************************
 *  packet.cpp 
 *
 *  Description: low level functions for UDP and CAN packets tx/rx
 * 
 *	Author			Date				Version
 *	Serge Hould		27 Feb. 2023		v1.0.0	    Tested in loopback mode - crude 
 *                                                  packets are retured to the client. No CAN 
 *                                                  packets involved. See server bbb_server_udp-can.cpp
 * SH               3 Mar. 2023         v2.0.0      Populates setFrame2(), setFrame3() and setFrame4()
 *                                                  Add limits to setFrame6() - the limits inside setFrame6() 
 *                                                  inside can.c were removed - see 247-607\projects\visual_studio
 *                                                  \multithread_mover4_udp\pdcurses_test\mover4
 *													Tested succesfully with the real robot.
 *	SH				6 Mar.2023						Was tested successfully on 3 robots running simulteanously
 *  SH				17 May 2023         v2.1.0      Add set_packet_timeout()
 *                                                  Remove closesocket(sockfd) and WSACleanup() to prevent 10093 error
 * SH               30 May 2023         v2.2.0      Remove joints's boundaries
 * SH               5 Dec. 2024         v3          File renamed. Option to send UDP or PCAN packets.
  *                  Sept 2025          v4          Added  CAN_Reset(PcanHandle) to setFramex to clear the queue before 
  *                                                 transmitting CAN packet. Otherwise the queue gets filled with too many packets.
  *                                                 Especially with Rebel4, since it contains extra packets.
  *                                                   
  *                                                 Use of buf_err_fill() to print errors
 *
 *  
 *****************************************************************************/

#ifdef _WIN32
//#include <Windows.h>
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
#include "header/public.h"
#endif
#include <curses.h>

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>

#include <winsock2.h>
#include <stdio.h>
#include <tchar.h>

#pragma comment(lib,"WS2_32")

#include "header/packet.h"
#include "header/config.h"
#include "header/task_controller.h"
#include "header/ncurses_init.h"
#include "header/PCANBasic.h"
#include "header/public.h"

#define HIGH_LIM1 	0xA41C 		// -150 degrees 
#define LOW_LIM1 	0x55E3		// +150 degrees 	
#define HIGH_LIM2 	0X89DD 		// -50 degrees 
#define LOW_LIM2 	0X6C47	//  +65degrees 
#define HIGH_LIM3 	0XA105 	//  +140 degrees 
#define LOW_LIM3	0X60B5	//  -110 degrees 
#define HIGH_LIM4 	0XB402 	//  -140 degrees 
#define LOW_LIM4 	0X45C9	//  +140 degrees 	

#define BUF_SIZE 1024
//#define BUF_SIZE 100000
#define PORT 5150


WSADATA wsa;
SOCKET sockfd;
struct sockaddr_in server_addr;

char send_buf[10];
char rec_buf[BUF_SIZE];
int cnt = 0;
static char buf_temp2[250];	// temporary buffer





#ifdef UDP
/* Function to set a timeout to the recvfrom()
    The timeout is in ms.
*/
void set_packet_timeout(DWORD  _timeout) {
    DWORD timeout = _timeout; // in mS
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof timeout);
}

/* Initializes the udp client */
int packet_init(void) {

    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("udp WSAStartup failed. Error Code : %d\n", WSAGetLastError());
        return -1;
    }

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET) {
        printf("udp socket creation failed. Error Code : %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));

    // Filling server information
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(ADDRESS);
}

/* Sends a UDP packet containing a frame of 6 data */
int setFrame6(int id, int data1, int data2, int data3, int data4, int data5, int data6) {
        char buff[30] = { 0 };
        long sum = 0;
        switch (id) {
            case 0x10:
            case 0x11:
                sum = data3 * 256 + data4;
                //if ((sum > HIGH_LIM1) || (sum < LOW_LIM1)) {
                //    mvprintw(20, 0, "Joint 1 exceeds limits\n\n");
                //    refresh();
                //    return -1;
                //}
                break;
            case 0x20:
            case 0x21:
                sum = data3 * 256 + data4;
                //if ((sum > HIGH_LIM2) || (sum < LOW_LIM2)) {
                //    mvprintw(20, 0, "Joint 2 exceeds limits\n\n");
                //    refresh();
                //    //	printf("sum: %x\n",sum);
                //    return -1;
                //}
                break;
            case 0x30:
            case 0x31:
                sum = data3 * 256 + data4;
                //if ((sum > HIGH_LIM3) || (sum < LOW_LIM3)) {
                //    mvprintw(20, 0, "Joint 3 exceeds limits\n\n");
                //    refresh();
                //    //	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
                //    return -1;
                //}
                break;
            case 0x40:
            case 0x41:
                sum = data3 * 256 + data4;
                //if ((sum > HIGH_LIM4) || (sum < LOW_LIM4)) {
                //    mvprintw(20, 0, "Joint 4 exceeds limits\n\n");
                //    refresh();
                //    return -1;
                //}
                break;
        }

        /*Fills the buffer*/
        send_buf[0] = id;
        send_buf[1] = data1;
        send_buf[2] = data2;
        send_buf[3] = data3;
        send_buf[4] = data4;
        send_buf[5] = data5;
        send_buf[6] = data6;

        // Send datagram  to server
        if (sendto(sockfd, (char*)send_buf, 7, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
           // mvprintw_m(RED_WHITE,23, 0, "*udp sendto failed. Error Code : %d\n", WSAGetLastError());
            sprintf(buf_temp2, "* udp sendto failed.Error Code : % d\n", WSAGetLastError());
            buf_err_fill(ERR_ROBOT, buf_temp2);
            //printf("sendto failed. Error Code : %d\n", WSAGetLastError());
            //closesocket(sockfd);
            //WSACleanup();
            return NULL;
        }
        return NULL;

}


/* Sends a UDP packet containing a frame of 8 data */
int setFrame8(int id, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8) {
    char buff[30] = { 0 };
    long sum = 0;
    switch (id) {
    case 0x10:
    case 0x11:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM1) || (sum < LOW_LIM1)) {
        //    mvprintw(20, 0, "Joint 1 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    case 0x20:
    case 0x21:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM2) || (sum < LOW_LIM2)) {
        //    mvprintw(20, 0, "Joint 2 exceeds limits\n\n");
        //    refresh();
        //    //	printf("sum: %x\n",sum);
        //    return -1;
        //}
        break;
    case 0x30:
    case 0x31:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM3) || (sum < LOW_LIM3)) {
        //    mvprintw(20, 0, "Joint 3 exceeds limits\n\n");
        //    refresh();
        //    //	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
        //    return -1;
        //}
        break;
    case 0x40:
    case 0x41:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM4) || (sum < LOW_LIM4)) {
        //    mvprintw(20, 0, "Joint 4 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    }

    /*Fills the buffer*/
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;
    send_buf[3] = data3;
    send_buf[4] = data4;
    send_buf[5] = data5;
    send_buf[6] = data6;
    send_buf[7] = data7;
    send_buf[8] = data8;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 9, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        //mvprintw_m(RED_WHITE, 23, 0, "*udp sendto failed. Error Code : %d\n", WSAGetLastError());
        sprintf(buf_temp2, "*udp sendto failed. Error Code : %d\n", WSAGetLastError());
        buf_err_fill(ERR_ROBOT, buf_temp2);
        //printf("sendto failed. Error Code : %d\n", WSAGetLastError());
        //closesocket(sockfd);
        //WSACleanup();
        return NULL;
    }
    return NULL;

}
/* Sends a UDP packet containing a frame of 3 data */
int setFrame3(int id, int data1, int data2, int data3) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;
    send_buf[3] = data3;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 4, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        //mvprintw_m(RED_WHITE, 23, 0, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        sprintf(buf_temp2, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        buf_err_fill(ERR_ROBOT, buf_temp2);
        //closesocket(sockfd);
        //WSACleanup();
        return NULL;
    }
    return NULL;


}

/* Sends a UDP packet containing a frame of 2 data */
int setFrame2(int id, int data1, int data2) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 3, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        //mvprintw_m(RED_WHITE,23, 0, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        sprintf(buf_temp2, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        buf_err_fill(ERR_ROBOT, buf_temp2);
        //printf("sendto failed. Error Code : %d\n", WSAGetLastError());
        //closesocket(sockfd);
        //WSACleanup();
        return NULL;
    }
    return NULL;


}

/* Sends a UDP packet containing a frame of 4 data */
int setFrame4(int id, int data1, int data2, int data3, int data4) { 
    send_buf[0] = id;
    send_buf[1] = data1;
    send_buf[2] = data2;
    send_buf[3] = data3;
    send_buf[4] = data4;

    // Send datagram  to server
    if (sendto(sockfd, (char*)send_buf, 5, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
       // mvprintw_m(RED_WHITE, 23, 0, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        sprintf(buf_temp2, "udp sendto failed. Error Code : %d\n", WSAGetLastError());
        buf_err_fill(ERR_ROBOT, buf_temp2);
        //closesocket(sockfd);
        //WSACleanup();
        return NULL;
    }
    return NULL;

}

/* Reads received UDP packets */
can_frame_ get_packet_mess(void) {
	    can_frame_ the_frame;
        static int nb = 0;
        int n;
        int server_addr_len;
        nb++;
        
        // Receive acknowledgement from server
        server_addr_len = sizeof(server_addr);

        rec_buf[0] = 0xff; // give a wrong id. In case of timeout it means no specific joint will be returned
        //mvprintw_m(22, 0, "get_packet_mess before recfrom %d           ", nb);
        if ((n = recvfrom(sockfd, rec_buf, BUF_SIZE, 0, (struct sockaddr*)&server_addr, &server_addr_len)) == SOCKET_ERROR) {
            //mvprintw_m(RED_WHITE,24, 0, "*udp recvfrom failed. Error Code : %d\n", WSAGetLastError());
            sprintf(buf_temp2, "*udp recvfrom failed. Error Code : %d\n", WSAGetLastError());
            buf_err_fill(ERR_ROBOT, buf_temp2);
            //printf("recvfrom failed. Error Code : %d\n", WSAGetLastError());
            //closesocket(sockfd);
            //WSACleanup();
            return the_frame;
        }
        //mvprintw_m(23, 0, "get_packet_mess afer recfrom %d           ", nb);
        the_frame.id = rec_buf[0]; //id
        the_frame.data[0] = rec_buf[1];  // error code
        the_frame.data[1] = rec_buf[2];  // V2:pos0 MSB, V1: velocity 
        the_frame.data[2] = rec_buf[3];  // V2:pos1, V1: posH
        the_frame.data[3] = rec_buf[4];  // V2:pos2, V1: posL
        the_frame.data[4] = rec_buf[5];  // V2: pos3 LSB, V1: shunt
        the_frame.data[5] = rec_buf[6];  // timestamp
        the_frame.data[6] = rec_buf[7]; //V2: shunt, V1: divValue
        the_frame.data[7] = rec_buf[8]; //digitalInputs
		the_frame.data[8] = rec_buf[9]; //adc LSB
        the_frame.data[9] = rec_buf[10]; //adc LSB
        //memset(rec_buf, 0, 10); // clears buffer
        return the_frame;
        //printf("n= %d \n", n);
        //rec_buf[n] = '\0';
        //printf("Just received from server at time  %d ms %s\n", current_ms, buffer);
}
#else  // PCAN
/* Function to set a timeout for udp mode only
    The timeout is in ms.
*/
void set_packet_timeout(DWORD  _timeout) {
    // PCAN receiver non-blocking, no need for timeout in PCAN mode
}
TPCANMsg msgCanMessage;
TPCANStatus stsResult;
TPCANHandle PcanHandle = PCAN_USBBUS1;
TPCANBaudrate Bitrate = PCAN_BAUD_500K;
/* Initializes the pcan client */
int packet_init(void) {
    //	//stsResult = CAN_InitializeFD(PcanHandle, "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");
    stsResult = CAN_Initialize(PcanHandle, Bitrate);
    if (stsResult != PCAN_ERROR_OK) {
        //std::cout << "Can not initialize. Please check the defines in the code.\n";
        //ShowStatus(stsResult);
        //std::cout << "\n";
        //std::cout << "Closing...\n";
        //system("PAUSE");
        return -1;
    }
    return 0;
}

/* Sends a CAN packet containing a frame of 6 data */
int setFrame6(int id, int data1, int data2, int data3, int data4, int data5, int data6) {
    long sum = 0;
    TPCANStatus stsResult;
    switch (id) {
    case 0x10:
    case 0x11:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM1) || (sum < LOW_LIM1)) {
        //    mvprintw(20, 0, "Joint 1 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    case 0x20:
    case 0x21:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM2) || (sum < LOW_LIM2)) {
        //    mvprintw(20, 0, "Joint 2 exceeds limits\n\n");
        //    refresh();
        //    //	printf("sum: %x\n",sum);
        //    return -1;
        //}
        break;
    case 0x30:
    case 0x31:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM3) || (sum < LOW_LIM3)) {
        //    mvprintw(20, 0, "Joint 3 exceeds limits\n\n");
        //    refresh();
        //    //	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
        //    return -1;
        //}
        break;
    case 0x40:
    case 0x41:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM4) || (sum < LOW_LIM4)) {
        //    mvprintw(20, 0, "Joint 4 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    }

     // Sends a CAN message with extended ID, and 8 data bytes
    TPCANMsg msgCanMessage;
    msgCanMessage.ID = id;
    msgCanMessage.LEN = (BYTE)6;
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    /*Fills the buffer*/
    msgCanMessage.DATA[0] = data1;
    msgCanMessage.DATA[1] = data2;
    msgCanMessage.DATA[2] = data3;
    msgCanMessage.DATA[3] = data4;
    msgCanMessage.DATA[4] = data5;
    msgCanMessage.DATA[5] = data6;
    CAN_Reset(PcanHandle);// clear rx queue
    stsResult = CAN_Write(PcanHandle, &msgCanMessage);
    if (stsResult != PCAN_ERROR_OK) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "**pcan write failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);
        return NULL;
    }
    return NULL;

}


/* Sends a CAN packet containing a frame of 8 data */
int setFrame8(int id, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8) {
    char buff[30] = { 0 };
    long sum = 0;
    TPCANStatus stsResult;
    switch (id) {
    case 0x10:
    case 0x11:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM1) || (sum < LOW_LIM1)) {
        //    mvprintw(20, 0, "Joint 1 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    case 0x20:
    case 0x21:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM2) || (sum < LOW_LIM2)) {
        //    mvprintw(20, 0, "Joint 2 exceeds limits\n\n");
        //    refresh();
        //    //	printf("sum: %x\n",sum);
        //    return -1;
        //}
        break;
    case 0x30:
    case 0x31:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM3) || (sum < LOW_LIM3)) {
        //    mvprintw(20, 0, "Joint 3 exceeds limits\n\n");
        //    refresh();
        //    //	mvprintw(17,0,"Joint angle : %x\n\n",angle3);
        //    return -1;
        //}
        break;
    case 0x40:
    case 0x41:
        sum = data3 * 256 + data4;
        //if ((sum > HIGH_LIM4) || (sum < LOW_LIM4)) {
        //    mvprintw(20, 0, "Joint 4 exceeds limits\n\n");
        //    refresh();
        //    return -1;
        //}
        break;
    }

    // Sends a CAN message with extended ID, and 8 data bytes
    TPCANMsg msgCanMessage;
    msgCanMessage.ID = id;
    msgCanMessage.LEN = (BYTE)8;
    //msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    /*Fills the buffer*/
    msgCanMessage.DATA[0] = data1;
    msgCanMessage.DATA[1] = data2;
    msgCanMessage.DATA[2] = data3;
    msgCanMessage.DATA[3] = data4;
    msgCanMessage.DATA[4] = data5;
    msgCanMessage.DATA[5] = data6;
    msgCanMessage.DATA[6] = data7;
    msgCanMessage.DATA[7] = data8;
    CAN_Reset(PcanHandle); // clear rx queue
    stsResult = CAN_Write(PcanHandle, &msgCanMessage);
    if (stsResult != PCAN_ERROR_OK) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);
        return NULL;
    }
    return NULL;

}
/* Sends a CAN packet containing a frame of 3 data */
int setFrame3(int id, int data1, int data2, int data3) {
    TPCANStatus stsResult;

    // Sends a CAN message with extended ID, and 8 data bytes
    TPCANMsg msgCanMessage;
    msgCanMessage.ID = id;
    msgCanMessage.LEN = (BYTE)3;
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    /*Fills the buffer*/
    msgCanMessage.DATA[0] = data1;
    msgCanMessage.DATA[1] = data2;
    msgCanMessage.DATA[2] = data3;
    CAN_Reset(PcanHandle);// clear rx queue
    stsResult = CAN_Write(PcanHandle, &msgCanMessage);
    if (stsResult != PCAN_ERROR_OK) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);
        return NULL;
    }
    return NULL;


}

/* Sends a CAN packet containing a frame of 2 data */
int setFrame2(int id, int data1, int data2) {
    TPCANStatus stsResult;

    // Sends a CAN message with extended ID, and 8 data bytes
    TPCANMsg msgCanMessage;
    msgCanMessage.ID = id;
    msgCanMessage.LEN = (BYTE)2;
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    /*Fills the buffer*/
    msgCanMessage.DATA[0] = data1;
    msgCanMessage.DATA[1] = data2;
    CAN_Reset(PcanHandle);// clear rx queue
    stsResult = CAN_Write(PcanHandle, &msgCanMessage);
    if (stsResult != PCAN_ERROR_OK) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);

        return NULL;
    }
    return NULL;

}

/* Sends a CAN packet containing a frame of 4 data */
int setFrame4(int id, int data1, int data2, int data3, int data4) {
    TPCANStatus stsResult;

    // Sends a CAN message with extended ID, and 8 data bytes
    TPCANMsg msgCanMessage;
    msgCanMessage.ID = id;
    msgCanMessage.LEN = (BYTE)4;
    msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
    /*Fills the buffer*/
    msgCanMessage.DATA[0] = data1;
    msgCanMessage.DATA[1] = data2;
    msgCanMessage.DATA[2] = data3;
    msgCanMessage.DATA[3] = data4;
    CAN_Reset(PcanHandle);// clear rx queue
    stsResult = CAN_Write(PcanHandle, &msgCanMessage);
    if (stsResult != PCAN_ERROR_OK) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "*pcan write failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);
        return NULL;
    }
    return NULL;

}

/* Reads received PCAN packets */
can_frame_ get_packet_mess(void) {
    can_frame_ the_frame;
    static int nb = 0;
    nb++;
    TPCANMsg CANMsg;
    TPCANTimestamp CANTimeStamp;

    // We execute the "Read" function of the PCANBasic   
    TPCANStatus stsResult = CAN_Read(PcanHandle, &CANMsg, &CANTimeStamp);
    if (stsResult == PCAN_ERROR_QRCVEMPTY) {
        //mvprintw_m(RED_WHITE, 23, 0, "*pcan receive failed. Error Code : 0x%x\n", stsResult);
        sprintf(buf_temp2, "*pcan receive failed. Error Code : 0x%x\n", stsResult);
        buf_err_fill(ERR_ROBOT, buf_temp2);
        return the_frame;
    }

    //if ((n = recvfrom(sockfd, rec_buf, BUF_SIZE, 0, (struct sockaddr*)&server_addr, &server_addr_len)) == SOCKET_ERROR) {
    //    mvprintw_m(RED_WHITE, 24, 0, "*udp recvfrom failed. Error Code : %d\n", WSAGetLastError());
    //    //printf("recvfrom failed. Error Code : %d\n", WSAGetLastError());
    //    //closesocket(sockfd);
    //    //WSACleanup();
    //    return the_frame;
    //}
    //mvprintw_m(23, 0, "get_packet_mess afer recfrom %d           ", nb);

    the_frame.id = CANMsg.ID; //id
    the_frame.data[0] = CANMsg.DATA[0];  // error code
    the_frame.data[1] = CANMsg.DATA[1];  // V2: pos0 MSB, V2 velocity
    the_frame.data[2] = CANMsg.DATA[2]; // V2: pos1, V1:posH
    the_frame.data[3] = CANMsg.DATA[3]; // V2: pos2, V1: posL
    the_frame.data[4] = CANMsg.DATA[4];  // pos3 LSB
    the_frame.data[5] = CANMsg.DATA[5];  // current mA
    the_frame.data[6] = CANMsg.DATA[6];  //current mA
    the_frame.data[7] = CANMsg.DATA[7]; //digitalInputs + flags
    the_frame.data[8] = 0xee; //adc LSB not implemented
    the_frame.data[9] = 0xff; //adc LSB not implemented
    //memset(rec_buf, 0, 10); // clears buffer
    return the_frame;
    //printf("n= %d \n", n);
    //rec_buf[n] = '\0';
    //printf("Just received from server at time  %d ms %s\n", current_ms, buffer);
}
#endif
