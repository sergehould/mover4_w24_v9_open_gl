 /*
 * 	packet.h
 *	Description: TODO
 *	
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		
 *	SH					17 May 2023
 */
 
#ifndef UDP_H
#define UDP_H
typedef struct
{
	int data[16];		//data
	int id;
}can_frame_;

int setFrame8(int id, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8);
int setFrame6(int id, int data1, int data2,int data3, int data4,int data5, int data6);
int setFrame3(int id, int data1, int data2, int data3);
int setFrame2(int id, int data1, int data2);
int setFrame4(int id, int data1, int data2, int data3, int data4);
void write_can_mess(char*);
can_frame_ get_packet_mess(void);
int packet_init(void);
void set_packet_timeout(DWORD  _timeout);

#endif  // UDP_H