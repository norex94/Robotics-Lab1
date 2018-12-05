/*
ELE418 - Robotics.
interface.cpp
	Last revision date : 2-4-99 : Jamie Stultz
	Last revision date : 10-10-2018 : Scott Harding
		1. Replaced all _itoa() function calls with ISO C++11 STL String functions
		2. Replaced all parameters in functions of type Pointer to type Reference
			so all code using a struct must use "." dot operator to reference
			struct data instead of "->" arrow.
*/
#include <string>
#include "kinematics.h"


using String = std::string;
Microbot::Microbot(){
	
	port.Open(1,9600);

}
Microbot::Microbot(Taskspace home)
{
}
int Microbot::MoveTo(Taskspace & t)
{
	return 0;
}
int Microbot::Error(int)
{
	return 0;
}
int Microbot::SetSpeed(int)
{
	return 0;
}
void Microbot::CurrentPosition(Taskspace & t)
{
}
void Microbot::CurrentPosition(Jointspace & j)
{
}
void Microbot::CurrentPosition(Registerspace & r)
{
}
;

int Microbot::SendStep(int speed, Registerspace del)
{
	int i;
	int ret_num=0;
	double timer=1000000;
	char ret[2];
	char c[80] = "@STE ";

	String spe	{std::to_string( speed )};
	String m1	{std::to_string( del.r[1] )};
	String m2	{std::to_string( del.r[2] )};
	String m3	{std::to_string( del.r[3] )};
	String m4	{std::to_string( del.r[4] )};
	String m5	{std::to_string( del.r[5] )};
	String m6	{std::to_string( del.r[6] )};
	String m7	{std::to_string( del.r[7] )};
	
	
	strcat(c, spe.c_str());
	strcat(c, ",");
	strcat(c, m1.c_str());
	strcat(c, ",");
	strcat(c, m2.c_str());
	strcat(c, ",");
	strcat(c, m3.c_str());
	strcat(c, ",");
	strcat(c, m4.c_str());
	strcat(c, ",");
	strcat(c, m5.c_str());
	strcat(c, ",");
	strcat(c, m6.c_str());	
	strcat(c, ",");
	strcat(c, m7.c_str());	
	strcat(c, "\r");

	// print statement for debugging; may be commented out afterwards
	printf("String = %s\n",c);

	fflush( stdin );

	i = strlen(c);
	port.SendData(c,i);

	
	while((ret_num==0) && (timer > 0))
			{			
			ret_num = port.ReadData(ret,2);
			timer = (timer - 0.25);
			}
	
	if(timer <= 0)
		printf("Error");
	
	if(ret_num!=0)
		{
		// print statement for debugging; may be commented out afterwards
		printf( "Return from Microbot: %c\n", ret[0]);
		ret[1] = '\0';
		i = atoi(ret);
		}
	
	return i;
	
}


int Microbot::SendClose(int speed, int force)
{

	int i;
	int ret_num=0;
	char ret[2];
	char c[80] = "@CLO ";

	Registerspace r;

	String spe=std::to_string(speed);

	if (speed != -1)
	{
		strcat(c, spe.c_str());
	
	}

	strcat(c, "\r");

	printf("String = %s\n",c);

	fflush( stdin );

	i = strlen(c);

	port.SendData(c,i);

	i = 0;

	while(i==0)
		i = port.ReadData(ret,2);

	i = 1;

if (force != -1)
	{

	if (speed == -1)
	{speed = 221;}

	r.r[1] = 0;
	r.r[2] = 0;
	r.r[3] = 0;
	r.r[4] = 0;
	r.r[5] = 0;
	r.r[6] = -10*force;

	i = SendStep( speed, r );
	}

return i;		

}



int Microbot::SendRead(Registerspace &read)
{
	int	i, count=1;
	int a=0;
	int ret_num=0;


	char c[10] = "@READ\r";
	char d[80];

	i = strlen(c);
	port.SendData(c,i);

	while(ret_num==0)
		{			
			
			ret_num = port.ReadData(d,1);
	
		}
	

	while(a<2)
	{
		ret_num = 0;

		while(ret_num==0)
			ret_num = port.ReadData(d+count,1);
		
		if(d[count]=='\r')
			a++;

		count++;
	
	}
	
	d[count-1] = '\0';

	int index=2;
	int index1=1;

	while(d[index]!='\0')
	{

	read.r[index1]	 = atoi(d+index);

	index1++;

		while(d[index]!=','&&d[index]!='\0')
			index++;

		if(d[index]==',')
			index++;
	}

	printf("Motor steps in ascii =  %s\n", d+2);
	
	

	return 0;
}


int Microbot::SendSet(int speed)
{	
	int	i;
	int ret_num=0;
	char ret[2];
	char c[15] = "@SET";
	
	std::string spe=std::to_string(speed);
	
	strcat(c, spe.c_str());
	strcat(c, ",");
	strcat(c, "\r");
	
	printf("String = %s\n",c);
	
	i = strlen(c);
	port.SendData(c,i);
	
	while(ret_num==0)
		ret_num = port.ReadData(ret,2);

	ret[1] = '\0';
	i = atoi(ret);
	
	return i;
}

int Microbot::SendReset()
{

	int i;
	int ret_num=0;
	char ret[2];
	char c[80] = "@RESET ";

	
	strcat(c, "\r");

	printf("String = %s\n",c);

	fflush( stdin );

	i = strlen(c);

	port.SendData(c,i);

	i = 0;

	while(i==0)
		i = port.ReadData(ret,2);

	i = 1;
return i;		

}
