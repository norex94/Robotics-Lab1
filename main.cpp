#include "kinematics.h"


int main()
{	
	Microbot robot;				// Local variable of the microbot class
    Registerspace delta;		// Local variable for input of motor steps

    int spe=236;				// Motor speed; should not be higher than 240
	Registerspace RegCurrent;
	Jointspace JointCurrent;
	Taskspace TaskCurrent;
	Registerspace RegNext;
	Jointspace JointNext;
	Taskspace TaskNext;
	robot.SetTaskspace(TaskNext);
	TaskCurrent.x = 0;
	TaskCurrent.y = 0;
	TaskCurrent.z = 0;
	TaskCurrent.p = 0;
	TaskCurrent.r = 0;
	TaskCurrent.g = 0;

	robot.InverseKinematics(TaskCurrent, JointCurrent);
	robot.JointToRegister(JointCurrent, RegCurrent);

	while (1) {
		printf("TaskCurr \n", TaskCurrent);
		printf("JointCurr \n", JointCurrent);
		printf("regCurr \n", RegCurrent);
		printf("Settu inn drasl \n");
		scanf("lf", TaskNext.x);
		//printf("c",TaskCurrent);


	}



	
	




/* Example; replace it with your own program

	delta.r[7]=0;				// Assign number of steps for each motor
	delta.r[6]=0;
	delta.r[5]=0;
	delta.r[4]=0;
    delta.r[3]=0;
	delta.r[2]=0;	
    delta.r[1]=-500;

	robot.SendStep(spe, delta);	// Send instruction to the microbot
*/
}