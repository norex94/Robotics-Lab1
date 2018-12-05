#include "kinematics.h"
#include <stdio.h>

void askForSteps(Registerspace & delta)
{
	printf("Please insert steps: \n");
	printf("Base, Shoulder, Elbow, Right Wrist, Left Wrist, Grip: ");
	scanf("%u", & delta.r[0]);
	scanf(", ");
	scanf("%u", & delta.r[1]);
	scanf(", ");
	scanf("%u", & delta.r[2]);
	scanf(", ");
	scanf("%u", & delta.r[3]);
	scanf(", ");
	scanf("%u", & delta.r[4]);
	scanf(", ");
	scanf("%u", & delta.r[5]);
	scanf(", ");
	scanf("%u", & delta.r[6]);

}



int main()
{
	Microbot robot;				// Local variable of the microbot class

    //Registerspace delta;		// Local variable for input of motor steps
    int spe=236;				// Motor speed; should not be higher than 240
	Registerspace RegCurrent{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskCurrent{ 1,1,1,0,0,0 };
	Registerspace RegNext{ 0,0,0,0,0,0,0,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	
	//robot.JointToRegister(JointCurrent, RegCurrent);



	printf("TaskCurr %lf \n", TaskCurrent.x);
	printf("TaskCurr %lf \n", TaskCurrent.y);
	printf("TaskCurr %lf \n", TaskCurrent.z);
	printf("TaskCurr %lf \n", TaskCurrent.p);
	printf("TaskCurr %lf \n", TaskCurrent.r);
	printf("TaskCurr %lf \n", TaskCurrent.g);
	printf("Joint %lf \n", JointCurrent.t[0]);
	printf("Joint %lf \n", JointCurrent.t[1]);
	printf("Joint %lf \n", JointCurrent.t[2]);
	printf("Joint %lf \n", JointCurrent.t[3]);
	printf("Joint %lf \n", JointCurrent.t[4]);
	printf("Joint %lf \n", JointCurrent.t[5]);
	printf("Joint %lf \n", JointCurrent.t[6]);
	robot.InverseKinematics(TaskCurrent, JointCurrent);
	
	printf("Joint %lf \n", JointCurrent.t[0]);
	printf("Joint %lf \n", JointCurrent.t[1]);
	printf("Joint %lf \n", JointCurrent.t[2]);
	printf("Joint %lf \n", JointCurrent.t[3]);
	printf("Joint %lf \n", JointCurrent.t[4]);
	printf("Joint %lf \n", JointCurrent.t[5]);
	printf("Joint %lf \n", JointCurrent.t[6]);
	robot.ForwardKinematics(JointCurrent, TaskCurrent);
	printf("TaskCurr %lf \n", TaskCurrent.x);
	printf("TaskCurr %lf \n", TaskCurrent.y);
	printf("TaskCurr %lf \n", TaskCurrent.z);
	printf("TaskCurr %lf \n", TaskCurrent.p);
	printf("TaskCurr %lf \n", TaskCurrent.r);
	printf("TaskCurr %lf \n", TaskCurrent.g);

	//printf("JointCurr \n", JointCurrent.t);
	




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