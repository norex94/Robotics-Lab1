#include "kinematics.h"
#include <stdio.h>


int main()
{
	Microbot robot;				// Local variable of the microbot class

   
    int spe=230;				// Motor speed; should not be higher than 240
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskCurrent{ 127,0,0,-1.5707,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	
	robot.InverseKinematics(TaskCurrent, JointCurrent);
	char yes = 0;
	bool out = true;

	while (out) {
		printf("Please insert steps: \n");
		printf("X: ");
		scanf("%lf", &TaskNext.x);
		scanf(", ");
		printf("Y: ");
		scanf("%lf", &TaskNext.y);
		scanf(", ");
		printf("Z: ");
		scanf("%lf", &TaskNext.z);
		scanf(", ");
		printf("P: ");
		scanf("%lf", &TaskNext.p);
		scanf(", ");
		printf("R: ");
		scanf("%lf", &TaskNext.r);
		scanf(", ");
		printf("G: ");
		scanf("%lf", &TaskNext.g);
		TaskNext.p = TaskNext.p*PI / 180;
		TaskNext.r = TaskNext.r*PI / 180;
		TaskNext.y = -TaskNext.y;
		robot.InverseKinematics(TaskNext, JointNext);
		for (int i = 1; i < 6; i++)
		{
			JointNext.t[i] = JointCurrent.t[i] - JointNext.t[i];
			JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
		}
		robot.JointToRegister(JointNext, Steps);
		robot.SendStep(spe, Steps);
		printf("Continue? y/n \n");
		scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		
	}
	robot.SendClose(221, 1);
	TaskNext.x = 125;
	TaskNext.y = 0;
	TaskNext.z = 0;
	TaskNext.p = -90 * PI / 180;
	TaskNext.r = 0;
	robot.InverseKinematics(TaskNext, JointNext);
	for (int i = 1; i < 6; i++)
	{
		JointNext.t[i] = JointCurrent.t[i] - JointNext.t[i];
		JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
	}
	robot.JointToRegister(JointNext, Steps);
	robot.SendStep(spe, Steps);
	printf("Thank you goodbye");


}





