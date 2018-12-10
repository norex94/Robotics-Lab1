


#include "kinematics.h"
#include <stdio.h>



void moveTo(Jointspace &JointCurrent, Jointspace &JointNext, int spe, Registerspace &Steps, Microbot &robot) {
	
	for (int i = 1; i < 6; i++)
	{
		JointNext.t[i] = JointCurrent.t[i] - JointNext.t[i];
		JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
	}
	robot.JointToRegister(JointNext, Steps);
	robot.SendStep(spe, Steps);
}

int main()
{

	FILE *fp;
	fp = fopen("file.txt", "w+");

	Microbot robot;				// Local variable of the microbot class

   
    int spe=230;				// Motor speed; should not be higher than 240
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskHome{ 125,0,0,-1.5707,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	
	robot.InverseKinematics(TaskHome, JointCurrent);
	char yes = 0;
	bool out = true;

	while (out) {
		printf("Please move arm: \n");
		scanf(",");
		//robot.SendRead(Steps);
		robot.RegisterToJoint(Steps, JointNext);
		for (int i = 1; i < 6; i++)
		{
			JointNext.t[i] = JointCurrent.t[i] - JointNext.t[i];
			JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
		}
		robot.ForwardKinematics(JointNext, TaskNext);
		putc(TaskNext.x, fp);
		putc(TaskNext.y, fp);
		putc(TaskNext.z, fp);
		putc(TaskNext.p, fp);
		putc(TaskNext.r, fp);
		putc(TaskNext.g, fp);
		putc('\n', fp);
		printf("Continue? y/n \n");
		scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		
	}
	
	printf("Thank you goodbye");

	fclose(fp);
}





