#include "kinematics.h"
#include <stdio.h>






int main()
{
	Microbot robot;				// Local variable of the microbot class

    //Registerspace delta;		// Local variable for input of motor steps
    int spe=230;				// Motor speed; should not be higher than 240
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskCurrent{ 127,0,0,-1.5707,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	
	robot.InverseKinematics(TaskCurrent, JointCurrent);

	
	while (1) {
		printf("Please insert steps: \n");
		printf("X");
		scanf("%lf", &TaskNext.x);
		scanf(", ");
		printf("Y");
		scanf("%lf", &TaskNext.y);
		scanf(", ");
		printf("Z");
		scanf("%lf", &TaskNext.z);
		scanf(", ");
		printf("P");
		scanf("%lf", &TaskNext.p);
		scanf(", ");
		printf("R");
		scanf("%lf", &TaskNext.r);
		scanf(", ");
		printf("G");
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




	}
	
	




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