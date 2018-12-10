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
		printf("Please insert coordinates: \n");
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
		TaskNext.g = -TaskNext.g;
		robot.InverseKinematics(TaskNext, JointNext);
		moveTo(JointCurrent, JointNext, spe, Steps, robot);
		printf("Continue? y/n \n");
		scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		
	}
	robot.InverseKinematics(TaskHome, JointNext);
	moveTo(JointCurrent, JointNext, spe, Steps, robot);
	printf("Thank you goodbye");


}





