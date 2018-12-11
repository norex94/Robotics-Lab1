#include "kinematics.h"
#include <stdio.h>

void moveTo(Jointspace &JointCurrent, Jointspace &JointNext, int Speed, Registerspace &Steps, Microbot &robot) {
	
	for (int i = 1; i < 7; i++)
	{
		JointNext.t[i] = JointCurrent.t[i] - JointNext.t[i];
		JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
	}
	robot.JointToRegister(JointNext, Steps);
	robot.SendStep(Speed, Steps);
}

void lineTo(Taskspace &Travelling, Taskspace &TaskNext, Taskspace &TaskCurrent, Jointspace &JointCurrent, Jointspace &JointNext, int Speed, Registerspace &Steps, Microbot &robot)
{
	double a = pow((TaskNext.x - TaskCurrent.x), 2);
	double b = pow((TaskNext.y - TaskCurrent.y), 2);
	double c = pow((TaskNext.z - TaskCurrent.z), 2);
	double distance = sqrt(a + b + c);						// Finna lengd l�nu � milli byrjunar sta�s og enda sta�s
	double numOfIntervals = round((distance / 20) +1);		// Fj�ldi punkta sem reikna � �t � lei�inni. 20 allavega einn punktur � hverjum 2 sentimetrum
	printf("distance: ");
	printf("%lf", numOfIntervals);
	printf("\n");
	for (int i = 0; i < numOfIntervals+1; i++)				// Fyrir hvern punkt er kalla� � inverse kinematics og svo � moveTo
	{
		Travelling.x = TaskCurrent.x + ((i / numOfIntervals) * (TaskNext.x - TaskCurrent.x));
		Travelling.y = TaskCurrent.y + ((i / numOfIntervals) * (TaskNext.y - TaskCurrent.y));
		Travelling.z = TaskCurrent.z + ((i / numOfIntervals) * (TaskNext.z - TaskCurrent.z));
		Travelling.p = TaskCurrent.p + ((i / numOfIntervals) * (TaskNext.p - TaskCurrent.p));
		Travelling.r = TaskCurrent.r + ((i / numOfIntervals) * (TaskNext.r - TaskCurrent.r));
		printf("%lf", Travelling.x);
		printf("%lf", Travelling.y);
		printf("%lf", Travelling.z);
		printf("%lf", Travelling.p);
		printf("%lf", Travelling.r);
		robot.InverseKinematics(Travelling, JointNext);
		moveTo(JointCurrent, JointNext, Speed, Steps, robot);
	}
	robot.InverseKinematics(TaskNext, JointNext);
	moveTo(JointCurrent, JointNext, Speed, Steps, robot);

	TaskCurrent = TaskNext;
}

int main()
{
	Microbot robot;				// Local variable of the microbot class

    int Speed = 235;				// Motor speed; should not be higher than 240
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskHome{ 125,0,0,-1.5707,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Jointspace TravellingJ{ 0,0,0,0,0,0,0 };
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	Taskspace TaskCurrent{ 125,0,0,-1.5707,0,0 };
	Taskspace Travelling{ 125,0,0,-1.5707,0,0 };
	
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

		lineTo(Travelling, TaskNext, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);

		//robot.InverseKinematics(TaskNext, JointNext);
		//moveTo(JointCurrent, JointNext, Speed, Steps, robot);
		printf("Continue? y/n \n");
		scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		
	}
	lineTo(Travelling, TaskHome, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);
	printf("Thank you goodbye");


}
