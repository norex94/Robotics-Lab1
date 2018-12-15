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
	double distance = sqrt(a + b + c);						// Straight line length between start and finish.
	double numOfIntervals = round((distance / 20) +1);		// Number of intervals to calculate. 20 -> atleast one point for every 2 cm. Atleast one interval if this rounds to zero for some reason.
	printf("distance: ");
	printf("%lf", numOfIntervals);
	printf("\n");
	for (int i = 0; i < numOfIntervals+1; i++)				// For each point inverse kinematics is done and then moveTo is called.
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
	Travelling.g = TaskNext.g;								// The arm moves the entire path and then the gripper is opened/closed.
	robot.InverseKinematics(Travelling, JointNext);
	moveTo(JointCurrent, JointNext, Speed, Steps, robot);
	TaskCurrent = TaskNext;									// Houston, we have landed... the arm... at the desired position.
}

int main()
{
	Microbot robot;											// Local variable of the microbot class.
	const int logColumns = 50;
	const int logLine = 7;
	double log[logColumns][logLine];
	int n = 1;
	char movement;
	int move = 0;
    int Speed = 235;										// Motor speed; should not be higher than 240.
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskHome{ 220,0,0,0,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Jointspace TravellingJ{ 0,0,0,0,0,0,0 };
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	Taskspace TaskCurrent{ 220,0,0,0,0,0 };
	Taskspace Travelling{ 220,0,0,0,0,0 };
	
	robot.InverseKinematics(TaskHome, JointCurrent);
	char yes = 0;
	bool out = true;

	while (out) {

		printf("Please insert desired action: \n");			// Receive coordinates from user.
		printf("Speed: ");
		scanf("%lf", &Speed);
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
		printf("lineTo or moveTo -->  l/m: ");
		scanf("%lf", &movement);
		TaskNext.p = TaskNext.p*PI / 180;
		TaskNext.r = TaskNext.r*PI / 180;					// Do thing, because science.
		TaskNext.y = -TaskNext.y;
		TaskNext.g = -TaskNext.g;

		lineTo(Travelling, TaskNext, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);
		
		if (movement == 'l') {
			move = 1;
		}if (movement == 'm') {
			move = 2;
		}
		
		log[n][1] = Speed;
		log[n][2] = TaskNext.x;
		log[n][3] = TaskNext.y;
		log[n][4] = TaskNext.z;
		log[n][5] = TaskNext.p;
		log[n][6] = TaskNext.r;
		log[n][7] = move;

		printf("Continue? y/n \n");							// Go again: "y" to input new coordinates or "n" to exit the program.
		scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		n++;
	}

	lineTo(Travelling, TaskHome, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);
	//robot.InverseKinematics(TaskHome, JointCurrent);

	out = true;
	printf("Do you want to run trajectory? y/n \n");
	scanf(" %c", &yes);
	if (yes == 'n')
	{
		out = false;
	}
	if (out) {

		for (int i = 1; i < n + 1; i++)
		{
			Speed = log[i][1];
			TaskNext.x = log[i][2];
			TaskNext.y = log[i][3];
			TaskNext.z = log[i][4];
			TaskNext.p = log[i][5];
			TaskNext.r = log[i][6];
			move = log[i][7];

			if (move == 1) {
				lineTo(Travelling, TaskNext, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);
			}
			if (move == 2) {
				robot.InverseKinematics(TaskNext, JointNext);
				moveTo(JointCurrent, JointNext, Speed, Steps, robot);
			}
			
		}
	}

	lineTo(Travelling, TaskHome, TaskCurrent, JointCurrent, JointNext, Speed, Steps, robot);			//Program exited:   Go home, you're drunk!
	printf("Thank you goodbye");


}
