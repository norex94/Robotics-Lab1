


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
	fp = fopen("file.txt", "w");
	FILE *fp1;
	fp1 = fopen("file1.txt", "r");

	Microbot robot;				// Local variable of the microbot class

   
    int spe=230;				// Motor speed; should not be higher than 240
	Registerspace Steps{0,0,0,0,0,0,0,0,0};
	Jointspace JointCurrent{0,0,0,0,0,0,0};
	Taskspace TaskHome{ 125,0,0,-1.5707,0,0 };
	Jointspace JointNext{0,0,0,0,0,0,0};
	Taskspace TaskNext{ 1,1,1,0,0,0 };
	Taskspace auka{ 0,0,0,0,0,0 };
	
	robot.InverseKinematics(TaskHome, JointCurrent);
	//robot.InverseKinematics(TaskHome, JointNext);
	char yes = 0;
	bool out = true;



	
	while (out) {
		

	
		printf("Please move arm: \n");
		//scanf(" %c", &yes);
		//Sleep(1000);         1 sek seinkun
		//robot.SendRead(Steps);
		Steps.r[1] = Steps.r[1] - 10;
		Steps.r[2] = Steps.r[2] - 10;
		Steps.r[3] = Steps.r[3] - 10;
		Steps.r[4] = Steps.r[4] - 10;
		Steps.r[5] = Steps.r[5] + 10;
		robot.RegisterToJoint(Steps, JointNext);
		
		//robot.ForwardKinematics(JointCurrent, TaskNext);
		for (int i = 1; i < 6; i++)
		{
			//JointNext.t[i] = JointCurrent.t[i] + JointNext.t[i];
			JointCurrent.t[i] = JointCurrent.t[i] - JointNext.t[i];
			
		}

		robot.ForwardKinematics(JointCurrent, TaskNext);


		fprintf(fp, "%lf", TaskNext.x);
		fprintf(fp, " ");
		fprintf(fp, "%lf", TaskNext.y);
		fprintf(fp, " ");
		fprintf(fp, "%lf", TaskNext.z);
		fprintf(fp, " ");
		fprintf(fp, "%lf", TaskNext.p);
		fprintf(fp, " ");
		fprintf(fp, "%lf", TaskNext.r);
		fprintf(fp, " ");
		fprintf(fp, "%lf \n", TaskNext.g);
		printf("Continue? y/n \n");
		//scanf(" %c", &yes);
		if (yes == 'n')
		{
			out = false;
		}
		
	}

/*
	while (!feof(fp1)) {
		fscanf(fp1, "%lf", &auka.x);
		fscanf(fp1, "%lf", &auka.y);
		fscanf(fp1, "%lf", &auka.z);
		fscanf(fp1, "%lf", &auka.p);
		fscanf(fp1, "%lf", &auka.r);
		fscanf(fp1, "%lf \n", &auka.g);
		printf("%lf %lf %lf %lf \n", auka.x, auka.y, auka.z, auka.p);
	
	}
	*/
	printf("Thank you goodbye");

	fclose(fp);
}





