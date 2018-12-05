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
	Registerspace delta{0,0,0,0,0,0,0,0,0};		// Local variable for input of motor steps

    int spe=236;				// Motor speed; should not be higher than 240
	Registerspace RegCurrent;	// Þrepin sem mótorar eru í núna.
	Jointspace JointCurrent;	// Radianar á hverju samskeyti núna.
	Taskspace TaskCurrent;		// Hnit x,y,z,p,r,g --->  p = pitch, r = roll, g = grip
	//Registerspace RegNext;		// Þrep sem svara til næstu staðsetningu arms
	//Jointspace JointNext;		// Radianar á hverju samskeyti í næstu staðsetningu
	Taskspace TaskNext;			// Hnit á næstu staðsetningu.
	robot.SetTaskspace(TaskNext);	// segja róbótanum hnitin á næstu staðsetningu.
	TaskCurrent.x = 0;				// núllstilla þessa staðsetningu sem á að vera "heima" staðsetningin.
	TaskCurrent.y = 0;
	TaskCurrent.z = 0;
	TaskCurrent.p = 0;
	TaskCurrent.r = 0;
	TaskCurrent.g = 0;

	robot.InverseKinematics(TaskCurrent, JointCurrent);		// Tökum hnitin á núverandi staðsetningu og fáum út radiana á samskeytum
	robot.JointToRegister(JointCurrent, RegCurrent);		// Tökum þessa radíana og breytum þeim í steps fyrir hvern mótor

	askForSteps(delta);										// Biðja um þrep
	robot.SendStep(spe, delta);								// Senda þrepin 
		
		//printf("c",TaskCurrent);

	



	
	




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