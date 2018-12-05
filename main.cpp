#include "kinematics.h"


int main()
{	
	Microbot robot;				// Local variable of the microbot class
    Registerspace delta;		// Local variable for input of motor steps

    int spe=236;				// Motor speed; should not be higher than 240
	Registerspace RegCurrent;	// �repin sem m�torar eru � n�na.
	Jointspace JointCurrent;	// Radianar � hverju samskeyti n�na.
	Taskspace TaskCurrent;		// Hnit x,y,z,p,r,g --->  p = pitch, r = roll, g = grip
	Registerspace RegNext;		// �rep sem svara til n�stu sta�setningu arms
	Jointspace JointNext;		// Radianar � hverju samskeyti � n�stu sta�setningu
	Taskspace TaskNext;			// Hnit � n�stu sta�setningu.
	robot.SetTaskspace(TaskNext);	// segja r�b�tanum hnitin � n�stu sta�setningu.
	TaskCurrent.x = 0;				// n�llstilla �essa sta�setningu sem � a� vera "heima" sta�setningin.
	TaskCurrent.y = 0;
	TaskCurrent.z = 0;
	TaskCurrent.p = 0;
	TaskCurrent.r = 0;
	TaskCurrent.g = 0;

	robot.InverseKinematics(TaskCurrent, JointCurrent);		// T�kum hnitin � n�verandi sta�setningu og f�um �t radiana � samskeytum
	robot.JointToRegister(JointCurrent, RegCurrent);		// T�kum �essa rad�ana og breytum �eim � steps fyrir hvern m�tor

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