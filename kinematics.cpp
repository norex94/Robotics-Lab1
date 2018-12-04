#include "kinematics.h"
#include "math.h"

void Microbot::SetTaskspace(Taskspace t)
{
}

void Microbot::SetJointspace(Jointspace j)
{
}

void Microbot::SetRegisterspace(Registerspace r)
{
}

int Microbot::InverseKinematics(Taskspace t, Jointspace & j)
{
	double Joint1 = atan(t.y / t.x);
	double RR = sqrt(t.x ^ 2 + t.y ^ 2);
	double Joint5 = t.p + t.r + (R1 * Joint1);
	double Joint4 = t.p - t.r - R1;
	double R0 = RR - LL * cos(t.p);
	double Z0 = t.z - LL * sin(t.p) - H;
	double beta = atan(Z0 / R0);
	double alpha = atan(sqrt((4*L^2 / (R0^2))
	


	return 0;
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace & t)
{	
	
	return 0;
}

int Microbot::JointToRegister(Jointspace j, Registerspace & r)
{
	return 0;
}

int Microbot::RegisterToJoint(Registerspace r, Jointspace & j)
{
	return 0;
}

int Microbot::SetDelta(Registerspace start, Registerspace finish)
{
	return 0;
}
