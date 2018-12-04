#include "kinematics.h"

double Microbot::ABS(double)
{
	return 0.0;
}

int Microbot::ROUND(double)
{
	return 0;
}

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
	return 0;
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace & t)
{
	t.p = (j.t[5] + j.t[4]) / 2;
	t.r = (j.t[5] - j.t[4]) / 2;
	double RR = L * cos(j.t[2]) + L * cos(j.t[3]) + LL * cos(t.p);
	t.x = RR * cos(j.t[1]);
	t.y = RR * sin(j.t[1]);
	t.z = H + L * sin(j.t[2]) + L * sin(j.t[3]) + LL * sin(t.p);
	return 1;
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
