#include "kinematics.h"

double Microbot::ABS(double)
{
	return 0.0;
}

int Microbot::ROUND(double)
{
	return 0;
}

void Microbot::SetTaskspace(Taskspace &t)
{
}

void Microbot::SetJointspace(Jointspace &j)
{
}

void Microbot::SetRegisterspace(Registerspace &r)
{
}

int Microbot::InverseKinematics(Taskspace t, Jointspace & j)
{
	j.t[1] = atan2(t.y, t.x);
	double RR = sqrt(t.x * t.x + t.y * t.y);
	j.t[5] = t.p + t.r - 1 * j.t[1];		//Var R1
	j.t[4] = t.p - t.r + 1 * j.t[1];		//Var R1
	double R0 = RR - LL * cos(t.p);
	double Z0 = t.z - LL * sin(t.p) - H;
	double beta = atan2(Z0, R0);
	double under = ((R0 * R0) + (Z0 * Z0));
	double thing = sqrt((4.0 * L * L) / (under)-1.0);
	double alpha = atan(thing);
	j.t[2] = beta + alpha;
	j.t[3] = beta - alpha;
	j.t[6] = t.g;
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
	return 0;
}

int Microbot::JointToRegister(Jointspace j, Registerspace & r)
{
	r.r[1] = (int)(j.t[1] * 1125);
	r.r[2] = (int)(j.t[2] * 1125);
	r.r[3] = (int)(j.t[3] * 661.2);
	r.r[4] = (int)(j.t[4] * 241);
	r.r[5] = (int)(j.t[5] * 241);
	r.r[6] = (int)((j.t[6] * 14.606) + r.r[3]);
	return 0;
}

int Microbot::RegisterToJoint(Registerspace r, Jointspace & j)
{
	r.r[1] = (int)(j.t[1] / 1125);
	r.r[2] = (int)(j.t[2] / 1125);
	r.r[3] = (int)(j.t[3] / 661.2);
	r.r[4] = (int)(j.t[4] / 241);
	r.r[5] = (int)(j.t[5] / 241);
	r.r[6] = (int)((j.t[6] - r.r[3]) / 14.6);
	
	return 0;
}

int Microbot::SetDelta(Registerspace start, Registerspace finish)
{

	return 0;
}
