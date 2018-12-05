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
	j.t[0] = atan2(t.y, t.x); //Joint1
	double RR = sqrt(pow(t.x, 2) + pow(t.y, 2));


	j.t[4] = (t.p*(PI/180)) + (t.r*(PI/180)) + (R1 * j.t[0]); //Joint 5
	j.t[3] = (t.p*(PI/180)) - (t.r*(PI/180)) - (R1 * j.t[0]);
	double R0 = RR - LL * cos(t.p);
	double Z0 = t.z - LL * sin(t.p) - H;

	double beta = atan(Z0 / R0);
	double alpha = atan(sqrt(((4*pow(L, 2)) / (pow(R0, 2) + pow(Z0, 2))) - 1));
	j.t[1] = alpha + beta;
	j.t[2] = beta - alpha;
	return 0;
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace & t)
{

	t.p = ((j.t[4] / (PI*180)) + (j.t[3] / (PI*180))) / 2;
	t.r = ((j.t[4] / (PI*180)) - (j.t[3] / (PI*180))) / 2;

	double RR = (L * cos(j.t[1])) + (L * cos(j.t[2])) + (LL * cos(t.p));
	t.x = RR * cos(j.t[0]);
	t.y = RR * sin(j.t[0]);
	t.z = H + (L * sin(j.t[1])) + (L * sin(j.t[2])) + (LL * sin(t.p));

	return 0;
}

int Microbot::JointToRegister(Jointspace j, Registerspace & r)
{
	r.r[1] = j.t[1] * 7072 / (2 * PI);
	r.r[2] = j.t[2] * 7072 / (2 * PI);
	r.r[3] = j.t[3] * 4158 / (2 * PI);
	r.r[4] = j.t[5] * 1536 / (2 * PI);
	r.r[5] = j.t[5] * 1536 / (2 * PI);
	return 0;
}

int Microbot::RegisterToJoint(Registerspace r, Jointspace & j)
{
	j.t[1] = (r.r[1] * (2 * PI)) / 7072;
	j.t[2] = (r.r[2] * (2 * PI)) / 7072;
	j.t[3] = (r.r[3] * (2 * PI)) / 4158;
	j.t[4] = (r.r[4] * (2 * PI)) / 1536;
	j.t[5] = (r.r[5] * (2 * PI)) / 1536;
	return 0;
}

int Microbot::SetDelta(Registerspace start, Registerspace finish)
{

	return 0;
}
