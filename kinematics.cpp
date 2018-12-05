#include "kinematics.h"
#include "math.h"

void Microbot::SetTaskspace(Taskspace t)
{
	Taskspace t = t;
}

void Microbot::SetJointspace(Jointspace j)
{
	Jointspace j = j;
}

void Microbot::SetRegisterspace(Registerspace r)
{
	Registerspace r = r;
}

int Microbot::InverseKinematics(Taskspace t, Jointspace & j)
{
	j.t[0] = atan(t.y / t.x); //Joint1
	double RR = sqrt(pow(t.x,2) + pow(t.y,2));
	j.t[4] = t.p + t.r + (R1 * j.t[0]); //Joint 5
	j.t[3] = t.p - t.r - R1;
	double R0 = RR - LL * cos(t.p);
	double Z0 = t.z - LL * sin(t.p) - H;
	double beta = atan(Z0 / R0);
	double alpha = atan(sqrt((pow(4 * L,2) / (pow(R0,2) + pow(Z0,2))) - 1));
	j.t[1] = alpha + beta;
	j.t[2] = beta - alpha;

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
	r.r[0] = j.t[0] * 1125;
	r.r[1] = j.t[1] * 1125;
	r.r[2] = j.t[2] * 661.2;
	r.r[3] = j.t[3] * 241;
	r.r[4] = j.t[4] * 241;

	return 0;
}

int Microbot::RegisterToJoint(Registerspace r, Jointspace & j)
{
	j.t[0] = r.r[0] / 1125;
	j.t[1] = r.r[1] / 1125;
	j.t[2] = r.r[2] / 661.2;
	j.t[3] = r.r[3] / 241;
	j.t[4] = r.r[4] / 241;

	return 0;
}

int Microbot::SetDelta(Registerspace start, Registerspace finish)
{

	start.r[0] = ;

	finish.r[0] = ;
	finish.r[1] = ;
	finish.r[2] = ;
	finish.r[3] = ;
	finish.r[4] = ;
	finish.r[5] = ;


	
	return 0;
}
