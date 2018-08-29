#include <math.h>
struct Vec4{
	float v[4]={0};
};

struct Vec3{
	float v[3]={0};
};

Vec3 Quat2RPY(Vec4 q){
	float q0 = q.v[0]; //q0 = q.v[1];
	float q1 = q.v[1]; //q1 = q.v[2];
	float q2 = q.v[2]; //q2 = q.v[3];
	float q3 = q.v[3]; //q3 = q.v[0];
	Vec3 RPY;

	RPY.v[0] = atan2(2*(q0*q1 + q2*q3) , 1 - 2*(q1*q1 + q2*q2) );	//Roll
	RPY.v[1] = asin(2*(q0*q2 - q3*q1));				//Pitch
	RPY.v[2] = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3) );	//Yaw

	return RPY;
}
