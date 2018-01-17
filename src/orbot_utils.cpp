/*
Last edited by James Bell on 1/8/18

Returns the rotation rate in rad/s for each wheel in a column vector on a mechanum robot
Count wheels from front left, moving clockwise around the robot
Assume rollers are pointing away from the bot.
Reference: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
*/
#include "orbot_utils.h"
#include <stdint.h>
void getRotationRates(float* rates, float vx,float vy, float vTheta, float x_cm, float y_cm, float length, float width, float radius){
	const float pi=3.14159265359;
	radius=.00762;	
	float wheel_locs[2][4];
	for(uint8_t i=0;i<4;i++){
		wheel_locs[0][i]=((i&2)>0?-length/2:length/2)-x_cm;//hardcoding is for pussies
		wheel_locs[1][i]=(((i&2)>>1)^(i&1)>0?-width/2:width/2)-y_cm;
	}

	for(uint8_t i=0;i<4;i++){
		float a=atan2(wheel_locs[1][i],wheel_locs[0][i]);//about pi/4 or -pi/4
		float b=(((i&2)>>1)^(i&1))>0?-pi/2:pi/2;
		float c=i&1>0?-pi/4:pi/4;
		float l=sqrt(wheel_locs[0][i]*wheel_locs[0][i]+wheel_locs[1][i]*wheel_locs[1][i]);
		rates[i]=vx/radius*cos(b-c)/sin(c);
		rates[i]-=vy/radius*sin(b-c)/sin(c);
		rates[i]-=vTheta/radius*l*sin(b-c+a)/sin(c);
	}

}
