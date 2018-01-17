/*
DESCRIPTION: 

FUNCTIONS:
void 	messageCallbackVicon(geometry_msgs::TransformStamped)
	Subscribes to the orbot object on vicon at /vicon/orbot/orbot and assigns the pose to loc and att
void	messageCallbackTarget(geometry_msgs::TransformStamped)
	Subscribes to the target on /orbot_server/target and assigns the target pose to tarLoc and tarAtt
int 	main(int, char**)
	Starts the subscribers and publisher, publishes delta between target and current pose to the /orbot_server/orbot_delta

NOTES:
	Would like to move this functionality into the orbot client eventually, since the client should be able to subscribe
	to both on its own. Only currently using this for debugging/separation of tasks

Last edited by James Bell on 1/9/18
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include "quaternion.h"

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <SerialPort.h>//to install, use sudo apt-get install libserial-dev
#include "orbot_utils.h"

ros::Subscriber viconSub,targetSub;
bool update;
const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.00762*wMax/1.424;// m/s
const float P=2;
const float I=0.2;
const float D=1;
int iteration;
float errorIntX =0.0;
float errorLastX = 0.0;
float errorIntY =0.0;
float errorLastY = 0.0;

Vec3 tarLoc;
Vec3 tarAtt;
Vec3 loc;
Vec3 att;	
void messageCallbackVicon( geometry_msgs::TransformStamped t){
  std::string a =  t.header.frame_id;//currently unused
  //defined in quaternion.h		
  Vec4 quat;
  loc.v[0] = t.transform.translation.x;
  loc.v[1] = t.transform.translation.y;
  loc.v[2] = t.transform.translation.z;
  quat.v[0] = t.transform.rotation.w;
  quat.v[1] = t.transform.rotation.x;
  quat.v[2] = t.transform.rotation.y;
  quat.v[3] = t.transform.rotation.z;
  att = Quat2RPY(quat);
  if(!update)
  	update=true;
}
void messageCallbackTarget( geometry_msgs::TransformStamped t){
	std::string a =  t.header.frame_id;//currently unused
  Vec4 quat;
	tarLoc.v[0] = t.transform.translation.x;
	tarLoc.v[1] = t.transform.translation.y;
	tarLoc.v[2] = t.transform.translation.z;
	quat.v[0] = t.transform.rotation.w;
	quat.v[1] = t.transform.rotation.x;
	quat.v[2] = t.transform.rotation.y;
	quat.v[3] = t.transform.rotation.z;
	tarAtt = Quat2RPY(quat);
	if(!update)
		update=true;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	viconSub=nh.subscribe("/vicon/orbot/orbot",1000,messageCallbackVicon);
	targetSub=nh.subscribe("/orbot_client/target",1000,messageCallbackTarget);
	
	update=true;

	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
	ser2.SetBaudRate(SerialPort::BAUD_115200);

	ros::Rate loop_rate(10);//TODO: spiit up the updating and writing rates with time class
	float rates[4];
	float dx,dy,dTheta;
	while(ros::ok()){
		//convert delta positions to velocities somehow
		//divide all by 2 for right now	
		if(update){
			update=false;
			std::cout<<"Updating orbot\n";//TODO: preserve ratios

			float dx0 = (float)(tarLoc.v[0] - loc.v[0]);
	 	  float dy0 = (float)(tarLoc.v[1] - loc.v[1]);
	 	  dTheta  = (float)(tarAtt.v[2] - att.v[2]);

	 	  dx=dx0*cos(dTheta)-dy0*sin(dTheta);
	 	  dy=dx0*sin(dTheta)+dy0*cos(dTheta);

	 	  float vx,vy,vTheta;
	 	  vx=dx;
	 	  vy=dy;
	 	  vTheta=dTheta;
			if(fabs(dx)>vMax/2){
				//vx=(dx>0?1:-1)*vMax/2*(fabs(dy)>fabs(dx)?fabs(dx/dy):1);
				errorIntX+=dx;
				vx=P*dx + I*errorIntX +  D*(dx-errorLastX);
				errorLastX = dx;
			}
			if(fabs(dy)>vMax/2){
				//vy=(dy>0?1:-1)*vMax/2*(fabs(dx)>fabs(dy)?fabs(dy/dx):1);
				errorIntY+=dy;
				vy=P*dy + I*errorIntY + D*(dy-errorLastY);
				errorLastY = dy;
			}
			if(fabs(dTheta)>vMax/.04)//.02 = (r_wheels x v_wheels)/v_wheels 
				vTheta=P*dTheta;//(dTheta>0?1:-1)*vMax/.04; TODO: FIX THIS
			getRotationRates(rates,vx,vy,vTheta);

			for(int i=0;i<4;i++){
				rates[i]=rates[i]/pi/2*60;
				if(rates[i]>120)
					rates[i]=120;
				if(rates[i]<-120)
					rates[i]=-120;
				rates[i]=round(rates[i]/122*1000);
			}
		}
		std::cout<<"Deltas are: "<<dx<<"\t"<<dy<<"\t"<<dTheta<<"\n";
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser1.Write(write);
		std::cout<<"wrote "<<write<<"\n";
			
		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser1.Write(write);
		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser2.Write(write);
		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		ser2.Write(write);
		write[len]='\0';
		std::cout<<"wrote "<<write<<"\n";
//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();		
	}
	ser1.Close();
	ser2.Close();
	return 0;
}
