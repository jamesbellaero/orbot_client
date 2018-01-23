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
	1. 	Could try and ensure rotation is mostly taken care of by the time that the bot reaches within a certain radius of its goal, 
		then fix rotation and translation separately until it converges. This involves giving rotation a superficially large priority based
		on distance to the goal (pretty much a multiplier, just need to ensure it's less than 42/14 times y at all times).
	2.	Could also attempt to have it run regularly until it's less than .5 meters out then go to translation and rotation separately
*/
#include <ros/ros.h>
#include <time.h>
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
//constants for PID's, etc
const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
const float P=2;
const float I=.01;//0.01;
const float D=0;//1;
const float minDelta=.01;//minimum distance before moving

float eIntX,eLastX,eIntY,eLastY,eLastTheta,eIntTheta;

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

void writeToPort(SerialPort *ser, char* write){
	bool written=false;
	while(!written){
		try{
			ser->Write(write);
			written=true;
		}
		catch(std::exception& e){
			continue;
		}
	}
}
void writeDeltas(SerialPort *ser1, SerialPort *ser2, float *rates){
		
		//TODO: make this more straightforward
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		std::cout<<"wrote "<<write<<"\n";
}

int main(int argc, char** argv){
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	viconSub=nh.subscribe("/vicon/orbot/orbot",1000,messageCallbackVicon);
	targetSub=nh.subscribe("/orbot_client/target",1000,messageCallbackTarget);
	bool firstIter=true;
	bool translate=true;//when false, rotate
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
	ros::Time prevWrite=ros::Time::now();
	ros::Duration writeDelay(.8);
	while(ros::ok()){
		//convert delta positions to velocities somehow
		//divide all by 2 for right now	
		bool write=false;
		if(update){
			update=false;
			write=true;
			std::cout<<"Updating orbot\n";

			//determine delta
			float dx0 = (float)(tarLoc.v[0] - loc.v[0]);
	 	  float dy0 = (float)(tarLoc.v[1] - loc.v[1]);
	 	  dTheta  = (float)(tarAtt.v[2] - att.v[2]);

	 	  //rotate from global to local reference
	 	  dx=dx0*cos(dTheta)-dy0*sin(dTheta);
	 	  dy=dx0*sin(dTheta)+dy0*cos(dTheta);

	 	  if(firstIter && fabs(dx)>.00001){
				eLastX=dx;
				eLastY=dy;
				eLastTheta=dTheta;
				translate=true;
				firstIter=false;
			}

	 	  float vx,vy,vTheta;
	 	  //pid stuff here
			if(translate){
				vx=P*dx+ I*eIntX +  (D)*(dx-eLastX);
				eIntX+=fabs(eIntX+dx)>fabs(eIntX)&&fabs(eIntX+dx)>fabs(P*dx/I)?0:dx;//don't increment if too high already
				eLastX = dx;

				//TODO: MAKE THIS NON-NEGATIVE P
				vy=-(P*dy+ I*eIntY + (D)*(dy-eLastY));
				eIntY+=fabs(eIntY+dy)>fabs(eIntY)&&fabs(eIntY+dy)>fabs(P*dy/I)?0:dy;
				eLastY = dy;
			}
			else{
				vTheta=P*dTheta+I*eIntTheta+(D)*(dTheta-eLastTheta);
				eIntTheta+=fabs(eIntTheta+dTheta)>fabs(eIntTheta)&&fabs(eIntTheta+dTheta)>fabs(P*dTheta/I)?0:dTheta;
				eLastTheta = dTheta;
			}
			
	 	  float vTotal=sqrt(pow(vx,2)+pow(vy,2)+pow(vTheta,2));
			if(vTotal>vMax/1.5){//normalize by maximum velocity
				vx*=vMax/1.5/vTotal;
				vy*=vMax/1.5/vTotal;
				vTheta*=vMax/1.5/vTotal;
			}
			if(translate&&sqrt(pow(dx0,2)+pow(dy0,2))<.05){
				vx=0;
				vy=0;
				eIntX=0;
				eIntY=0;
				eLastTheta=dTheta;
				translate=false;
			}
			if(!translate&&fabs(dTheta)<.01){
				vTheta=0;
				eIntTheta=0;
				eLastX=dx;
				eLastY=dy;
				translate=true;
			}
			if(sqrt(pow(dx0,2)+pow(dy0,2)+pow(dTheta,2))<.05){
				vx=0;
				vy=0;
				vTheta=0;
				firstIter=true;
			}
			std::cout<<"Velocities are: "<<vx<<"\t"<<vy<<"\t"<<vTheta<<"\n"<<"With errors: "<<eIntX<<"\t"<<eIntY<<"\t"<<eIntTheta<<"\n";
			getRotationRates(rates,vx,vy,vTheta);
			std::cout<<"Rates are: "<<rates[0]<<"\t"<<rates[1]<<"\t"<<rates[2]<<"\t"<<rates[3]<<"\n";
			for(int i=0;i<4;i++){
				rates[i]=rates[i]/pi/2*60;
				if(rates[i]>120)
					rates[i]=120;
				if(rates[i]<-120)
					rates[i]=-120;
				rates[i]=round(rates[i]/122*1000);
			}
		}
		if(write || ros::Time::now()-prevWrite>writeDelay){
			std::cout<<"Deltas are: "<<dx<<"\t"<<dy<<"\t"<<dTheta<<"\n";
			write=false;
			writeDeltas(&ser1,&ser2,rates);
			prevWrite=ros::Time::now();
		}
		
//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();		
	}
	ser1.Close();
	ser2.Close();
	return 0;
}
