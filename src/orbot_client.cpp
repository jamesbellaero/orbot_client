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
#include "kinova_driver/kinova_comm.h"
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/HomeArmRequest.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <SerialPort.h>//to install, use sudo apt-get install libserial-dev
#include "orbot_utils.h"
#include "eigen3/Eigen/Dense"

using namespace kinova;
ros::Subscriber viconSub,targetSub,stateSub;
bool update;
//constants for PID's, etc
const float pi=3.14159265358979;
const float wMax=122*2*pi/60;
const float vMax=.0762*wMax/1.424;// m/s
const float P=2;
const float I=.01;//0.01;
const float D=0;//1;
const float minDelta=.01;//minimum distance before moving
const float kp = 1;

const float ki = 4;
const float kd = 1;

float eIntX,eLastX,eIntY,eLastY,eLastTheta,eIntTheta;
KinovaPose position;
CartesianInfo cartPos;


Vec3 tarLoc;
Vec3 tarAtt;
Vec3 loc;
Vec3 att;	
float timeperiod = 30;
float omega = 2*pi/timeperiod;
float radius = 1;

double xr(double t)
{
	return radius*cos(t*omega);
}

double yr(double t)
{
	return radius*sin(t*omega);
}

double xrdot(double t)
{
	return -radius*omega*sin(t*omega);
}
double yrdot(double t)
{
	return radius*omega*cos(t*omega);
}

double xrdotdot(double t)
{
	return -radius*omega*omega*cos(t*omega);
}
double yrdotdot(double t)
{
	return -radius*omega*omega*sin(t*omega);
}
/*
double xr(double t)
{
	return 0.7;
}

double yr(double t)
{
	return 0.7;
}

double xrdot(double t)
{
	return 0;
}
double yrdot(double t)
{
	return 0;
}

double xrdotdot(double t)
{
	return 0;
}
double yrdotdot(double t)
{
	return 0;
}
*/
void messageCallbackVicon( geometry_msgs::TransformStamped t){
  // std::string a =  t.header.frame_id;//currently unused
  // //defined in quaternion.h		
  // Vec4 quat;
  // loc.v[0] = t.transform.translation.x;
  // loc.v[1] = t.transform.translation.y;
  // loc.v[2] = t.transform.translation.z;
  // quat.v[0] = t.transform.rotation.w;
  // quat.v[1] = t.transform.rotation.x;
  // quat.v[2] = t.transform.rotation.y;
  // quat.v[3] = t.transform.rotation.z;
  // att = Quat2RPY(quat);
  // if(!update)
  // 	update=true;
}

void messageCallbackState( robot_controller::State msg){
  //defined in quaternion.h		
  Vec4 quat;
	for(int i=0;i<3;i++){
		loc.v[i]=msg.r[i];
		quat.v[i+1]=msg.q[i];
	}
	quat.v[0]=msg.q[3];
  att = Quat2RPY(quat);
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
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser1,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		writeToPort(ser2,write);
		//std::cout<<"wrote "<<write<<"\n";
}

int main(int argc, char** argv){

	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	robotName = nh.getParam("RobotName",robotName);
	viconSub=nh.subscribe("/vicon/Orbot/Orbot",1000,messageCallbackVicon);
	stateSub=nh.subscribe(robotName+std::string("/state"),1000,messageCallbackState);
	//targetSub=nh.subscribe("/orbot_client/target",1000,messageCallbackTarget);
	bool firstIter=true;
	bool translate=true;//when false, rotate
	update=true;
	ros::ServiceClient homeclient = nh.serviceClient<kinova_msgs::HomeArm>("/m1n6s300_driver/in/home_arm");
	ros::ServiceClient client = nh.serviceClient<kinova_msgs::AddPoseToCartesianTrajectory>("/m1n6s300_driver/in/add_pose_to_Cartesian_trajectory");
	if(homeclient)
	{
		std::cout<<"Home client created\n";
	}

	if(client)
	{
		std::cout<<"Client Created\n";
	}
	
	// if(homeclient.call())
	// {
	// 	std::cout<<"Arm homed\n";
	// }

	kinova_msgs::AddPoseToCartesianTrajectory add_pose;
	add_pose.request.X = 1;
	add_pose.request.Y = 1;
	add_pose.request.Z = 1;
	add_pose.request.ThetaX = 10;
	add_pose.request.ThetaY = 30;
	add_pose.request.ThetaZ = 0;


	if (client.call(add_pose))
	{
		std::cout<<"Service success"<<std::endl;
	}
	else
	{
		std::cout<<"service failed"<<std::endl;
	}
	
	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
	ser2.SetBaudRate(SerialPort::BAUD_115200);
	
	ros::Rate loop_rate(30);//TODO: spiit up the updating and writing rates with time class
	float rates[4];
	float dx,dy,dTheta;
	ros::Time prevWrite=ros::Time::now();
	ros::Time start_time=ros::Time::now();
	ros::Duration writeDelay(.01);
	ros::Duration d;
	
	double t;
	
	Eigen::Matrix3f R1(3,3);
		Eigen::Matrix3f R2(3,3);
		Eigen::Vector3f Xr,Xrdot,Xc,z,IXe,DXe;
		Xc(0) = loc.v[0];
		Xc(1) = loc.v[1];
		Xc(2) = att.v[2];
		IXe(0) = 0;
		IXe(1) = 0;
		IXe(2) = 0;
		DXe(0) = 0;
		DXe(1) = 0;
		DXe(2) = 0;
		double dt = 1/30;

		// Xc(loc.v[0], loc.v[1], att.v[2]);

	while(ros::ok()){
		//convert delta positions to velocities somehow
		//divide all by 2 for right now	
		bool write=false;
		if(update){
			update=false;
			write=true;
			
			Xc(0) = loc.v[0];
			Xc(1) = loc.v[1];
			Xc(2) = att.v[2];
			//determine delta
			float dx0 = (float)(tarLoc.v[0] - loc.v[0]);
			float dy0 = (float)(tarLoc.v[1] - loc.v[1]);
			dTheta  = 0;//(float)(tarAtt.v[2] - att.v[2]);
			float vx,vy,vTheta;
			//rotate from global to local reference
			dx=dx0*cos(dTheta)-dy0*sin(dTheta);
			dy=dx0*sin(dTheta)+dy0*cos(dTheta);

	 	    double c = cos(Xc(2));
			double s = sin(Xc(2));
			//Matrix3x3 R1(c,-s,0,s,c,0,0,0,1);
			//Matrix3x3 R2(c,s,-c*(yr(t)-yc)+s*(xr(t)-xc),-s,c,c*(xr(t)-xc)+s*(yr(t)-yc),0,0,1);
		
			
			d = (ros::Time::now()-start_time);
			t = d.toSec();
			//t = time since program start;
			std::cout<<"Time= "<<t<<std::endl;
			Xr(0) = xr(t);
			Xr(1) = yr(t);
			Xrdot(0) = xrdot(t);
			Xrdot(1) = yrdot(t);
			//Xr(2) = atan2(Xrdot(1),Xrdot(0));
			Xr(2) = Xc(2);
			Xrdot(2) = 0;
			//Xrdot(2) = (Xrdot(0)*yrdotdot(t) - Xrdot(1)*xrdotdot(t))/(Xrdot(0)*Xrdot(0) + Xrdot(1)*Xrdot(1));
			double Ie0 = IXe(0);
			double Ie1 = IXe(1);
			IXe(0) += (Xr(0) - Xc(0))*dt;
			IXe(1) += (Xr(1) - Xc(1))*dt;
			if(sqrt(IXe(0)*IXe(0) + IXe(1)*IXe(1))<1)
			{
				IXe(0) = Ie0;
				IXe(1) = Ie1;
			}

			R1 << c,-s,0,
					s,c,0,
					0,0,1;
			R2 << c ,s ,-c*(Xr(1)-Xc(1))+s*(Xr(0)-Xc(0)),
			      -s ,c ,c*(Xr(0)-Xc(0))+s*(Xr(1)-Xc(1)),
			      0, 0 ,1;
			
			
			std::cout<<"Current Pose: "<<Xc.transpose()<<std::endl;
			std::cout<<"Expected Pose: "<<Xr.transpose()<<std::endl;
			std::cout<<"Difference: "<<(Xr-Xc).transpose()<<std::endl;
			//Eigen::Vector3f Xrr;
			//Xrr << 1,1,0;
			//std::cout<<Xrr<<std::endl;
			//z = R1.inverse()*R2.inverse()*R1.inverse()*Xrdot + k*R1.inverse()*R2.inverse()*(Xr-Xc);
			z(0) = Xrdot(0) + kp*(Xr(0)-Xc(0)) + ki*IXe(0) + kd*(Xr(0)-Xc(0) - DXe(0));
			z(1) = Xrdot(1) + kp*(Xr(1)-Xc(1)) + ki*IXe(1) + kd*(Xr(1)-Xc(1) - DXe(1));
			//z(2) = 0 + k*(0-Xc(2));
			z(2) = Xrdot(2) + kp*(Xr(2)-Xc(2));
			DXe(0) = Xr(0)-Xc(0);
			DXe(1) = Xr(1)-Xc(1);
			DXe(2) = Xr(2)-Xc(2);

			std::cout<<z<<std::endl;
			std::cout<<"Vx= "<<(z(0)*c + z(1)*s)<<" Vy = "<<(z(0)*s - z(1)*c)<<" W= "<<z(2)<<std::endl;
			vx = z(0)*c + z(1)*s;
			vy = (z(0)*s - z(1)*c);
			vTheta = 0;//z(2);
	 	    float vTotal=sqrt(pow(vx,2)+pow(vy,2)+pow(vTheta,2));
			if(vTotal>vMax/1.5){//normalize by maximum velocity
				vx*=vMax/1.5/vTotal;
				vy*=vMax/1.5/vTotal;
				vTheta*=vMax/1.5/vTotal;
			}
			if(translate && sqrt(pow(dx0,2)+pow(dy0,2))<.05){
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
			//std::cout<<"Velocities: "<<vx<<"\t"<<vy<<"\t"<<vTheta<<"\n";//<<"With errors: "<<eIntX<<"\t"<<eIntY<<"\t"<<eIntTheta<<"\n";
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
			add_pose.request.X = c;
	add_pose.request.Y = s;
	add_pose.request.Z = 1;
	add_pose.request.ThetaX = 10;
	add_pose.request.ThetaY = 30;
	add_pose.request.ThetaZ = 0;
		}
		if(write || ros::Time::now()-prevWrite>writeDelay){
			//std::cout<<"Deltas are: "<<dx<<"\t"<<dy<<"\t"<<dTheta<<"\n";
			write=false;
			writeDeltas(&ser1,&ser2,rates); 
			
			
if (client.call(add_pose))
	{
		std::cout<<"Service success"<<std::endl;
	}
	else
	{
		std::cout<<"service failed"<<std::endl;
	}
			prevWrite=ros::Time::now();

		}
	
		
		ros::spinOnce();
		loop_rate.sleep();		
	}
	//ser1.Close(); Rover Uncomment
	//ser2.Close(); Rover Uncomment

	return 0;
}
