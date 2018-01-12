#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <SerialStream.h>//to install, use sudo apt-get install libserial-dev
#include <SerialPort.h>
#include "orbot_utils.h"

ros::Subscriber sub;
float dx,dy,dTheta;
bool update;
const float pi=3.14159265359;
const float wMax=122*2*pi/60;
const float vMax=.00762*wMax/1.424;// m/s
void messageCallback(geometry_msgs::Vector3 vec){
	ROS_INFO("A Callback!!!");
	dx=vec.x;
	dy=vec.y;
	dTheta=vec.z;
	update=true;
}
int main(int argc, char** argv){
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	sub=nh.subscribe("/orbot_server/orbot_delta",1000,messageCallback);
	
	update=true;

	using namespace LibSerial;
	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
	ser2.SetBaudRate(SerialPort::BAUD_115200);

	ros::Rate loop_rate(2);//TODO: spiit up the updating and writing rates with time class
	float rates[4];
	while(ros::ok()){
		//convert delta positions to velocities somehow
		//divide all by 2 for right now	
		if(update){
			update=false;
			std::cout<<"Updating orbot\n";
			if(abs(dx)>vMax/2)
				dx=(dx>0?1:-1)*vMax/2;
			if(abs(dy)>vMax/2)
				dy=(dy>0?1:-1)*vMax/2;
			if(abs(dTheta)>vMax/.04)//.02 = (r_wheels x v_wheels)/v_wheels 
				dTheta=(dTheta>0?1:-1)*vMax/.04;

			getRotationRates(rates,dx,dy,dTheta);

			for(int i=0;i<4;i++){
				rates[i]=rates[i]/pi/2*60;
				if(rates[i]>122)
					rates[i]=122;
				if(rates[i]<-122)
					rates[i]=-122;
				rates[i]=round(rates[i]/122*1000);
			}
		}
		char output_buffer[64];
		int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
		char* write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser1.Write(write);
//		std::cout<<"wrote "<<write<<"\n";
			
		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser1.Write(write);
//		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		write[len]='\0';
		ser2.Write(write);
//		std::cout<<"wrote "<<write<<"\n";

		len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
		write =(char*) malloc(len+1);
		strncpy(write,output_buffer,len);
		ser2.Write(write);
		write[len]='\0';
//		std::cout<<"wrote "<<write<<"\n";
	//	for(int i=0;i<4;i++){
	//		std::cout<<rates[i]<<"\n";
	//	}
//		std::cout<<"Wrote all rates\n";
		
//		loop_rate.sleep();
		ros::spinOnce();
		loop_rate.sleep();		
	}
	ser1.Close();
	ser2.Close();
	return 0;
}
