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
	dx=vec.x;
	dy=vec.y;
	dTheta=vec.z;
	update=true;
}
void setupSerial(LibSerial::SerialStream *ser){
	using namespace LibSerial;
	ser->SetBaudRate(SerialStreamBuf::BAUD_115200);
	ser->SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	ser->SetNumOfStopBits(1);
	ser->SetParity(SerialStreamBuf::PARITY_NONE);
	ser->SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
}
int main(int argc, char** argv){
	ros::init(argc,argv,"orbot_client");
	ros::NodeHandle nh;
	sub=nh.subscribe("orbot_server/orbot_delta",1000,messageCallback);
	
	update=true;

	using namespace LibSerial;
	SerialPort ser1("/dev/ttyACM0");
	ser1.Open();//may be incorrect ports
	SerialPort ser2("/dev/ttyACM1");
	ser2.Open();
//	setupSerial(&ser1);
//	setupSerial(&ser2);
	ser1.SetBaudRate(SerialPort::BAUD_115200);
	ser2.SetBaudRate(SerialPort::BAUD_115200);
	int count=0;
	ser1.Write("!g 1 400\r");
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//convert delta positions to velocities somehow
		//divide all by 2 for right now
		
	

		if(update){
			update=false;
			if(abs(dx)>vMax/2)
				dx=vMax/2;
			if(abs(dy)>vMax/2)
				dy=vMax/2;
			if(abs(dTheta)>vMax/.04)//.02 = (r_wheels x v_wheels)/v_wheels 
				dTheta=vMax/.04;
			float rates[4];
			getRotationRates(rates,dx,dy,dTheta);
			std::cout<<count++;
			for(int i=0;i<4;i++){
				rates[i]=rates[i]/pi/2*60;
				if(rates[i]>122)
					rates[i]=122;
				if(rates[i]<-122)
					rates[i]=-122;
				rates[i]=round(rates[i]/122*1000);
			}
			std::cout<<count++;
			char output_buffer[64];
			int len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[0]);
			char* write =(char*) malloc(len+1);
			strncpy(write,output_buffer,len);
			ser1.Write(write);

			len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[1]);
			write =(char*) malloc(len+1);
			strncpy(write,output_buffer,len);
			ser1.Write(write);

			len=sprintf(output_buffer,"!g 1 %d\r",(int)rates[2]);
			write =(char*) malloc(len+1);
			strncpy(write,output_buffer,len);
			ser2.write(write);

			len=sprintf(output_buffer,"!g 2 %d\r",(int)rates[3]);
			write =(char*) malloc(len+1);
			strncpy(write,output_buffer,len);
			ser2.write(write);

			for(int i=0;i<4;i++){
				std::cout<<rates[i]<<"\n"
			}
			std::cout<<"Wrote all rates";
		}
		loop_rate.sleep();
		
	}
	ser1.Close();
	ser2.Close();
	return 0;
}
