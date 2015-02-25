//
//  teleop_save.cpp
//  
//
//  Created by Carmine Tommaso Recchiuto on 13/11/14.
//
//

#include "teleop_save.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"

double x_real_pos[10], z_real_pos[10], y_real_pos[10], w_real_pos[10];
double x_noobst_pos[10], z_noobst_pos[10], y_noobst_pos[10];
double accum_time=0;
double form_orient, com_x, com_y;
double starting_time;
int saving=0, counta=0, ide1, ide2, ide3, counta2=0;
double contaloop=0.0, totalloop=0.0;
  std::string result;
ros::Publisher pubb, pubb2;
struct timeval starttime2, endtime2;

using namespace std;

void initializeDataSaving(){
    
    //char test[10]="positions";
    std::stringstream sstm;
    sstm << "positions_exp_" << ide1 << "_" << ide2 << "_" << ide3 << ".txt";
    result=sstm.str();
    const char *cha = result.c_str();

    ofstream fs2(cha, std::ofstream::out | std::ofstream::trunc);
    
    if (fs2.is_open()){
        fs2 << "Salviamo i dati!!" << "\n";
        fs2 << "time \t";
        fs2 << "x0 \t";
        fs2 << "y0 \t";
        fs2 << "z0 \t";
        fs2 << "x1 \t";
        fs2 << "y1 \t";
        fs2 << "z1 \t";
        fs2 << "x2 \t";
        fs2 << "y2 \t";
        fs2 << "z2 \t";
        fs2 << "x3 \t";
        fs2 << "y3 \t";
        fs2 << "z3 \t";
        fs2 << "x4 \t";
        fs2 << "y4 \t";
        fs2 << "z4 \t";
        fs2 << "x5 \t";
        fs2 << "y5 \t";
        fs2 << "z5 \t";
        fs2 << "x6 \t";
        fs2 << "y6 \t";
        fs2 << "z6 \t";
        fs2 << "x7 \t";
        fs2 << "y7 \t";
        fs2 << "z7 \t";
        fs2 << "x8 \t";
        fs2 << "y8 \t";
        fs2 << "z8 \t";
        fs2 << "x9 \t";
        fs2 << "y9 \t";
        fs2 << "z9 \t";
	
	fs2 << "x0_no \t";
        fs2 << "y0_no \t";
        fs2 << "z0_no \t";
        fs2 << "x1_no  \t";
        fs2 << "y1_no  \t";
        fs2 << "z1_no  \t";
        fs2 << "x2_no  \t";
        fs2 << "y2_no  \t";
        fs2 << "z2_no  \t";
        fs2 << "x3_no  \t";
        fs2 << "y3_no  \t";
        fs2 << "z3_no  \t";
        fs2 << "x4_no  \t";
        fs2 << "y4_no  \t";
        fs2 << "z4_no  \t";
        fs2 << "x5_no  \t";
        fs2 << "y5_no  \t";
        fs2 << "z5_no  \t";
        fs2 << "x6_no  \t";
        fs2 << "y6_no  \t";
        fs2 << "z6_no  \t";
        fs2 << "x7_no  \t";
        fs2 << "y7_no  \t";
        fs2 << "z7_no  \t";
        fs2 << "x8_no  \t";
        fs2 << "y8_no  \t";
        fs2 << "z8_no  \t";
        fs2 << "x9_no  \t";
        fs2 << "y9_no  \t";
        fs2 << "z9_no  \t";

	fs2 << "x0_no_t \t";
        fs2 << "y0_no_t \t";
        fs2 << "z0_no_t \t";
        fs2 << "x1_no_t \t";
        fs2 << "y1_no_t \t";
        fs2 << "z1_no_t \t";
        fs2 << "x2_no_t \t";
        fs2 << "y2_no_t \t";
        fs2 << "z2_no_t \t";
        fs2 << "x3_no_t \t";
        fs2 << "y3_no_t \t";
        fs2 << "z3_no_t \t";
        fs2 << "x4_no_t \t";
        fs2 << "y4_no_t \t";
        fs2 << "z4_no_t \t";
        fs2 << "x5_no_t \t";
        fs2 << "y5_no_t \t";
        fs2 << "z5_no_t \t";
        fs2 << "x6_no_t \t";
        fs2 << "y6_no_t \t";
        fs2 << "z6_no_t \t";
        fs2 << "x7_no_t \t";
        fs2 << "y7_no_t \t";
        fs2 << "z7_no_t \t";
        fs2 << "x8_no_t \t";
        fs2 << "y8_no_t \t";
        fs2 << "z8_no_t \t";
        fs2 << "x9_no_t \t";
        fs2 << "y9_no_t \t";
        fs2 << "z9_no_t \t";

        fs2 << "\n";
        
        cout << "File with controls initialized" << endl;
    }
    else {
        cerr << "File not opened!" << endl;
    }
}

void sync_received(const std_msgs::Int32::ConstPtr& msg)
{
  saving=msg->data;
}

void info1_received(const std_msgs::Int32::ConstPtr& msg)
{
  ide1=msg->data;
}

void info2_received(const std_msgs::Int32::ConstPtr& msg)
{
  ide2=msg->data;
}

void info3_received(const std_msgs::Int32::ConstPtr& msg)
{
  ide3=msg->data;
}



void ROSnoobst0_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[0]=msg->pose.position.x;
    y_noobst_pos[0]=msg->pose.position.y;
    z_noobst_pos[0]=msg->pose.position.z;
}

void ROSnoobst1_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[1]=msg->pose.position.x;
    y_noobst_pos[1]=msg->pose.position.y;
    z_noobst_pos[1]=msg->pose.position.z;
}

void ROSnoobst2_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[2]=msg->pose.position.x;
    y_noobst_pos[2]=msg->pose.position.y;
    z_noobst_pos[2]=msg->pose.position.z;
}

void ROSnoobst3_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[3]=msg->pose.position.x;
    y_noobst_pos[3]=msg->pose.position.y;
    z_noobst_pos[3]=msg->pose.position.z;
}

void ROSnoobst4_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[4]=msg->pose.position.x;
    y_noobst_pos[4]=msg->pose.position.y;
    z_noobst_pos[4]=msg->pose.position.z;
}

void ROSnoobst5_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[5]=msg->pose.position.x;
    y_noobst_pos[5]=msg->pose.position.y;
    z_noobst_pos[5]=msg->pose.position.z;
}

void ROSnoobst6_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[6]=msg->pose.position.x;
    y_noobst_pos[6]=msg->pose.position.y;
    z_noobst_pos[6]=msg->pose.position.z;
}

void ROSnoobst7_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[7]=msg->pose.position.x;
    y_noobst_pos[7]=msg->pose.position.y;
    z_noobst_pos[7]=msg->pose.position.z;
}

void ROSnoobst8_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[8]=msg->pose.position.x;
    y_noobst_pos[8]=msg->pose.position.y;
    z_noobst_pos[8]=msg->pose.position.z;
}

void ROSnoobst9_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_noobst_pos[9]=msg->pose.position.x;
    y_noobst_pos[9]=msg->pose.position.y;
    z_noobst_pos[9]=msg->pose.position.z;
}

void ROSorient_received(const std_msgs::Float32::ConstPtr& msg)
{
    form_orient=msg->data;
}

void ROScomx_received(const std_msgs::Float32::ConstPtr& msg)
{
    com_x=msg->data;
	if (com_x==200.0)
	{
	//std::cout << "BOH" << std::endl;
	double perc = contaloop / totalloop;
	std::cout << "TOTAL TIME = " << accum_time-starting_time << std::endl;
	std::cout << "OUT OF FORMATION = " << perc*100 << " %" << std::endl;
	ros::shutdown();	
	}
	
}

void ROScomy_received(const std_msgs::Float32::ConstPtr& msg)
{
    com_y=msg->data;
}


void ROSrealposition0_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[0]=msg->pose.position.x;
    y_real_pos[0]=msg->pose.position.y;
    z_real_pos[0]=msg->pose.position.z;
    w_real_pos[0]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition1_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[1]=msg->pose.position.x;
    y_real_pos[1]=msg->pose.position.y;
    z_real_pos[1]=msg->pose.position.z;
    w_real_pos[1]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition2_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[2]=msg->pose.position.x;
    y_real_pos[2]=msg->pose.position.y;
    z_real_pos[2]=msg->pose.position.z;
    w_real_pos[2]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition3_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[3]=msg->pose.position.x;
    y_real_pos[3]=msg->pose.position.y;
    z_real_pos[3]=msg->pose.position.z;
    w_real_pos[3]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition4_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[4]=msg->pose.position.x;
    y_real_pos[4]=msg->pose.position.y;
    z_real_pos[4]=msg->pose.position.z;
    w_real_pos[4]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition5_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[5]=msg->pose.position.x;
    y_real_pos[5]=msg->pose.position.y;
    z_real_pos[5]=msg->pose.position.z;
    w_real_pos[5]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition6_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[6]=msg->pose.position.x;
    y_real_pos[6]=msg->pose.position.y;
    z_real_pos[6]=msg->pose.position.z;
    w_real_pos[6]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition7_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[7]=msg->pose.position.x;
    y_real_pos[7]=msg->pose.position.y;
    z_real_pos[7]=msg->pose.position.z;
    w_real_pos[7]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition8_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[8]=msg->pose.position.x;
    y_real_pos[8]=msg->pose.position.y;
    z_real_pos[8]=msg->pose.position.z;
    w_real_pos[8]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));
}

void ROSrealposition9_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[9]=msg->pose.position.x;
    y_real_pos[9]=msg->pose.position.y;
    z_real_pos[9]=msg->pose.position.z;
    w_real_pos[9]=atan2((2*((msg->pose.orientation.w*msg->pose.orientation.z)+(msg->pose.orientation.x*msg->pose.orientation.y))),(1-2*((msg->pose.orientation.y*msg->pose.orientation.y)+(msg->pose.orientation.z*msg->pose.orientation.z))));


if (saving==1)
{
if (counta ==0)
{
 initializeDataSaving();
 counta=1;
}
else
{
double real_xom = 0;
double real_yom = 0;
    for (int i=0;i<10;i++)
	{
	real_xom = real_xom+x_real_pos[i];
	real_yom = real_yom+y_real_pos[i];
	}
real_xom = real_xom/10.0;
real_yom = real_yom/10.0;

   double secc, msec, mtime;
    
    gettimeofday(&endtime2, NULL);
    secc=endtime2.tv_sec - starttime2.tv_sec;
    msec=endtime2.tv_usec- starttime2.tv_usec;
    mtime = ((secc)+(msec/1000000.0)); //time in sec
    accum_time=accum_time+mtime;
    gettimeofday(&starttime2, NULL);

	if (counta2==0)
	{starting_time=accum_time;
	 counta2=1;
	}
 
 
    //// eventuale elaborazione sui dati

    /*double max_dist = 0;	
    double dist_int;

	for (int i=0;i<10;i++)
	{	double dist=1000;
		dist_int = fabs(y_real_pos[i] - 0.15);
		if (dist_int<dist)
		{
		dist=dist_int;
		}
		dist_int = fabs(y_real_pos[i] - 19.3);
		if (dist_int<dist)
		{
		dist=dist_int;
		}
		dist_int = fabs(x_real_pos[i] + 1);
		if (dist_int<dist)
		{
		dist=dist_int;
		}
		dist_int = fabs(x_real_pos[i] - 15.4);
		if (dist_int<dist)
		{
		dist=dist_int;
		}
		if (dist > max_dist)
		{
		max_dist=dist;
		}
	}*/

		

    double x_disp = real_xom - com_x;
    double y_disp = real_yom - com_y;


   double x_noobst_pos_transl[10];
   double y_noobst_pos_transl[10];
    
	for (int i=0;i<10;i++)
	{
	x_noobst_pos_transl[i]= x_noobst_pos[i]+x_disp;
	y_noobst_pos_transl[i]= y_noobst_pos[i]+y_disp;
	}

    //std::cout << max_dist << std::endl;

    const char *cha = result.c_str();

    ofstream fs2(cha, std::ofstream::out | std::ofstream::app);

    
    if(fs2.is_open()){
        fs2 << accum_time << "\t";
        for (int i =0;i<10;i++)
        {
        fs2 << x_real_pos[i] << "\t" << y_real_pos[i] << "\t" << z_real_pos[i] << "\t";
        }
	for (int i =0;i<10;i++)
        {
        fs2 << x_noobst_pos[i] << "\t" << y_noobst_pos[i] << "\t" << z_noobst_pos[i] << "\t";
        }
	for (int i =0;i<10;i++)
        {
	fs2 << x_noobst_pos_transl[i] << "\t" << y_noobst_pos_transl[i] << "\t" << z_noobst_pos[i] << "\t";
	}
	//fs2 << max_dist;
        fs2 << "\n";
    }
    else {
        std::cerr << "ERRORE nella scrittura del file!" << std::endl;
    }
	
	  std_msgs::Float32 timetime;
	timetime.data=accum_time-starting_time;
	pubb.publish(timetime);

totalloop=totalloop+1.0;

int p = 0;
for (int i=0;i<10;i++)
	{
	if (((x_real_pos[i]-x_noobst_pos_transl[i])*(x_real_pos[i]-x_noobst_pos_transl[i])+(y_real_pos[i]-y_noobst_pos_transl[i])*(y_real_pos[i]-y_noobst_pos_transl[i]))>0.8)
		{
			p=1;
		}
	}

	std_msgs::Float32 format;
	if (p==0)
	{format.data=0.0;}
	if (p==1)
	{format.data=1.0;
	contaloop=contaloop+1.0;}
	pubb2.publish(format);
   }}
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "saver");
    
    ros::NodeHandle n_j;
   

    gettimeofday(&starttime2, NULL);
    
    ros::Subscriber quadcop_realPos0_subj = n_j.subscribe("quadcop_realPos_0", 100, ROSrealposition0_received);
    ros::Subscriber quadcop_realPos1_subj = n_j.subscribe("quadcop_realPos_1", 100, ROSrealposition1_received);
    ros::Subscriber quadcop_realPos2_subj = n_j.subscribe("quadcop_realPos_2", 100, ROSrealposition2_received);
    ros::Subscriber quadcop_realPos3_subj = n_j.subscribe("quadcop_realPos_3", 100, ROSrealposition3_received);
    ros::Subscriber quadcop_realPos4_subj = n_j.subscribe("quadcop_realPos_4", 100, ROSrealposition4_received);
    ros::Subscriber quadcop_realPos5_subj = n_j.subscribe("quadcop_realPos_5", 100, ROSrealposition5_received);
    ros::Subscriber quadcop_realPos6_subj = n_j.subscribe("quadcop_realPos_6", 100, ROSrealposition6_received);
    ros::Subscriber quadcop_realPos7_subj = n_j.subscribe("quadcop_realPos_7", 100, ROSrealposition7_received);
    ros::Subscriber quadcop_realPos8_subj = n_j.subscribe("quadcop_realPos_8", 100, ROSrealposition8_received);
    ros::Subscriber quadcop_realPos9_subj = n_j.subscribe("quadcop_realPos_9", 100, ROSrealposition9_received);

    ros::Subscriber quadcop_noobst0 = n_j.subscribe("cPos_0", 100, ROSnoobst0_received);
    ros::Subscriber quadcop_noobst1 = n_j.subscribe("cPos_1", 100, ROSnoobst1_received);
    ros::Subscriber quadcop_noobst2 = n_j.subscribe("cPos_2", 100, ROSnoobst2_received);
    ros::Subscriber quadcop_noobst3 = n_j.subscribe("cPos_3", 100, ROSnoobst3_received);
    ros::Subscriber quadcop_noobst4 = n_j.subscribe("cPos_4", 100, ROSnoobst4_received);
    ros::Subscriber quadcop_noobst5 = n_j.subscribe("cPos_5", 100, ROSnoobst5_received);
    ros::Subscriber quadcop_noobst6 = n_j.subscribe("cPos_6", 100, ROSnoobst6_received);
    ros::Subscriber quadcop_noobst7 = n_j.subscribe("cPos_7", 100, ROSnoobst7_received);
    ros::Subscriber quadcop_noobst8 = n_j.subscribe("cPos_8", 100, ROSnoobst8_received);
    ros::Subscriber quadcop_noobst9 = n_j.subscribe("cPos_9", 100, ROSnoobst9_received);

    ros::Subscriber orientamento = n_j.subscribe("orient", 100, ROSorient_received);

    ros::Subscriber sub_com_x = n_j.subscribe("com_x_pub", 100, ROScomx_received);
    ros::Subscriber sub_com_y = n_j.subscribe("com_y_pub", 100, ROScomy_received);

    ros::Subscriber synchr = n_j.subscribe("synchro", 100, sync_received);
    ros::Subscriber ric1 = n_j.subscribe("info1", 100, info1_received);
    ros::Subscriber ric2 = n_j.subscribe("info2", 100, info2_received);
    ros::Subscriber ric3 = n_j.subscribe("info3", 100, info3_received);

	pubb = n_j.advertise<std_msgs::Float32>("time", 1);
pubb2 = n_j.advertise<std_msgs::Float32>("forma", 1);	


     ros::spin();
	
	
    return 0;
    
}
