#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "sensor_msgs/Joy.h"
#include "errno.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_S 0x73
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_ROTR 0x72
#define KEYCODE_ROTL 0x6C

#define PI 3,1415926535897932384


	double x_targ[10], y_targ[10], z_targ[10], w_targ[10], zang_targ[10], xang_targ[10], yang_targ[10];
	double x_targ_back[10], y_targ_back[10];
	double x_int_back[10], y_int_back[10];
	double x_pos[10], y_pos [10], z_pos[10], w_pos [10], zang_pos[10];
        double x_real_pos[10], z_real_pos[10], y_real_pos[10], w_real_pos[10], zang_real_pos[10], w_real_back[10], z_real_back[10];
        double x_int[10], y_int[10], z_int[10], w_int[10], zang_int[10];
	double alpha[10], radius[10], temp[10];
        int init[10], direction[10];
	int num, test2, perp=0;
        double xom=4.5, yom=0.125, zom=4.8;
        int formation =1;
	int left=0, right=0;
	double pend;
	int robot;
        ros::Publisher pos_pub_0, pos_pub_1, pos_pub_2, pos_pub_3, pos_pub_4, pos_pub_5, pos_pub_6, pos_pub_7, pos_pub_8, pos_pub_9;
        ros::Publisher c_pub_0, c_pub_1, c_pub_2, c_pub_3, c_pub_4, c_pub_5, c_pub_6, c_pub_7, c_pub_8, c_pub_9;
	ros::Publisher com_x_pub, com_y_pub;
	ros::Publisher or_pub;
	ros::Publisher synchro_pub, info1_pub, info2_pub, info3_pub;
        int ended=1;
	int rot=0;
        double alpha_or=0.0;
	int kfd = 0;
    	int obst_num = 25 ;
	double x_0[25], y_0[25], r[25];
    	double joy_x=0, joy_y=0, joy_z=0, joy_lat=0, joy_ind=0, joy_left=0;
    	int c=0;
	int dirty1[10];
	int dirty2[10];
	int button_1, button_2, button_3, button_4, button_5, button_6, button_x, button_y;
	double alpha_rot = 30.0;  // 90.0
	double spost = 0.5;
	int f_block=0, f_follow=1, angle=0;
	int variable;
	int saving =0;
	int id, id2, id3;

	struct termios cooked, raw;

int wait_for_drone(int i, double x_targ, double y_targ, double z_targ)
{
    double norm;
    norm=sqrt((x_real_pos[i]-x_targ)*(x_real_pos[i]-x_targ)+(y_real_pos[i]-y_targ)*(y_real_pos[i]-y_targ));
    //std::cout << "norm " << i << " = " << norm << std::endl;
   // if (angle==1)
//	{
   	if ((norm <1) & (fabs(fabs(w_real_pos[i])-fabs(w_targ[i]))<0.2) & (fabs(fabs(zang_real_pos[i])-fabs(zang_targ[i]))<0.2)) // (fabs(w_real_pos[i]-w_real_back[i])<0.0005) & (fabs(zang_real_pos[i]-z_real_back[i])<0.0005) )
    	{return 1;}
    	else
    	{return 0;}
//	}
/*	else {
		  if ((norm <1.8) & (fabs(fabs(w_real_pos[i])-fabs(w_targ[i]))<0.15) & (fabs(fabs(zang_real_pos[i])-fabs(zang_targ[i]))<0.15)) 
		{return 1;}
   		 else
   		 {return 0;}
	}*/
}

void int_obstacle_avoidance(int i, int j)
{
if (sqrt((x_int[i]-x_0[j])*(x_int[i]-x_0[j])+(y_int[i]-y_0[j])*(y_int[i]-y_0[j]))<(r[j]-0.05))
		{
			//std::cout << i << " x[i] " << x_int[i] << " y[i] " << y_int[i] << " x_targ[i] " << x_targ[i] << " y_targ[i] " << y_targ[i] << "direction" << direction[i]  << " " << fabs(x_targ[i]-x_int[i]) << std::endl;           
		if ((init[i])==0)
		{
		if (((x_targ[i]-x_int[i])*(x_targ[i]-x_int[i]))>=((y_targ[i]-y_int[i])*(y_targ[i]-y_int[i])))
			{	
			direction[i]=1;
			}
		else
			{
			direction[i]=2;
			}
		}
			if (direction[i]==1)
			{
                        
			x_int[i]=x_int[i];
			if ((y_int[i]-y_0[j])>0)
			{y_int[i]= sqrt((r[j])*(r[j])-(x_int[i]-x_0[j])*(x_int[i]-x_0[j]))+y_0[j];
			}
			else
			{y_int[i]= -sqrt((r[j])*(r[j])-(x_int[i]-x_0[j])*(x_int[i]-x_0[j]))+y_0[j];
			}
			if ((sqrt((x_targ[i]-x_int[i])*(x_targ[i]-x_int[i])))<0.1)
			{
			direction[i]=2;
			}
			}
            	else 
			{
			y_int[i]=y_int[i];
			if ((x_int[i]-x_0[j])>0)
			{x_int[i]= sqrt((r[j])*(r[j])-(y_int[i]-y_0[j])*(y_int[i]-y_0[j]))+x_0[j];
			}
			else
			{x_int[i]= -sqrt((r[j])*(r[j])-(y_int[i]-y_0[j])*(y_int[i]-y_0[j]))+x_0[j];
			}
			if ((sqrt((y_targ[i]-y_int[i])*(y_targ[i]-y_int[i])))<0.1)
			{
			direction[i]=1;
			}
			}
		}
}

void updating_positions()
{
double norm[num];
int test=num;
geometry_msgs::PoseStamped pos[10];
geometry_msgs::PoseStamped cam[10];
int done[num];

   for (int i=0;i<num;i++)
    {
        init[i]=0;
	direction[i]=1;
    } 

    for (int i=0;i<num;i++)
    {
        done[i]=0;
    }
    int arrived[num];
    for (int i=0;i<num;i++)
    {
        arrived[i]=1;
    }
    
    for (int i=0; i<10; i++)
    {
	if (angle!=1)
	{
         pos[i].pose.position.x = x_targ[i];
         pos[i].pose.position.y = y_targ[i];
         pos[i].pose.position.z = z_targ[i];
         pos[i].pose.orientation.x = 0;
         pos[i].pose.orientation.y = 0;
         pos[i].pose.orientation.z = zang_targ[i];
         pos[i].pose.orientation.w = w_targ[i];
	}
	else 
	{
	pos[i].pose.position.x = x_targ_back[i];
         pos[i].pose.position.y = y_targ_back[i];
         pos[i].pose.position.z = z_targ[i];
         pos[i].pose.orientation.x = 0;
         pos[i].pose.orientation.y = 0;
         pos[i].pose.orientation.z = zang_targ[i];
         pos[i].pose.orientation.w = w_targ[i];
	}
	 /*pos[i].pose.orientation.z = 0;
         pos[i].pose.orientation.w = 1;
	cam[i].pose.position.x = x_real_pos[i];
         cam[i].pose.position.y = y_real_pos[i];
         cam[i].pose.position.z = z_real_pos[i];
	cam[i].pose.orientation.x=xang_targ[i];
	cam[i].pose.orientation.y=yang_targ[i];
	cam[i].pose.orientation.z=zang_targ[i];
	cam[i].pose.orientation.w=w_targ[i];*/
    }
    
    
    while (test>0)
    {
        if (test2==1)
        {
            for (int i=0;i<num;i++)
            {
                arrived[i]=wait_for_drone(i, x_targ[i], y_targ[i], z_targ[i]);
		//std::cout << "arrived robot " << i << " : " << arrived[i] << std::endl;
                if (arrived[i]==1)
                {
                pos[i].pose.position.x = x_targ[i];
                pos[i].pose.position.y = y_targ[i];
                pos[i].pose.position.z = z_targ[i];
                pos[i].pose.orientation.x = 0;
                pos[i].pose.orientation.y = 0;
                pos[i].pose.orientation.z = zang_targ[i];
                pos[i].pose.orientation.w = w_targ[i];
		/*pos[i].pose.orientation.z = 0;
        	pos[i].pose.orientation.w = 1;
		cam[i].pose.position.x = x_real_pos[i];
         	cam[i].pose.position.y = y_real_pos[i];
         	cam[i].pose.position.z = z_real_pos[i];
		cam[i].pose.orientation.x=xang_targ[i];
		cam[i].pose.orientation.y=yang_targ[i];
		cam[i].pose.orientation.z=zang_targ[i];
		cam[i].pose.orientation.w=w_targ[i];*/
		test--;
                }
             /*   else if (arrived[i]==0)
                {
                pos[i].pose.position.x = pos[i].pose.position.x;
                pos[i].pose.position.y = pos[i].pose.position.y;
                pos[i].pose.position.z = pos[i].pose.position.z;
                //pos[i].pose.orientation.x = 0;
                //pos[i].pose.orientation.y = 0;
                pos[i].pose.orientation.z = pos[i].pose.orientation.z;
                pos[i].pose.orientation.w = pos[i].pose.orientation.w;
                }*/
            }
        }
        else if (angle==1)
 	{
		for (int i=0;i<num;i++)
            {
		arrived[i]=wait_for_drone(i, x_targ[i], y_targ[i], z_targ[i]);
		//std::cout << "arrived robot " << i << " : " << arrived[i] << std::endl;
		pos[i].pose.orientation.w = w_targ[i];
		pos[i].pose.orientation.z = zang_targ[i];
		if (arrived[i]==1)
			{
			test--;
			}	
	    }
	}
	/*  if (done[i]==0 & arrived[i]==1)
            	{
		std::cout << "entri qui?? " << std::endl;
		if ((fabs(fabs(w_pos[i])-fabs(w_targ[i]))>0.06))
			{
			if ((w_targ[i] -w_pos[i])>0.02)
				{w_int[i]= w_pos[i] + 0.04;}
			else if ((w_targ[i] -w_pos[i])<-0.02)
				{w_int[i]= w_pos[i] - 0.04;}
			else
				{w_int[i]=w_targ[i];}

			/*if ((zang_targ[i] -zang_pos[i])>0.02)
				{zang_int[i]= zang_pos[i] + 0.04;}
			else if ((zang_targ[i] -zang_pos[i])<-0.02)
				{zang_int[i]= zang_pos[i] - 0.04;}
			else
				{zang_int[i]=zang_targ[i];}


			//pos[i].pose.orientation.z = zang_int[i];
                    	pos[i].pose.orientation.w = w_int[i];

			}
		else
			{
			test--;
                        done[i]=1;
			}	
	    	}*/
        else
        {
        
        for (int i=0;i<num;i++)
        {
	x_int_back[i]=x_int[i];
		y_int_back[i]=y_int[i];
            if (done[i]==0 & arrived[i]==1)
            {
            norm[i]=sqrt((x_pos[i]-x_targ[i])*(x_pos[i]-x_targ[i])+(y_pos[i]-y_targ[i])*(y_pos[i]-y_targ[i])+(z_pos[i]-z_targ[i])*(z_pos[i]-z_targ[i]));

            if (norm[i]>0.1)
                {
                if ((x_targ[i] -x_pos[i])>0.05)
                    {x_int[i]= x_pos[i] + 0.1;}
                else if ((x_targ[i] -x_pos[i])<-0.05)
                    {x_int[i]= x_pos[i] - 0.1;}
                else
                    {x_int[i]=x_targ[i];}
            
                if ((y_targ[i] - y_pos[i])>0.05)
                    {y_int[i]= y_pos[i] + 0.1;}
                else if ((y_targ[i] - y_pos[i])<-0.05)
                    {y_int[i]= y_pos[i] - 0.1;}
                else
                    {y_int[i] = y_targ[i];}
            
                if ((z_targ[i] -z_pos[i])>0.05)
                    {z_int[i]= z_pos[i] + 0.1;}
                else if ((z_targ[i] -z_pos[i])<-0.05)
                    {z_int[i]= z_pos[i] - 0.1;}
                else
                    {z_int[i]=z_targ[i];}
            
		/*if (arcsin(w_targ[i])-arcsin(w_pos[i]))>3.141593/alpha_rot)
                    {w_int[i]= w_pos[i] + 0.3;}
                else if ((w_targ[i] -w_pos[i])<-0.15)
                    {w_int[i]= w_pos[i] - 0.3;}
                else
                    {w_int[i]=w_targ[i];}*/

		for (int j=0; j<obst_num; j++)
		{int_obstacle_avoidance(i,j);}

		init[i]=1;

                for (int j=0; j<10; j++)
                    {
                    if (j<i)
                        {
                    if (sqrt((x_int[i]-x_int[j])*(x_int[i]-x_int[j])+(y_int[i]-y_int[j])*(y_int[i]-y_int[j])+(z_int[i]-z_int[j])*(z_int[i]-z_int[j]))<0.75)
                            {
                            if (((x_int[i]-x_int[j])*(x_int[i]-x_int[j])+(y_int[i]-y_int[j])*(y_int[i]-y_int[j])+(z_int[i]-z_int[j])*(z_int[i]-z_int[j])) < ((x_pos[i]-x_int[j])*(x_pos[i]-x_int[j])+(y_pos[i]-y_int[j])*(y_pos[i]-y_int[j])+(z_pos[i]-z_int[j])*(z_pos[i]-z_int[j])))
                                {
                                if (done[j]==0)
                                    {
                                    x_int[i]=x_pos[i];
                                    y_int[i]=y_pos[i];
                                    if (z_pos[i] > 1.2)
                                        {
                                        if (z_pos[i]-z_pos[j]>0)
                                            {
                                            z_int[i]=z_pos[i]+0.8;
                                            }
                                        else
                                            {
                                            z_int[i]=z_pos[i]-0.8;
                                            }
                                        }
                                    }
				
                                    else
                                    {
                                    x_int[i]=x_pos[i];
                                    y_int[i]=y_pos[i];
                                    if (z_pos[i]-z_pos[j]>0)
                                        {
                                        z_int[i]=z_pos[i]+0.8;
                                        }
                                    else
                                        {
                                        z_int[i]=z_pos[i]-0.8;
                                        }
                                    }
                                }
                            }
                        }
                    if (j>i)
                        {
                            if (sqrt((x_int[i]-x_pos[j])*(x_int[i]-x_pos[j])+(y_int[i]-y_pos[j])*(y_int[i]-y_pos[j])+(z_int[i]-z_pos[j])*(z_int[i]-z_pos[j]))<0.5)
                            {
                            if (((x_int[i]-x_pos[j])*(x_int[i]-x_pos[j])+(y_int[i]-y_pos[j])*(y_int[i]-y_pos[j])+(z_int[i]-z_pos[j])*(z_int[i]-z_pos[j])) < ((x_pos[i]-x_pos[j])*(x_pos[i]-x_pos[j])+(y_pos[i]-y_pos[j])*(y_pos[i]-y_pos[j])+(z_pos[i]-z_pos[j])*(z_pos[i]-z_pos[j])))
                                {
                                if (done[j]==0)
                                    {
                                    x_int[i]=x_pos[i];
                                    y_int[i]=y_pos[i];
                                    z_int[i]=z_pos[i];
                                    }
                                else
                                    {
                                    x_int[i]=x_pos[i];
                                    y_int[i]=y_pos[i];
                                    if (z_pos[i]-z_pos[j]>0)
                                        {
                                        z_int[i]=z_pos[i]+0.8;
                                        }
                                    else
                                        {
                                        z_int[i]=z_pos[i]-0.8;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    pos[i].pose.position.x = x_int[i];
                    pos[i].pose.position.y = y_int[i];
                    pos[i].pose.position.z = z_int[i];
                    pos[i].pose.orientation.z = zang_targ[i];
                    pos[i].pose.orientation.w = w_targ[i];
		/*	pos[i].pose.orientation.z = 0;
        		pos[i].pose.orientation.w = 1;
			cam[i].pose.position.x = x_real_pos[i];
        		 cam[i].pose.position.y = y_real_pos[i];
        		 cam[i].pose.position.z = z_real_pos[i];
			cam[i].pose.orientation.x=xang_targ[i];
			cam[i].pose.orientation.y=yang_targ[i];
			cam[i].pose.orientation.z=zang_targ[i];
			cam[i].pose.orientation.w=w_targ[i];*/
                }
            else
                {
                    test--;
                    done[i]=1;
                    x_int[i]=  x_targ[i];
		    y_int[i] = y_targ[i];
		    z_int[i] = z_targ[i];                
                }
            }
        }
        }
        
        pos_pub_0.publish(pos[0]);
        pos_pub_1.publish(pos[1]);
        pos_pub_2.publish(pos[2]);
        pos_pub_3.publish(pos[3]);
        pos_pub_4.publish(pos[4]);
        pos_pub_5.publish(pos[5]);
        pos_pub_6.publish(pos[6]);
        pos_pub_7.publish(pos[7]);
        pos_pub_8.publish(pos[8]);
        pos_pub_9.publish(pos[9]);

	//cam_pub_0.publish(cam[0]);
	

        sleep(1);
        ros::spinOnce();
	if (test>0)
	{
	    std::cout << "Waiting....some robots are too far from their target positions...." << std::endl;
	}
else
	{
		 std::cout << "READY! " << std::endl; 
		 std::cout << "Use the analog joysticks to move " << std::endl;
		std::cout << "Formation angle: " << fmod(alpha_or*180/3.141593,360) << " degrees" << std::endl;
		std_msgs::Int32 sync;
		sync.data = saving;
		synchro_pub.publish(sync);
		std_msgs::Int32 inf1, inf2, inf3;
		inf1.data=id;
		inf2.data=id2;
		inf3.data=id3;
		info1_pub.publish(inf1);
		info2_pub.publish(inf2);
		info3_pub.publish(inf3);
	}
        for (int i=0;i<num;i++)
        {
            arrived[i]=wait_for_drone(i, x_int[i], y_int[i], z_int[i]);
        }
    }
}

void obstacle_check(int j, int i)
{
if (sqrt((x_targ[j]-x_0[i])*(x_targ[j]-x_0[i])+(y_targ[j]-y_0[i])*(y_targ[j]-y_0[i]))<(r[i]-0.05))
    {
     dirty1[j]=1;
     double alpha_or_p = alpha_or;

     if (rot==1)
	{
	if (formation==1)
	{perp=0;
	if(j==0)
	{perp=1;}
	if ((j==2)|(j==7))
	{alpha_or_p=alpha_or-3.141593/4;}
	if ((j==1)|(j==8))
        {alpha_or_p=alpha_or+3.141593/4;}
	if (j==6)
	{alpha_or_p=alpha_or-0.32;}
	if (j==9)
	{alpha_or_p=alpha_or+0.32;}
	}
	if (formation==2)
	{perp=1;}
	if (formation==3)
	{perp=0;}
	if (formation==4)
	{
	perp=0;
	if ((j==0)|(j==9))
	{alpha_or_p=alpha_or+1.249;}
	if ((j==1)|(j==8))
	{alpha_or_p=alpha_or-1.249;}
	if ((j==2)|(j==7))
	{alpha_or_p=alpha_or+0.54;}
	if ((j==3)|(j==6))
	{alpha_or_p=alpha_or-0.54;}
	}
	}

         int alpha_test = alpha_or*180/3.141593;

        while (alpha_test <0)
	{alpha_test=alpha_test+360;}

	//std::cout << "alpha_test: " << alpha_test << std::endl;
	//std::cout << "perp: " << perp << std::endl;

	if (perp==0)
	{
        if ((((abs(alpha_test%180))<=1) | ((abs(alpha_test%180))>=179))&(robot==0))
        {
			x_targ[j]=x_targ[j];
			if ((y_targ_back[j]-y_0[i])>0)
			{y_targ[j]= sqrt((r[i])*(r[i])-(x_targ[j]-x_0[i])*(x_targ[j]-x_0[i]))+y_0[i];
			}
			else
			{y_targ[j]= -sqrt((r[i])*(r[i])-(x_targ[j]-x_0[i])*(x_targ[j]-x_0[i]))+y_0[i];
			}
        }
        
		else if ((((abs(alpha_test%180))>=89) & ((abs(alpha_test%180))<=91))&(robot==0))
        {
			y_targ[j]=y_targ[j];
			if ((x_targ_back[j]-x_0[i])>0)
			{x_targ[j]= sqrt((r[i])*(r[i])-(y_targ[j]-y_0[i])*(y_targ[j]-y_0[i]))+x_0[i];
			}
			else
			{x_targ[j]= -sqrt((r[i])*(r[i])-(y_targ[j]-y_0[i])*(y_targ[j]-y_0[i]))+x_0[i];
			}
        }
        
        else
        {
            double a;
	    if (robot==1)
	    {a = pend;}
	    else
            {a = tan((3.141593/2)-alpha_or_p);}
            double b = y_targ[j]-a*x_targ[j];
            double a1 = a*a+1;
            double a2 = a*(b-y_0[i])-x_0[i];
            double a3 = ((b-y_0[i])*(b-y_0[i]))+(x_0[i]*x_0[i])-(r[i]*r[i]);
            double x_tent_1=(-a2-sqrt(a2*a2-a1*a3))/a1;
            double x_tent_2=(-a2+sqrt(a2*a2-a1*a3))/a1;
            double y_tent_1= a*x_tent_1+b;
            double y_tent_2= a*x_tent_2+b;
            if (sqrt((x_targ[j]-x_tent_1)*(x_targ[j]-x_tent_1)+(y_targ[j]-y_tent_1)*(y_targ[j]-y_tent_1))<sqrt((x_targ[j]-x_tent_2)*(x_targ[j]-x_tent_2)+(y_targ[j]-y_tent_2)*(y_targ[j]-y_tent_2)))
            {
                x_targ[j]=x_tent_1;
                y_targ[j]=y_tent_1;
            }
            else
            {
                x_targ[j]=x_tent_2;
                y_targ[j]=y_tent_2;
            } 
        }
	}
        
	else if (perp==1)
	{
        if (((abs(alpha_test%180))>=89) & ((abs(alpha_test%180))<=91))
        {
			x_targ[j]=x_targ[j];
			if ((y_targ_back[j]-y_0[i])>0)
			{y_targ[j]= sqrt((r[i])*(r[i])-(x_targ[j]-x_0[i])*(x_targ[j]-x_0[i]))+y_0[i];
			}
			else
			{y_targ[j]= -sqrt((r[i])*(r[i])-(x_targ[j]-x_0[i])*(x_targ[j]-x_0[i]))+y_0[i];
			}
        }
        
		else if (((abs(alpha_test%180))<=1) | ((abs(alpha_test%180))>=179))
        {
			y_targ[j]=y_targ[j];
			if ((x_targ_back[j]-x_0[i])>0)
			{x_targ[j]= sqrt((r[i])*(r[i])-(y_targ[j]-y_0[i])*(y_targ[j]-y_0[i]))+x_0[i];
			}
			else
			{x_targ[j]= -sqrt((r[i])*(r[i])-(y_targ[j]-y_0[i])*(y_targ[j]-y_0[i]))+x_0[i];
			}
        }
        
        else
        {
	    double a;
	    if (robot==1)
	    {a = pend;}
	    else
            {a = tan(-alpha_or);}
            double b = y_targ[j]-a*x_targ[j];
            double a1 = a*a+1;
            double a2 = a*(b-y_0[i])-x_0[i];
            double a3 = ((b-y_0[i])*(b-y_0[i]))+(x_0[i]*x_0[i])-(r[i]*r[i]);
            double x_tent_1=(-a2-sqrt(a2*a2-a1*a3))/a1;
            double x_tent_2=(-a2+sqrt(a2*a2-a1*a3))/a1;
            double y_tent_1= a*x_tent_1+b;
            double y_tent_2= a*x_tent_2+b;
            if (sqrt((x_targ[j]-x_tent_1)*(x_targ[j]-x_tent_1)+(y_targ[j]-y_tent_1)*(y_targ[j]-y_tent_1))<sqrt((x_targ[j]-x_tent_2)*(x_targ[j]-x_tent_2)+(y_targ[j]-y_tent_2)*(y_targ[j]-y_tent_2)))
            {
                x_targ[j]=x_tent_1;
                y_targ[j]=y_tent_1;
            }
            else
            {
                x_targ[j]=x_tent_2;
                y_targ[j]=y_tent_2;
            }
        }
	}
    }
}


void robot_check(int i, int j)
{
   int alpha_test = alpha_or*180/3.141593;
        if (sqrt((x_targ[i]-x_targ[j])*(x_targ[i]-x_targ[j])+(y_targ[i]-y_targ[j])*(y_targ[i]-y_targ[j]))<0.95)
        {    dirty2[i]=1;
            double a=(y_targ[j]-y_targ[i])/(x_targ[j]-x_targ[i]);
            	double b= (y_targ[i]*(x_targ[j]-x_targ[i])-x_targ[i]*(y_targ[j]-y_targ[i]))/(x_targ[j]-x_targ[i]);
            	if(x_targ_back[i]>x_targ[j])
            	{
		 x_targ[i]=x_targ[i]+double(rand()%500)/10000;
	        }
            	else
            	{x_targ[i]=x_targ[i]-double(rand()%500)/10000;
                 }
	
                 if(y_targ_back[i]>y_targ[j])
            	{
		 y_targ[i]=y_targ[i]+double(rand()%500)/10000;
	          }
            	else
            	{y_targ[i]=y_targ[i]-double(rand()%500)/10000;
                 }
	}
}

void fromCoMtopositions()
{
	int still_rotating[num];
	for (int i =0; i<num; i++)
        {
            still_rotating[i]=0;
        }
	variable=0;

    //std::cout << "boh" << std::endl;
    double form1_x[10], form1_y[10], form1_z[10], form2_x[10], form2_y[10], form2_z[10];
    double form3_x[10], form3_y[10], form3_z[10], form4_x[10], form4_y[10], form4_z[10];
		
    form1_x[0]=1.5;
    form1_y[0]=0;
    form1_z[0]=0;
    form1_x[1]=0.75;
    form1_y[1]=0.75;
    form1_z[1]=0;
    form1_x[2]=0.75;
    form1_y[2]=-0.75;
    form1_z[2]=0;
    form1_x[3]=0;
    form1_y[3]=1.5;
    form1_z[3]=0;
    form1_x[4]=0;
    form1_y[4]=0;
    form1_z[4]=0;
    form1_x[5]=0;
    form1_y[5]=-1.5;
    form1_z[5]=0;
    form1_x[6]=-0.75;
    form1_y[6]=2.25;
    form1_z[6]=0;
    form1_x[7]=-0.75;
    form1_y[7]=0.75;
    form1_z[7]=0;
    form1_x[8]=-0.75;
    form1_y[8]=-0.75;
    form1_z[8]=0;
    form1_x[9]=-0.75;
    form1_y[9]=-2.25;
    form1_z[9]=0;
    
    form2_x[0]=4.5;
    form2_y[0]=0;
    form2_z[0]=0;
    form2_x[1]=3.5;
    form2_y[1]=0;
    form2_z[1]=0;
    form2_x[2]=2.5;
    form2_y[2]=0;
    form2_z[2]=0;
    form2_x[3]=1.5;
    form2_y[3]=0;
    form2_z[3]=0;
    form2_x[4]=0.5;
    form2_y[4]=0;
    form2_z[4]=0;
    form2_x[5]=-0.5;
    form2_y[5]=0;
    form2_z[5]=0;
    form2_x[6]=-1.5;
    form2_y[6]=0;
    form2_z[6]=0;
    form2_x[7]=-2.5;
    form2_y[7]=0;
    form2_z[7]=0;
    form2_x[8]=-3.5;
    form2_y[8]=0;
    form2_z[8]=0;
    form2_x[9]=-4.5;
    form2_y[9]=0;
    form2_z[9]=0;

    form3_x[0]=0;
    form3_y[0]=0.5;
    form3_z[0]=0;
    form3_x[1]=0;
    form3_y[1]=-0.5;
    form3_z[1]=0;
    form3_x[2]=0;
    form3_y[2]=1.5;
    form3_z[2]=0;
    form3_x[3]=0;
    form3_y[3]=-1.5;
    form3_z[3]=0;
    form3_x[4]=0;
    form3_y[4]=2.5;
    form3_z[4]=0;
    form3_x[5]=0;
    form3_y[5]=-2.5;
    form3_z[5]=0;
    form3_x[6]=0;
    form3_y[6]=3.5;
    form3_z[6]=0;
    form3_x[7]=0;
    form3_y[7]=-3.5;
    form3_z[7]=0;
    form3_x[8]=0;
    form3_y[8]=4.5;
    form3_z[8]=0;
    form3_x[9]=0;
    form3_y[9]=-4.5;
    form3_z[9]=0;

    form3_x[0]=0;
    form3_y[0]=0.5;
    form3_z[0]=0;
    form3_x[1]=0;
    form3_y[1]=-0.5;
    form3_z[1]=0;
    form3_x[2]=0;
    form3_y[2]=1.5;
    form3_z[2]=0;
    form3_x[3]=0;
    form3_y[3]=-1.5;
    form3_z[3]=0;
    form3_x[4]=0;
    form3_y[4]=2.5;
    form3_z[4]=0;
    form3_x[5]=0;
    form3_y[5]=-2.5;
    form3_z[5]=0;
    form3_x[6]=0;
    form3_y[6]=3.5;
    form3_z[6]=0;
    form3_x[7]=0;
    form3_y[7]=-3.5;
    form3_z[7]=0;
    form3_x[8]=0;
    form3_y[8]=4.5;
    form3_z[8]=0;
    form3_x[9]=0;
    form3_y[9]=-4.5;
    form3_z[9]=0;

    form4_x[0]=1.5;
    form4_y[0]=0.5;
    form4_z[0]=0;
    form4_x[1]=1.5;
    form4_y[1]=-0.5;
    form4_z[1]=0;
    form4_x[2]=0.75;
    form4_y[2]=1.25;
    form4_z[2]=0;
    form4_x[3]=0.75;
    form4_y[3]=-1.25;
    form4_z[3]=0;
    form4_x[4]=0.0;
    form4_y[4]=2.0;
    form4_z[4]=0;
    form4_x[5]=-0.0;
    form4_y[5]=-2.0;
    form4_z[5]=0;
    form4_x[6]=-0.75;
    form4_y[6]=1.25;
    form4_z[6]=0;
    form4_x[7]=-0.75;
    form4_y[7]=-1.25;
    form4_z[7]=0;
    form4_x[8]=-1.5;
    form4_y[8]=0.5;
    form4_z[8]=0;
    form4_x[9]=-1.5;
    form4_y[9]=-0.5;
    form4_z[9]=0;

    
    for (int i=0;i<num;i++)
    {
     	x_targ_back[i]=x_targ[i];
     	y_targ_back[i]=y_targ[i];
    }
    
    
    for (int i=0;i<num;i++)
    {
        if (formation==1)
        {
        x_targ[i]=xom+form1_x[i];
        y_targ[i]=yom+form1_y[i];
        z_targ[i]=zom+form1_z[i];
        }
        if (formation==2)
        {
        x_targ[i]=xom+form2_x[i];
        y_targ[i]=yom+form2_y[i];
        z_targ[i]=zom+form2_z[i];
        }
        if (formation==3)
        {
        x_targ[i]=xom+form3_x[i];
        y_targ[i]=yom+form3_y[i];
        z_targ[i]=zom+form3_z[i];
        }
        if (formation==4)
        {
        x_targ[i]=xom+form4_x[i];
        y_targ[i]=yom+form4_y[i];
        z_targ[i]=zom+form4_z[i];
        }
        alpha[i] = atan2((y_targ[i]-yom),(x_targ[i]-xom));
        radius[i] = sqrt((x_targ[i]-xom)*(x_targ[i]-xom)+(y_targ[i]-yom)*(y_targ[i]-yom));
        temp[i] = alpha[i] - alpha_or;
        x_targ[i] = xom + radius[i]*cos(temp[i]);
        y_targ[i] = yom + radius[i]*sin(temp[i]);
	if (f_follow==1)
	{
	/*xang_targ[i] = 0.5*cos(alpha_or/2)+0.5*sin(alpha_or/2);
	yang_targ[i] = 0.5*cos(alpha_or/2)-0.5*sin(alpha_or/2);
	w_targ[i] = 0.5*cos(alpha_or/2)+0.5*sin(alpha_or/2);
	zang_targ[i] = 0.5*cos(alpha_or/2)-0.5*sin(alpha_or/2);*/

		
	double alpha_confr=alpha_or;
	double alpha_act;
	//if (zang_real_pos[i]<0)
	//{alpha_act=2*asin(zang_real_pos[i]);}
	//else
	{alpha_act=-2*asin(zang_pos[i]);}
	while ((alpha_confr-alpha_act)>3.141592)
	{alpha_confr=alpha_confr-2*3.141592;}
	while ((alpha_confr-alpha_act)<-3.141592)
	{alpha_confr=alpha_confr+2*3.141592;}

	while ((alpha_confr-alpha_act)>3.141592/5)
		{
		alpha_confr=(alpha_confr+alpha_act)/2;
		still_rotating[i]=1;
		variable=1;
		
		}

	while ((alpha_confr-alpha_act)<-3.141592/5)
		{
		alpha_confr=(alpha_confr+alpha_act)/2;
		still_rotating[i]=1;
		variable=1;
		}

	if (i==0)
	{
	//std::cout << "alpha_confr: " << alpha_confr << " alpha_act: " << alpha_act << std::endl;
	}
	
	if (still_rotating[i]==1)
	{
	zang_targ[i] = sin(-alpha_confr/2);
        w_targ[i] = cos(-alpha_confr/2);
	}
	else
	{
	zang_targ[i] = sin(-alpha_or/2);
        w_targ[i] = cos(-alpha_or/2);
	}
	if (i==0)
	{
	//std::cout << "Rotazione concorde con l'orientamento della formazione" << std::endl;
	}
	}
	else if (f_block==1)
	{
	xang_targ[i] = xang_targ[i];
	yang_targ[i] = yang_targ[i];
	w_targ[i] = w_targ[i];
	zang_targ[i] = zang_targ[i];
	if (i==0)
	{
	//std::cout << "Rotazione bloccata" << std::endl;
	}
	}
	
        dirty1[i]=1;
        dirty2[i]=1;
        robot=0;

	int alpha_test = alpha_or*180/3.141593;

        while (alpha_test <0)
	{alpha_test=alpha_test+360;}

	//if (i==9)
	//{//std::cout << "angolo: " << alpha_test << std::endl;}
	
	if (rot==1)
	{	if (left==1)
		{
			if (((abs(alpha_test%360))>=0) & ((abs(alpha_test%360))<90))
			{
			double deltayom=radius[0]*(sin(temp[0]-(3.141593/alpha_rot))-sin(temp[0]));
			y_targ[i]=y_targ[i]+deltayom;
			}

			if (((abs(alpha_test%360))>=90) & ((abs(alpha_test%360))<180))
			{
			double deltaxom=radius[0]*(cos(temp[0]-(3.141593/alpha_rot))-cos(temp[0]));
			x_targ[i]=x_targ[i]+deltaxom;
			}

			if (((abs(alpha_test%360))>=180) & ((abs(alpha_test%360))<270))
			{
			double deltayom=radius[0]*(sin(temp[0]-(3.141593/alpha_rot))-sin(temp[0]));
			y_targ[i]=y_targ[i]+deltayom;
			}
			
			if (((abs(alpha_test%360))>=270) & ((abs(alpha_test%360))<360))
			{
			double deltaxom=radius[0]*(cos(temp[0]-(3.141593/alpha_rot))-cos(temp[0]));
			x_targ[i]=x_targ[i]+deltaxom;
			}


		}

		else if (right==1)
		{
			if (((abs(alpha_test%360))>0) & ((abs(alpha_test%360))<=90))
			{
			double deltaxom=radius[0]*(cos(temp[0]+(3.141593/alpha_rot))-cos(temp[0]));
			x_targ[i]=x_targ[i]+deltaxom;
			}

			if (((abs(alpha_test%360))>90) & ((abs(alpha_test%360))<=180))
			{
			double deltayom=radius[0]*(sin(temp[0]+(3.141593/alpha_rot))-sin(temp[0]));
			y_targ[i]=y_targ[i]+deltayom;
			}

			if (((abs(alpha_test%360))>180) & ((abs(alpha_test%360))<=270))
			{
			double deltaxom=radius[0]*(cos(temp[0]+(3.141593/alpha_rot))-cos(temp[0]));
			x_targ[i]=x_targ[i]+deltaxom;
			}
			
			if (((abs(alpha_test%360))>270) & ((abs(alpha_test%360))<=360))
			{
			double deltayom=radius[0]*(sin(temp[0]+(3.141593/alpha_rot))-sin(temp[0]));
			y_targ[i]=y_targ[i]+deltayom;
			}


		}
	}


	if (i==9)
	{

	xom=0;
	yom=0;	

	for (int k=0;k<10;k++)
	{xom=xom+x_targ[k];
	 yom=yom+y_targ[k];
	}
	xom=xom/10;
	yom=yom/10;
	}

/// qui inviare gli x_targ e y_targ e alpha_or
 	
	geometry_msgs::PoseStamped ca[10];
	std_msgs::Float32 orien;
	std_msgs::Float32 com_x;
	std_msgs::Float32 com_y;

	for (int i=0;i<num;i++)
	{
	ca[i].pose.position.x = x_targ[i];
        ca[i].pose.position.y = y_targ[i];
	ca[i].pose.position.z = z_targ[i];
	}

	orien.data=fmod(alpha_or*180/3.141593,180);
	while(orien.data<0)
	{orien.data=orien.data+180.0;}

	com_x.data=xom;
	com_y.data=yom;
	

	c_pub_0.publish(ca[0]);
        c_pub_1.publish(ca[1]);
        c_pub_2.publish(ca[2]);
        c_pub_3.publish(ca[3]);
        c_pub_4.publish(ca[4]);
        c_pub_5.publish(ca[5]);
        c_pub_6.publish(ca[6]);
        c_pub_7.publish(ca[7]);
        c_pub_8.publish(ca[8]);
        c_pub_9.publish(ca[9]);

	or_pub.publish(orien);

	com_x_pub.publish(com_x);
	com_y_pub.publish(com_y);
	
////


   }
  for (int i=0;i<num;i++)
    {
	int conta=0;
        while (((dirty1[i]==1)|(dirty2[i]==1))&conta<100)
        {	
	     if (test2==1)
		{
	    
	      while (((x_targ[i]-x_targ_back[i])*(x_targ[i]-x_targ_back[i])+(y_targ[i]-y_targ_back[i])*(y_targ[i]-y_targ_back[i]))>2)
	    {x_targ[i]=(x_targ[i]+x_targ_back[i])/2;
	     y_targ[i]=(y_targ[i]+y_targ_back[i])/2;
	    robot=0;
		}
	    }
	    conta++;
	    dirty1[i]=0;
	    for (int j=0; j<obst_num; j++)
            {obstacle_check(i,j);}
	    dirty2[i]=0;
	    robot=0;
	    double x_tempor = x_targ[i];
	    double y_tempor = y_targ[i];
            for (int j=0;j<i;j++)
            {robot_check(i,j);}
	    if ((x_targ[i]!=x_tempor) & (y_targ[i]!=y_tempor))
	    {pend=(x_tempor-x_targ[i])/(y_targ[i]-y_tempor);
	     robot=1;}
	   if (conta == 99)
	    {z_targ[i]=z_targ[i]-1;}
	   
        }
	if (angle==1)
	{
	x_targ[i]=x_targ_back[i];
	y_targ[i]=y_targ_back[i];
	}
    }
std::cout << "The robot leader is in position: " << x_targ[0] << " " << y_targ[0] << std::endl;
//std::cout << "pos 1: " << x_targ[1] << " " << y_targ[1] << std::endl;
//std::cout << "pos 2: " << x_targ[2] << " " << y_targ[2] << std::endl;
//std::cout << "pos 3: " << x_targ[3] << " " << y_targ[3] << std::endl;
//std::cout << "pos 4: " << x_targ[4] << " " << y_targ[4] << std::endl;
//std::cout << "pos 5: " << x_targ[5] << " " << y_targ[5] << std::endl;
//std::cout << "pos 6: " << x_targ[6] << " " << y_targ[6] << std::endl;
//std::cout << "pos 7: " << x_targ[7] << " " << y_targ[7] << std::endl;
//std::cout << "pos 8: " << x_targ[8] << " " << y_targ[8] << std::endl; 
//std::cout << "pos 9: " << x_targ[9] << " " << y_targ[9] << std::endl;
//std::cout << "xom e yom: " << xom << " " << yom << std::endl;
    
    updating_positions();
    ended=1;
}


void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void ROSposition0_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[0]=msg->pose.position.x;
y_pos[0]=msg->pose.position.y;
z_pos[0]=msg->pose.position.z;
w_pos[0]=msg->pose.orientation.w;
zang_pos[0]=msg->pose.orientation.z;
}

void ROSposition1_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[1]=msg->pose.position.x;
y_pos[1]=msg->pose.position.y;
z_pos[1]=msg->pose.position.z;
w_pos[1]=msg->pose.orientation.w;
zang_pos[1]=msg->pose.orientation.z;
}

void ROSposition2_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[2]=msg->pose.position.x;
y_pos[2]=msg->pose.position.y;
z_pos[2]=msg->pose.position.z;
w_pos[2]=msg->pose.orientation.w;
zang_pos[2]=msg->pose.orientation.z;
}

void ROSposition3_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[3]=msg->pose.position.x;
y_pos[3]=msg->pose.position.y;
z_pos[3]=msg->pose.position.z;
w_pos[3]=msg->pose.orientation.w;
zang_pos[3]=msg->pose.orientation.z;
}

void ROSposition4_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[4]=msg->pose.position.x;
y_pos[4]=msg->pose.position.y;
z_pos[4]=msg->pose.position.z;
w_pos[4]=msg->pose.orientation.w;
zang_pos[4]=msg->pose.orientation.z;
}

void ROSposition5_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[5]=msg->pose.position.x;
y_pos[5]=msg->pose.position.y;
z_pos[5]=msg->pose.position.z;
w_pos[5]=msg->pose.orientation.w;
zang_pos[5]=msg->pose.orientation.z;
}

void ROSposition6_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[6]=msg->pose.position.x;
y_pos[6]=msg->pose.position.y;
z_pos[6]=msg->pose.position.z;
w_pos[6]=msg->pose.orientation.w;
zang_pos[6]=msg->pose.orientation.z;
}

void ROSposition7_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[7]=msg->pose.position.x;
y_pos[7]=msg->pose.position.y;
z_pos[7]=msg->pose.position.z;
w_pos[7]=msg->pose.orientation.w;
zang_pos[7]=msg->pose.orientation.z;
}

void ROSposition8_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[8]=msg->pose.position.x;
y_pos[8]=msg->pose.position.y;
z_pos[8]=msg->pose.position.z;
w_pos[8]=msg->pose.orientation.w;
zang_pos[8]=msg->pose.orientation.z;
}

void ROSposition9_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
x_pos[9]=msg->pose.position.x;
y_pos[9]=msg->pose.position.y;
z_pos[9]=msg->pose.position.z;
w_pos[9]=msg->pose.orientation.w;
zang_pos[9]=msg->pose.orientation.z;
}

void ROSrealposition0_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[0]=msg->pose.position.x;
    y_real_pos[0]=msg->pose.position.y;
    z_real_pos[0]=msg->pose.position.z;
    w_real_back[0]=w_real_pos[0];
    w_real_pos[0]=msg->pose.orientation.w;
    z_real_back[0]=zang_real_pos[0];
    zang_real_pos[0]=msg->pose.orientation.z;
}

void ROSrealposition1_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[1]=msg->pose.position.x;
    y_real_pos[1]=msg->pose.position.y;
    z_real_pos[1]=msg->pose.position.z;
	w_real_back[1]=w_real_pos[1];
    w_real_pos[1]=msg->pose.orientation.w;
	z_real_back[1]=zang_real_pos[1];
    zang_real_pos[1]=msg->pose.orientation.z;
}

void ROSrealposition2_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[2]=msg->pose.position.x;
    y_real_pos[2]=msg->pose.position.y;
    z_real_pos[2]=msg->pose.position.z;
	w_real_back[2]=w_real_pos[2];
    w_real_pos[2]=msg->pose.orientation.w;
	z_real_back[2]=zang_real_pos[2];
    zang_real_pos[2]=msg->pose.orientation.z;
}

void ROSrealposition3_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[3]=msg->pose.position.x;
    y_real_pos[3]=msg->pose.position.y;
    z_real_pos[3]=msg->pose.position.z;
	w_real_back[3]=w_real_pos[3];
    w_real_pos[3]=msg->pose.orientation.w;
	z_real_back[3]=zang_real_pos[3];
    zang_real_pos[3]=msg->pose.orientation.z;;
}

void ROSrealposition4_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[4]=msg->pose.position.x;
    y_real_pos[4]=msg->pose.position.y;
    z_real_pos[4]=msg->pose.position.z;
	w_real_back[4]=w_real_pos[4];
    w_real_pos[4]=msg->pose.orientation.w;
	z_real_back[4]=zang_real_pos[4];
    zang_real_pos[4]=msg->pose.orientation.z;
}

void ROSrealposition5_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[5]=msg->pose.position.x;
    y_real_pos[5]=msg->pose.position.y;
    z_real_pos[5]=msg->pose.position.z;
	w_real_back[5]=w_real_pos[5];
    w_real_pos[5]=msg->pose.orientation.w;
	z_real_back[5]=zang_real_pos[5];
    zang_real_pos[5]=msg->pose.orientation.z;
}

void ROSrealposition6_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[6]=msg->pose.position.x;
    y_real_pos[6]=msg->pose.position.y;
    z_real_pos[6]=msg->pose.position.z;
	w_real_back[6]=w_real_pos[6];
    w_real_pos[6]=msg->pose.orientation.w;
	z_real_back[6]=zang_real_pos[6];
    zang_real_pos[6]=msg->pose.orientation.z;
}

void ROSrealposition7_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[7]=msg->pose.position.x;
    y_real_pos[7]=msg->pose.position.y;
    z_real_pos[7]=msg->pose.position.z;
	w_real_back[7]=w_real_pos[7];
    w_real_pos[7]=msg->pose.orientation.w;
	z_real_back[7]=zang_real_pos[7];
    zang_real_pos[7]=msg->pose.orientation.z;
}

void ROSrealposition8_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[8]=msg->pose.position.x;
    y_real_pos[8]=msg->pose.position.y;
    z_real_pos[8]=msg->pose.position.z;
	w_real_back[8]=w_real_pos[8];
    w_real_pos[8]=msg->pose.orientation.w;
	z_real_back[8]=zang_real_pos[8];
    zang_real_pos[8]=msg->pose.orientation.z;
}

void ROSrealposition9_received(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_real_pos[9]=msg->pose.position.x;
    y_real_pos[9]=msg->pose.position.y;
    z_real_pos[9]=msg->pose.position.z;
	w_real_back[9]=w_real_pos[9];
    w_real_pos[9]=msg->pose.orientation.w;
	z_real_back[9]=zang_real_pos[9];
    zang_real_pos[9]=msg->pose.orientation.z;
}

void joyCallback( const sensor_msgs::JoyConstPtr& msg)
{
    
	if( msg->axes.size() < 3)
	{
		ROS_ERROR( "Too few joystick axes:(expected more than 3)");
		return;
	}
    
    joy_x = msg->axes[1];
    joy_y = msg->axes[0];
    joy_left = msg->axes[4];
    joy_lat = msg->axes[5];
    joy_ind = msg->axes[6];

   /* if (joy_z>0.33)
	{
	alpha_rot=30.0;
	spost = 0.4;
	}
	else if (fabs(joy_z)<=0.33)
	{
	alpha_rot=60.0;
	spost=0.2;
	}
	else if (joy_z<-0.33)
	{
	alpha_rot=90.0;
	spost=0.1;
	}*/
	;
   button_1=msg->buttons[0];
   button_2=msg->buttons[1];
   button_3=msg->buttons[2];
   button_4=msg->buttons[3];
   button_5=msg->buttons[5]; 
   button_6=msg->buttons[7];
   // to check
    button_x = msg->buttons[8];
    button_y =msg->buttons[9];


}


void keyLoop()
{
    for(;;)
    {
        ros::spinOnce();

//	if (((x_real_pos[0]>-6.00)&(x_real_pos[0]<7.325))&(y_real_pos[0]>34.00))
//	{
//	std::cout << "TARGET REACHED" << std::endl;
//	std_msgs::Float32 comi_x;
//	comi_x.data=200;
//	com_x_pub.publish(comi_x);
//	return;
//	}
    // cambio formazione
   
if (id2==1)
	{
	 if ((button_1!=0))
	{
	c=4;
	}
    	else if ((button_2!=0))
	{
	c=5;
	}
   	else if ((button_3!=0))
	{c=6;
	}
   	else if ((button_4!=0))
	{c=7;
	}
	else if (joy_lat <-0.7)
	{
	c=9;
	}
        else if (joy_lat > 0.7)
	{
	c=8;
	}
        else if (joy_ind ==-1.0)
	{
	c=11;
	}
   	else if (joy_ind ==1.0)
	{
	c=12;
	}
	
	else if (button_5!=0)
	{
	f_block=1;
	f_follow=0;
	//std::cout << "Rotazione bloccata" << std::endl;
	usleep(50000);
	alpha_rot = 30.0;
	c=0;
	}
   else if (button_6!=0)
	{
	f_block=0;
	f_follow=1;
	angle=1;
	std::cout << "Robots are going to align.." << std::endl;
	alpha_rot = 30.0; //90.0
	c=13;
	}
        
    else if (button_x!=0)
    {
        alpha_rot = 30.0;
        test2=1;
        f_block=1;
        f_follow=0;
        std::cout << "Rotate all quadrotors together, or press O to select the robot, start to go back to the standard mode" << std::endl;
        int nume=-1;
        
        double alpha_next[num];
        for (int i =0; i<num; i++)
        {
            alpha_next[i]=alpha_or;
        }
        
        while (button_y==0)
        {
            ros::spinOnce();
            if (button_2!=0)
            {nume++;
 		
                if (nume<num)
                {
                std::cout << "Selected for the rotation: robot n.  " << nume << std::endl;
		usleep(500000);
                }
                else

                {nume=-1;
                  std::cout << "All quadrotors together " << std::endl;  
		usleep(500000);
		}
                    
            }
            
            if ((nume ==-1)&(joy_left<-0.7))
            {
                std::cout << "Rotating all quadrotors together " << std::endl;
                for (int i =0; i<num; i++)
                {
                    alpha_next[i]=alpha_next[i]+ (3.141593/alpha_rot);
		    std::cout << "Robot angle " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << " degrees" << std::endl;
                }
                for (int i =0; i<num; i++)
                {
                zang_targ[i] = sin(-alpha_next[i]/2);
                w_targ[i] = cos(-alpha_next[i]/2);
                }
                updating_positions();
            }
            else if ((nume ==-1)&(joy_left>0.7))
            {
                std::cout << "Rotating all quadrotors together " << std::endl;
               for (int i =0; i<num; i++)
                {
                    alpha_next[i]=alpha_next[i]- (3.141593/alpha_rot);	
		   std::cout << "Robot angle " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << " degrees" << std::endl;
                }
               for (int i =0; i<num; i++)
                {
                zang_targ[i] = sin(-alpha_next[i]/2);
                w_targ[i] = cos(-alpha_next[i]/2);
                }
                updating_positions();
            }
            else if (joy_left<-0.7)
            {
                alpha_next[nume]=alpha_next[nume]+ (3.141593/alpha_rot);
		std::cout << "Robot angle " << nume << " = " << fmod(alpha_next[nume]*180/3.141593,360) << " degrees" << std::endl;
                zang_targ[nume] = sin(-alpha_next[nume]/2);
                w_targ[nume] = cos(-alpha_next[nume]/2);
                updating_positions();
            }
            else if (joy_left>0.7)
            {
                alpha_next[nume]=alpha_next[nume]- (3.141593/alpha_rot);
		std::cout << "Robot angle " << nume << " = " << fmod(alpha_next[nume]*180/3.141593,360) << " degrees" << std::endl;
                zang_targ[nume] = sin(-alpha_next[nume]/2);
                w_targ[nume] = cos(-alpha_next[nume]/2);
                updating_positions();
            }
            else
            {
                
            }
                
        }
            
    alpha_rot = 30.0;
    std::cout << "BACK IN STANDARD MODE. Press R1 to align the quadrotors" << std::endl;
    button_y=0;
    std::cout << "Formation angle: " << fmod(alpha_or*180/3.141593,360) << " degrees" << std::endl;
    for (int i =0; i<num; i++)
	{
	std::cout << "Robot angles " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << " degrees" << std::endl;
	
	}
        
    }

      else if (button_y!=0)
    	{
    std::cout << "SIMULATION ENDED" << std::endl;
    std_msgs::Float32 comi_x;
    comi_x.data=200;
    com_x_pub.publish(comi_x);
    return;
    	}
	
	else{

	double thresh = 0.00001; // to check
    
    if ((fabs(joy_x)<thresh)&(fabs(joy_y)<thresh))
    {
    c=0;
//	std::cout << "sei qu? 1 " << std::endl;
    }
    else
    {
    double alpha_rif = atan2(-joy_y, joy_x);
	//std::cout << "alpha_rif : " << alpha_rif << " alpha_or: " << alpha_or << std::endl;
    double alpha_k = alpha_or;
	//std::cout << "alpha_k " << alpha_k << std::endl;
    while (alpha_k<-3.141593) {
        alpha_k=alpha_k+2*3.141593;
	//std::cout << " alpha_k minore di PI " << alpha_k << std::endl;
    }
    while (alpha_k>3.141593) {
        alpha_k=alpha_k-2*3.141593;
	//std::cout << "alpha_k maggiore di PI " << alpha_k << std::endl;
    }
    if (fabs(alpha_rif-alpha_k)>3.141593)
    {
	if (alpha_rif < alpha_k)
	{
        alpha_rif=alpha_rif+2*3.141593;
	}
	else
	{alpha_rif=alpha_rif-2*3.141593;}
	
    }
    if (fabs(alpha_rif-alpha_k)<(3.141593/alpha_rot))
    {
        std::cout <<  "GO STRAIGHT!" << std::endl;
        c=1;
    }
    else if (alpha_rif>alpha_k)
    {
	if (f_block==1)
	{f_block=0;
	f_follow=1;
	angle=1;
	//std::cout << "Waiting.. robots too far from their target positions......... " << std::endl;
	c=13;} //90.0
        else {c=2;}
        std::cout <<  "TURN RIGHT!" << std::endl;
        
    
    }
    else if (alpha_rif<alpha_k)
    {
	if (f_block==1)
        {f_block=0;
	f_follow=1;
	angle=1;
	//std::cout << "Waiting.. robots too far from their target positions......... " << std::endl;
	alpha_rot = 30.0 ;
	c=13;} //90.0
        else 
	{c=3;}
	std::cout <<  "TURN LEFT!" << std::endl;
    }
        
    }
        
   }

	}




else {
    if ((button_1!=0))
	{
	c=4;
	}
    else if ((button_2!=0))
	{
	c=5;
	}
   else if ((button_3!=0))
	{c=6;
	}
   else if ((button_4!=0))
	{c=7;
	}
   // ruoto o vado avanti - indietro
   else if (joy_left>0.7)
	{
	if (f_block==1)
	{f_block=0;
	f_follow=1;
	angle=1;
	//std::cout << "Waiting.. robots too far from their target positions......... " << std::endl;
	alpha_rot = 30.0;
	c=13;} //90.0
	else
	{c=3;}
	}
   else if (joy_left<-0.7)
	{
	if (f_block==1)
	{f_block=0;
	f_follow=1;
	angle=1;
	//std::cout << "Waiting.. robots too far from their target positions......... " << std::endl;
	alpha_rot = 30.0;
	c=13;} //90.0
	else
	{c=2;}	
	}
   else if (joy_x>0.7)
	{
	c=1;
	}
   else if (joy_x<-0.7)
	{
	c=10;
	}
   else if (joy_y <-0.7)
	{
	c=9;
	}
  else if (joy_y > 0.7)
	{
	c=8;
	}
   else if (joy_ind ==-1.0)
	{
	c=11;
	}
   else if (joy_ind ==1.0)
	{
	c=12;
	}
   else if (button_5!=0)
	{
	f_block=1;
	f_follow=0;
	//std::cout << "Robots do not rotate when tu" << std::endl;
	usleep(50000);
	alpha_rot = 30.0;
	c=0;
	}
   else if (button_6!=0)
	{
	f_block=0;
	f_follow=1;
	angle=1;
	std::cout << "Robots are going to align......." << std::endl;
	alpha_rot = 30.0; //90.0
	c=13;
	}
        
    else if (button_x!=0)
    {
        alpha_rot = 30.0;
        test2=1;
        f_block=1;
        f_follow=0;
        std::cout << "Rotate all quadrotors together, or press O to select the robot, start to go back to the standard mode" << std::endl;
        int nume=-1;
        
        double alpha_next[num];
        for (int i =0; i<num; i++)
        {
            alpha_next[i]=alpha_or;
        }
        
        while (button_y==0)
        {
            ros::spinOnce();
            if (button_2!=0)
            {nume++;
 		
                if (nume<num)
                {
                std::cout << "Selected for the rotation: robot n.  " << nume << std::endl;
		usleep(500000);
                }
                else

                {nume=-1;
                  std::cout << "All quadrotors together " << std::endl;  
		usleep(500000);
		}
                    
            }
            
            if ((nume ==-1)&(joy_left<-0.7))
            {
                std::cout << "Rotating all quadrotors together " << std::endl;
                for (int i =0; i<num; i++)
                {
                    alpha_next[i]=alpha_next[i]+ (3.141593/alpha_rot);
		    std::cout << "Robot angle " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << " degrees" << std::endl;
                }
                for (int i =0; i<num; i++)
                {
                zang_targ[i] = sin(-alpha_next[i]/2);
                w_targ[i] = cos(-alpha_next[i]/2);
                }
                updating_positions();
            }
            else if ((nume ==-1)&(joy_left>0.7))
            {
                std::cout << "Rotating all quadrotors together " << std::endl;
               for (int i =0; i<num; i++)
                {
                    alpha_next[i]=alpha_next[i]- (3.141593/alpha_rot);	
		   std::cout << "Robot angle " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << "degrees" << std::endl;
                }
               for (int i =0; i<num; i++)
                {
                zang_targ[i] = sin(-alpha_next[i]/2);
                w_targ[i] = cos(-alpha_next[i]/2);
                }
                updating_positions();
            }
            else if (joy_left<-0.7)
            {
                alpha_next[nume]=alpha_next[nume]+ (3.141593/alpha_rot);
		std::cout << "Robot angle " << nume << " = " << fmod(alpha_next[nume]*180/3.141593,360) << " degrees" << std::endl;
                zang_targ[nume] = sin(-alpha_next[nume]/2);
                w_targ[nume] = cos(-alpha_next[nume]/2);
                updating_positions();
            }
            else if (joy_left>0.7)
            {
                alpha_next[nume]=alpha_next[nume]- (3.141593/alpha_rot);
		std::cout << "Robot angle " << nume << " = " << fmod(alpha_next[nume]*180/3.141593,360) << " degrees" << std::endl;
                zang_targ[nume] = sin(-alpha_next[nume]/2);
                w_targ[nume] = cos(-alpha_next[nume]/2);
                updating_positions();
            }
            else
            {
                
            }
                
        }
            
    alpha_rot = 30.0;
     std::cout << "BACK IN STANDARD MODE. Press R1 to align the quadrotors" << std::endl;
    button_y=0;
    std::cout << "Formation angle : " << fmod(alpha_or*180/3.141593,360) << " degrees" << std::endl;
    for (int i =0; i<num; i++)
	{
	std::cout << "Robot angle " << i << " = " << fmod(alpha_next[i]*180/3.141593,360) << " degrees" << std::endl;
	
	}
        
    }
   
    else if (button_y!=0)
    	{
    std::cout << "SIMULATION ENDED" << std::endl;
    std_msgs::Float32 comi_x;
    comi_x.data=200;
    com_x_pub.publish(comi_x);
    return;
    	}
    

   else 
	{
	c=0;
	}
     } // modalità in soggettiva
  

        switch(c)
            {
		case 0:
			usleep(200);
			break;
		case 14:
			test2=1;
		case 13:
			test2=0;
			variable=1;
			while(variable)
			{fromCoMtopositions();}
			angle=0;
			break;
                case 8:
                    ROS_DEBUG("LEFT");
                    ros::spinOnce();
                    //xom=xom+spost*sin(alpha_or);
                    //yom=yom+spost*cos(alpha_or);
		    xom=xom;
		    yom=yom;
                    zom=zom;
                    test2=1;
                    perp=1;
                    rot=0;
                    fromCoMtopositions();
	            break;
                case 9:
                    ROS_DEBUG("RIGHT");
                    ros::spinOnce();
                    //xom=xom-spost*sin(alpha_or);
                    //yom=yom-spost*cos(alpha_or);
		    xom=xom;
		    yom=yom;
                    zom=zom;
                    test2=1;
                    rot=0;
                    perp=1;
                    fromCoMtopositions();
                    break;
                case 1:
                    ROS_DEBUG("UP");
                    ros::spinOnce();
                    xom=xom+spost*cos(alpha_or);
                    yom=yom-spost*sin(alpha_or);
                    zom=zom;
                    test2=1;
                    rot=0;  
                    perp=0;
                    fromCoMtopositions();
                    break;
                case 10:
                    ROS_DEBUG("DOWN");
                    ros::spinOnce();
                    xom=xom-spost*cos(alpha_or);
                    yom=yom+spost*sin(alpha_or);
                    zom=zom;
                    rot=0;
                    test2=1;
                    perp=0;
                    fromCoMtopositions();
                    break;
                case 12:
                    ROS_DEBUG("SU");
                    ros::spinOnce();
                    xom=xom;
                    yom=yom;
                    zom=zom+spost;
                    test2=1;
                    rot=0;
                    fromCoMtopositions();
                    break;
                case 11:
                    ROS_DEBUG("GIU");
                   ros::spinOnce();
                    xom=xom;
                    yom=yom;
                    zom=zom-spost;
		    if (zom<0)
		    {zom=0;}
                    rot=0;
                    test2=1; 
                    fromCoMtopositions();
                    break;
                case 2:
                    ROS_DEBUG("TURN RIGHT");
                    ros::spinOnce();
                    zom=zom;
                    alpha_or=alpha_or+(3.141593/alpha_rot);
                    xom=xom;
                    yom=yom;
                    test2=1;
                    rot=1;
                    right=1;
                    left=0;
                    fromCoMtopositions();
                    break;
                case 3:
                    ROS_DEBUG("TURN LEFT");
                    ros::spinOnce();
                    zom=zom;
                    alpha_or=alpha_or-(3.141593/alpha_rot);
                    xom=xom;
                    yom=yom;
                    test2=1;
                    rot=1;
                    right=0;
                    left=1;
                    fromCoMtopositions();
                    break;
                case 4:
		    ros::spinOnce();
                    formation=1;
                    test2=0;
                    fromCoMtopositions();
                    break;
                case 5:
                    ros::spinOnce();
                    formation=2;
                    test2=0;
                    fromCoMtopositions();
                    break;
                case 6:
                    ros::spinOnce();
                    formation=3;
                    test2=0;
                    fromCoMtopositions();
                    break;
                case 7:
                    ros::spinOnce();
                    formation=4;
                    test2=0;
                    fromCoMtopositions();
                    break;
            }
	}
    return;
}





int main(int argc, char** argv)
{

  std::cout << "SIMULAZIONE DI VOLO IN FORMAZIONE" << std::endl;
  std::cout << "Vuoi salvare i dati? s/n" << std::endl;

  char k;
	
  std::cin >> k;
  if (k=='s')
  {saving=1;
   std::cout << "Inserire l'id dell'utente" << std::endl;	
   std::cin >> id;
   }
   std::cout << "Inserire la tipologia di controllo (0 per la visuale dall' alto (mod A), 1 per la visuale dall'alto (mod B), 2 per la visuale in soggettiva" << std::endl;
   std::cin >> id2;

   std::cout << "Inserire la formazione di partenza (1 cuneo, 2 colonna, 3 linea o 4 diamante)" << std::endl;
   std::cin >> formation;

  if (k=='s')
  {	
   std::cout << "Inserire il numero della prova" << std::endl;
   std::cin >> id3;
  }

  ros::init(argc, argv, "talker");

  num=strtol(argv[1],NULL,0);
  //std::cout << num << std::endl;
  ros::NodeHandle nh_;
  ros::NodeHandle n_;

  x_0[0] = 12.75;
  y_0[0]= 3.00;
  r[0] = 2.7;

  x_0[1] = 23.325;
  y_0[1]= -1.65;
  r[1] = 2.7;

  x_0[2] = 14.175;
  y_0[2]= 11.3;
  r[2] = 2.7;
    
  x_0[3] = 22.150;
  y_0[3]= 13.475;
  r[3] = 2.7;
    
  x_0[4] = 2.525;
  y_0[4]= -10.775;
  r[4] = 2.7;
    
  x_0[5] = -0.675;
  y_0[5]= 7.00;
  r[5] = 2.7;
    
  x_0[6] = 10.25;
  y_0[6]= 28.075;
  r[6] = 2.7;

  x_0[7] = 16.55;
  y_0[7]= 27.15;
  r[7] = 2.7;

  x_0[8] = 28.6;
  y_0[8]= 4.625;
  r[8] = 2.7;

  x_0[9] = -3.975;
  y_0[9]= -11.050;
  r[9] = 2.7;
  
  x_0[10] = -8.15;
  y_0[10]= -7.325;
  r[10] = 2.7;
  
  x_0[11] = 5.65;
  y_0[11]= 7.425;
  r[11] = 2.7;

  x_0[12] = 22.4;
  y_0[12]= 26.1;
  r[12] = 2.7;

  x_0[13] = 0.625;
  y_0[13]= 14.175;
  r[13] = 2.7;

  x_0[14] = 7.825;
  y_0[14]= 13.975;
  r[14] = 2.7;

  x_0[15] = 11.775;
  y_0[15]= 32.825;
  r[15] = 2.7;
  
  x_0[16] = 2.925;
  y_0[16]= 25.6;
  r[16] = 2.7;
  
  x_0[17] = -7.1750;
  y_0[17]= 16.375;
  r[17] = 2.7;

  x_0[18] = -0.6 ;
  y_0[18]= -5.05;
  r[18] = 2.7;
  
  x_0[19] = 17.825;
  y_0[19]= 11.375;
  r[19] = 1.3;

  x_0[20] = 3.95;
  y_0[20]= 29.175;
  r[20] = 1.3;

  x_0[21] = 7.325;
  y_0[21]= 30.5;
  r[21] = 1.3;
  
  x_0[22] = 13.325;
  y_0[22]= -16.7;
  r[22] = 1.3;
  
  x_0[23] = 16.1;
  y_0[23]= -7.6;
  r[23] = 2.2;
    
  x_0[24] = 13.45;
  y_0[24]= -7.275;
  r[24] = 2.7;

  signal(SIGINT,quit);
  synchro_pub = nh_.advertise<std_msgs::Int32>("synchro", 1);
  info1_pub = nh_.advertise<std_msgs::Int32>("info1", 1);
  info2_pub = nh_.advertise<std_msgs::Int32>("info2", 1);
  info3_pub = nh_.advertise<std_msgs::Int32>("info3", 1);  



  ros::Subscriber quadcopPos0_sub = n_.subscribe("quadcopPos_0", 100, ROSposition0_received);
  ros::Subscriber quadcopPos1_sub = n_.subscribe("quadcopPos_1", 100, ROSposition1_received);
  ros::Subscriber quadcopPos2_sub = n_.subscribe("quadcopPos_2", 100, ROSposition2_received); 
  ros::Subscriber quadcopPos3_sub = n_.subscribe("quadcopPos_3", 100, ROSposition3_received);
  ros::Subscriber quadcopPos4_sub = n_.subscribe("quadcopPos_4", 100, ROSposition4_received);
  ros::Subscriber quadcopPos5_sub = n_.subscribe("quadcopPos_5", 100, ROSposition5_received);
  ros::Subscriber quadcopPos6_sub = n_.subscribe("quadcopPos_6", 100, ROSposition6_received);
  ros::Subscriber quadcopPos7_sub = n_.subscribe("quadcopPos_7", 100, ROSposition7_received);
  ros::Subscriber quadcopPos8_sub = n_.subscribe("quadcopPos_8", 100, ROSposition8_received);
  ros::Subscriber quadcopPos9_sub = n_.subscribe("quadcopPos_9", 100, ROSposition9_received);
    
  ros::Subscriber quadcop_realPos0_sub = n_.subscribe("quadcop_realPos_0", 100, ROSrealposition0_received);
  ros::Subscriber quadcop_realPos1_sub = n_.subscribe("quadcop_realPos_1", 100, ROSrealposition1_received);
  ros::Subscriber quadcop_realPos2_sub = n_.subscribe("quadcop_realPos_2", 100, ROSrealposition2_received);
  ros::Subscriber quadcop_realPos3_sub = n_.subscribe("quadcop_realPos_3", 100, ROSrealposition3_received);
  ros::Subscriber quadcop_realPos4_sub = n_.subscribe("quadcop_realPos_4", 100, ROSrealposition4_received);
  ros::Subscriber quadcop_realPos5_sub = n_.subscribe("quadcop_realPos_5", 100, ROSrealposition5_received);
  ros::Subscriber quadcop_realPos6_sub = n_.subscribe("quadcop_realPos_6", 100, ROSrealposition6_received);
  ros::Subscriber quadcop_realPos7_sub = n_.subscribe("quadcop_realPos_7", 100, ROSrealposition7_received);
  ros::Subscriber quadcop_realPos8_sub = n_.subscribe("quadcop_realPos_8", 100, ROSrealposition8_received);
  ros::Subscriber quadcop_realPos9_sub = n_.subscribe("quadcop_realPos_9", 100, ROSrealposition9_received);

  pos_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_0", 1);
  pos_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_1", 1);
  pos_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_2", 1);
  pos_pub_3 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_3", 1);
  pos_pub_4 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_4", 1);
  pos_pub_5 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_5", 1);
  pos_pub_6 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_6", 1);
  pos_pub_7 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_7", 1);
  pos_pub_8 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_8", 1);
  pos_pub_9 = nh_.advertise<geometry_msgs::PoseStamped>("targetObjPos_9", 1);

  
  or_pub = nh_.advertise<std_msgs::Float32>("orient",1);

  com_x_pub = nh_.advertise<std_msgs::Float32>("com_x_pub",1);	
  com_y_pub = nh_.advertise<std_msgs::Float32>("com_y_pub",1);

  c_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_0", 1);
  c_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_1", 1);
  c_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_2", 1);
  c_pub_3 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_3", 1);
  c_pub_4 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_4", 1);
  c_pub_5 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_5", 1);
  c_pub_6 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_6", 1);
  c_pub_7 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_7", 1);
  c_pub_8 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_8", 1);
  c_pub_9 = nh_.advertise<geometry_msgs::PoseStamped>("cPos_9", 1);
    
  ros::Subscriber joy_sub = nh_.subscribe("joy", 10, &joyCallback);  
    
  //std::cout << "before sleep" << std::endl;
  sleep(3);
  ros::spinOnce();
  //std::cout << "after sleep" << std::endl;
  test2=0;
  fromCoMtopositions();
  //ros::spin();
  test2=0;
  // std::cout << "qua ci arrivi?" << std::endl;
  keyLoop();
  
  return(0);
}




