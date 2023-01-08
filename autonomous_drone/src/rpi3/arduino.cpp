#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int8.h>

int dist_sensor_1;
int dist_sensor_2;
int dist_sensor_3;
int dist_sensor_4;
int dist_sensor_5;
int dist_sensor_6;

int dist_min = 3;
int dist_max = 400;
int D2 = 100;
int D1 = 200;

int cmd = 88;
int cmd_final = 88;
const int cmd_arr_size = 10; 
int cmd_arr[cmd_arr_size];


void callback_1(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_1 = (msg->range * 100)/1;
}

void callback_2(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_2 = (msg->range * 100)/1;
}

void callback_3(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_3 = (msg->range * 100)/1;
}

void callback_4(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_4 = (msg->range * 100)/1;
}

void callback_5(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_5 = (msg->range * 100)/1;
}

void callback_6(const sensor_msgs::Range::ConstPtr& msg)
{	
	dist_sensor_6 = (msg->range * 100)/1;
}

int slide()
{
	if(dist_sensor_1 < D1)
	{
  		if(dist_sensor_5 > D1)
   			cmd = 76;
   		else if(dist_sensor_3 > D1)
   			cmd = 82;
   		else if(dist_sensor_4 > D1)
   			cmd = 98;
   		else
   			cmd = 83;
	}
  
	else if(dist_sensor_5 < D1)
	{
   		if(dist_sensor_1 > D1)
   			cmd = 70;
   		else if(dist_sensor_4 > D1)
   			cmd = 66;
   		else if(dist_sensor_3 > D1)
			cmd = 114;
		else
			cmd = 83;
  	}
  
  	else if(dist_sensor_3 < D1)
  	{
		if(dist_sensor_1 > D1)
			cmd = 70;
		else if(dist_sensor_4 > D1)
			cmd = 66;
		else if(dist_sensor_5 > D1)
			cmd = 108;
		else
			cmd = 83;
  	}

	else if(dist_sensor_2 < D1)
	{
		if(dist_sensor_5 > D1)
			cmd = 76;
		else if(dist_sensor_4 > D1)
			cmd = 66;
		else
			cmd = 83;
	}
	
	else if(dist_sensor_6 < D1)
	{
		if(dist_sensor_3 > D1)
			cmd = 82;
		else if(dist_sensor_4 > D1)
			cmd = 66;
		else
			cmd = 83;
	}

  	else if(dist_sensor_4 < D1)
  	{
		if(dist_sensor_3 > D1)
			cmd = 82;  
   		else if(dist_sensor_5 > D1)
			cmd = 76; 
		else if(dist_sensor_1 > D1)
			cmd = 102;
		else
			cmd = 83;
  	}
 
  	else
 		cmd = 88;
  
  	return cmd;
}

int repulse()
{
	if(dist_sensor_1 < D2)
	{
		if(dist_sensor_4 > D2)
			cmd = 66;
	 	else if(dist_sensor_3 > D2)
			cmd = 82;
		else if(dist_sensor_5 > D2)
			cmd = 76;
		else
			cmd = 83;
	}
  
	else if(dist_sensor_5 < D2)
	{
		if(dist_sensor_3 > D2)
			cmd = 82;
		else if(dist_sensor_4 > D2)
			cmd = 66;
		else if(dist_sensor_1 > D2)
			cmd = 70;
		else
			cmd = 83;
 	}
  
	else if(dist_sensor_3 < D2)
	{
		if(dist_sensor_5 > D2)
			cmd = 76;
		else if(dist_sensor_4 > D2)
			cmd = 66;
		else if(dist_sensor_1 > D2)
			cmd = 70;
		else
			cmd = 83;
	}

	else if(dist_sensor_2 < D2)
	{
		if(dist_sensor_5 > D2)
			cmd = 76;
		else if(dist_sensor_4 > D2)
			cmd = 66;
		else
			cmd = 83;
	}
	
	else if(dist_sensor_6 < D2)
	{
		if(dist_sensor_3 > D2)
			cmd = 82;
		else if(dist_sensor_4 > D2)
			cmd = 66;
		else
			cmd = 83;
	}

	else if(dist_sensor_4 < D2)
	{
		if(dist_sensor_1 > D2)
			cmd = 70;
		else if(dist_sensor_5 > D2)
			cmd = 76;
		else if(dist_sensor_3 > D2)
			cmd = 82;
		else
			cmd = 83;
	}

	else
		cmd = 88;
   
	return cmd;
}

int mode(int cmd_arr[], int cmd_arr_size)
{
	int i, j, temp;
	for(i=0;i<cmd_arr_size-1;i++)
	{
		for(j=0;j<cmd_arr_size-i-1;j++)
		{
			if(cmd_arr[j] > cmd_arr[j+1])
			{
				temp = cmd_arr[j];
				cmd_arr[j] = cmd_arr[j+1];
				cmd_arr[j+1] = temp;
			}
		}
	}
	int max_count = 1, curr_count = 1, res = cmd_arr[0];
	for(i=1;i<cmd_arr_size;i++)
	{
		if(cmd_arr[i] == cmd_arr[i-1])
		curr_count++;
		else
		{
			if(curr_count > max_count)
	  		{
				max_count = curr_count;
				res = cmd_arr[i-1];
			}
			curr_count = 1;
	  	}
  	}
	if (curr_count > max_count)
	{
		max_count = curr_count;
		res = cmd_arr[cmd_arr_size-1];
	}
	return res;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "arduino_node");
	ros::NodeHandle n;
	
	ros::Subscriber sub_1 = n.subscribe("/distance_sensor/1", 10, callback_1);
	ros::Subscriber sub_2 = n.subscribe("/distance_sensor/2", 10, callback_2);
	ros::Subscriber sub_3 = n.subscribe("/distance_sensor/3", 10, callback_3);
	ros::Subscriber sub_4 = n.subscribe("/distance_sensor/4", 10, callback_4);
	ros::Subscriber sub_5 = n.subscribe("/distance_sensor/5", 10, callback_5);
	ros::Subscriber sub_6 = n.subscribe("/distance_sensor/6", 10, callback_6);
	
	ros::Publisher cmd_publisher = n.advertise<std_msgs::Int8>("cmd_topic", 10);
	
	ROS_INFO("[Arduino] : Executing obstacle avoidance algorithm");
	ros::Rate rate(20);
	
	while(ros::ok())
	{
		for(int k=0;k<cmd_arr_size;k++)
  		{
  			if(dist_sensor_1 > D2 && dist_sensor_2 > D2 && dist_sensor_3 > D2 && dist_sensor_4 > D2 && dist_sensor_5 > D2 && dist_sensor_6 > D2)
				cmd_arr[k] = slide();
			else if(dist_sensor_1 > dist_min && dist_sensor_2 > dist_min && dist_sensor_3 > dist_min && dist_sensor_4 > dist_min && dist_sensor_5 > dist_min && dist_sensor_6 > dist_min)
				cmd_arr[k] = repulse();
			else
				cmd_arr[k] = 88;
  		}
 		cmd_final = mode(cmd_arr, cmd_arr_size);
 		
 		std_msgs::Int8 msg;
 		msg.data = cmd_final;
 		cmd_publisher.publish(msg);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
