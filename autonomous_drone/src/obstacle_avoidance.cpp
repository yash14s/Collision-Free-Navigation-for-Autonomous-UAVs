#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

float sensor_data[6];
float distance[6];
const int window_size = 11;
const float D1 = 2.0;
const float D2 = 1.0;

void callback_sensor1(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[0] = msg->range;
}

void callback_sensor2(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[1] = msg->range;
}

void callback_sensor3(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[2] = msg->range;
}

void callback_sensor4(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[3] = msg->range;
}

void callback_sensor5(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[4] = msg->range;
}

void callback_sensor6(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_data[5] = msg->range;
}

//Apply Median filter on sensor x's readings
float filter_sensor_data(int x)
{
	float data_array[window_size], key;
	int i, j;
	//Populate the array
	for (i = 0; i < window_size; i++)
	{
		data_array[i] = sensor_data[x];
	}
	//Insertion sort
	for (i = 1; i < window_size; i++)
	{
		key = data_array[i];
		j = i - 1;
		while (j >= 0 && data_array[j] > key)
		{
			data_array[j+1] = data_array[j];
			j = j - 1;
		}
		data_array[j+1] = key;
	}
	//Return median value
	return data_array[(window_size+1)/2];
}

float* slide()
{
	static float v[2]; //v = [vx,vy]
	float vx = 0, vy = 0;
	const float speed = 0.5;

	if(distance[0] < D1)
	{
		if(distance[4] > D1)
			vx = 0, vy = -speed;
		else if(distance[2] > D1)
			vx = 0, vy = speed;
		else
			vx = 0, vy = 0;
	}
	else if(distance[4] < D1)
	{
		if(distance[0] > D1)
			vx = speed, vy = 0;
		else if(distance[3] > D1)
			vx = -speed, vy = 0;
		else
			vx = 0, vy = 0;
	}
	else if(distance[2] < D1)
	{
		if(distance[0] > D1)
			vx = speed, vy = 0;
		else if(distance[3] > D1)
			vx = -speed, vy = 0;
		else
			vx = 0, vy = 0;
	}
	else if(distance[1] < D1)
	{
		if(distance[4] > D1)
			vx = 0, vy = -speed;
		else if(distance[3] > D1)
			vx = -speed, vy = 0;
		else
			vx = 0, vy = 0;
	}
	else if(distance[5] < D1)
	{
		if(distance[2] > D1)
			vx = 0, vy = speed;
		else if(distance[3] > D1)
			vx = -speed, vy = 0;
		else
			vx = 0, vy = 0;
	}
	else if(distance[3] < D1)
	{
		if(distance[2] > D1)
			vx = 0, vy = speed;
		else if(distance[4] > D1)
			vx = 0, vy = -speed;
		else
			vx = 0, vy = 0;
	}

	v[0] = vx, v[1] = vy;
	return v;
}

float* repulse()
{
	static float v[2]; //v = [vx,vy]
	float vx = 0, vy = 0;
	const float k1 = 0.38, k2 = 0.86;
	float speed = 0.0;
	const float theta[6] = {0.0, M_PI/6.0, M_PI/2.0, M_PI, 3*M_PI/2.0, 11*M_PI/6.0};
	const float vmin = 0.3, vmax = 0.75;

	for(int i = 0; i < 6;i++)
	{
		if(distance[i] < D2)
		{
			speed = (-1) * k1 * pow((distance[i] - (D2 + k2)), 2);
			vx += speed * cos(theta[i]);
			vy += speed * sin(theta[i]);
		}
	}
	if(abs(vx) > vmax)
	{
		vx > 0 ? vx = vmax : vx = -vmax;
	}
	if(abs(vy) > vmax)
	{
		vy > 0 ? vy = vmax : vy = -vmax;
	}
	if(abs(vx) < vmin && abs(vx) > 0.01)
	{
		vx > 0 ? vx = vmin : vx = -vmin;
	}
	if(abs(vy) < vmin && abs(vy) > 0.01)
	{
		vy > 0 ? vy = vmin : vy = -vmin;
	}

	v[0] = vx, v[1] = vy;
	return v;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_avoidance_algorithm_node");
	ros::NodeHandle n;

	ros::Subscriber sub_sensor1 = n.subscribe("/distance_sensor/1", 10, callback_sensor1);
	ros::Subscriber sub_sensor2 = n.subscribe("/distance_sensor/2", 10, callback_sensor2);
	ros::Subscriber sub_sensor3 = n.subscribe("/distance_sensor/3", 10, callback_sensor3);
	ros::Subscriber sub_sensor4 = n.subscribe("/distance_sensor/4", 10, callback_sensor4);
	ros::Subscriber sub_sensor5 = n.subscribe("/distance_sensor/5", 10, callback_sensor5);
	ros::Subscriber sub_sensor6 = n.subscribe("/distance_sensor/6", 10, callback_sensor6);

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("avoidance_velocity_topic",10);
	geometry_msgs::Twist velocity_msg;

	ROS_INFO("Executing obstacle avoidance algorithm");
	ros::Rate rate(10);

	while(ros::ok())
	{
		for (int i = 0; i < 6; i++)
		{
			distance[i] = filter_sensor_data(i);
			//ROS_INFO("%d:%f \n", i+1, distance[i]);
		}
		float vx, vy;
		if(distance[0] > D1 && distance[1] > D1 && distance[2] > D1 && distance[3] > D1 && distance[4] > D1 && distance[5] > D1)
		{
			vx = NAN, vy = NAN;
		}
		else if(distance[0] > D2 && distance[1] > D2 && distance[2] > D2 && distance[3] > D2 && distance[4] > D2 && distance[5] > D2)
		{
			float* v = slide();
			vx = v[0], vy = v[1];
		}
		else
		{
			float* v = repulse();
			vx = v[0], vy = v[1];
		}
		ROS_INFO("Avoidance velocity: vx = %f , vy = %f",vx,vy);
 		velocity_msg.linear.x = vx;
		velocity_msg.linear.y = vy;
 		velocity_publisher.publish(velocity_msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
