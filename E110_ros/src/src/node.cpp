#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/console.h>
#include <time.h>
#include "../sdk/include/bluesea.h"
#include "../sdk/include/usererror.h"
#define BLUESEA2_VERSION "1.0"

ros::Time g_lasttimestamp(0, 0);
double time_increment = 0;

bool get_range_param(ros::NodeHandle nh, const char *name, Range &range)
{
	std::vector<double> rg;
	if (nh.getParam(name, rg))
	{
		if (rg.size() == 2 && rg[0] < rg[1])
		{
			range.min = rg[0] * 180 / M_PI;
			range.max = rg[1] * 180 / M_PI;
			return true;
		}
	}
	return false;
}

bool ProfileInit(ros::NodeHandle priv_nh, ArgData &argdata)

{
	priv_nh.param("number", argdata.num, 1);
	priv_nh.param("type", argdata.type, std::string("uart"));
	priv_nh.param("frame_id", argdata.frame_id, std::string("LH_laser"));
	priv_nh.param("dev_id", argdata.dev_id, -1);
	priv_nh.param("port", argdata.connectargs.arg1, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", argdata.connectargs.arg2, 500000);
	bool inverted, reversed;
	priv_nh.param("inverted", argdata.inverted, false);
	priv_nh.param("reversed", argdata.reversed, false);
	// data output
	priv_nh.param("output_scan", argdata.output_scan, true);	  // true: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", argdata.output_cloud, false);	  // false: enable output xyz format, 0 : disable
	priv_nh.param("output_cloud2", argdata.output_cloud2, false); // false: enable output xyz format, 0 : disable

	//  angle filter
	bool with_angle_filter;
	double min_angle, max_angle;
	priv_nh.param("with_angle_filter", argdata.with_angle_filter, false); // true: enable angle filter, false: disable
	priv_nh.param("min_angle", argdata.min_angle, 0.0);				  // angle filter's low threshold, default value: -pi
	priv_nh.param("max_angle", argdata.max_angle, 360.0);				  // angle filters' up threashold, default value: pi

	// range limitation
	double min_dist, max_dist;
	priv_nh.param("min_dist", argdata.min_dist, 0.1);  // min detection range, default value: 0M
	priv_nh.param("max_dist", argdata.max_dist, 50.0); // max detection range, default value: 9999M
	// customize angle filter
	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(priv_nh, name, range))
			break;
		argdata.masks.push_back(range);
	}
	return true;
}

void PublishLaserScan(ros::Publisher &laser_pub, FrameData_LD framedata, ArgData argdata)
{
	sensor_msgs::LaserScan msg;
	int N = framedata.N;
	if (g_lasttimestamp.sec == 0)
	{
		g_lasttimestamp = ros::Time::now();
		return;
	}
	ros::Time timestamp = ros::Time::now();
	msg.header.stamp.sec = timestamp.sec;
	msg.header.stamp.nsec = timestamp.nsec;
	double ti = g_lasttimestamp.sec + g_lasttimestamp.nsec / 1000000000.0;
	double tx = timestamp.sec + timestamp.nsec / 1000000000.0;
	msg.scan_time = tx - ti;
	msg.time_increment = msg.scan_time / N;

	g_lasttimestamp.sec = timestamp.sec;
	g_lasttimestamp.nsec = timestamp.nsec;

	msg.header.frame_id = argdata.frame_id;
	msg.range_min = argdata.min_dist;
	msg.range_max = argdata.max_dist;

	// msg.angle_min = framedata.points[0].angle/18000.0*M_PI;
	// msg.angle_max = framedata.points[framedata.N-1].angle/18000.0*M_PI;
	msg.angle_min = 0.;
	msg.angle_max = 2.0 * M_PI;
	// DEBUG("%f %f\n",msg.angle_min,msg.angle_max);
	msg.angle_increment = (msg.angle_max - msg.angle_min) / (float)(N - 1);
	msg.ranges.assign(N, std::numeric_limits<float>::quiet_NaN());
	msg.intensities.assign(N, std::numeric_limits<float>::quiet_NaN());

	for (int idx = 0; idx < N; idx++)
	{
		LidarPoint_LD point = framedata.points[idx];
		float angle = point.angle/180*M_PI;
		double range = point.distance / 1000.0;
		float intensity = point.confidence;
		int index = static_cast<int>(ceil((angle - msg.angle_min) / msg.angle_increment));

		int index_anticlockwise = N - index - 1;
		if (!argdata.inverted)
			index_anticlockwise = index;

		if (range == 0 || range > argdata.max_dist || range < argdata.min_dist)
		{
			range = std::numeric_limits<float>::quiet_NaN();
		}

		if (argdata.with_angle_filter)
		{ // Angle crop setting, Mask data within the set angle range
			if ((angle >= (argdata.min_angle/180*M_PI)) && (angle <= (argdata.max_angle/180*M_PI)))
			{
				range = std::numeric_limits<float>::quiet_NaN();
				intensity = std::numeric_limits<float>::quiet_NaN();
			}
		}
		// DEBUG("%d %d  %d  %f %f %f",N,index_anticlockwise,index,point.angle, msg.angle_min,msg.angle_increment);

		if (std::isnan(msg.ranges[index_anticlockwise]))
		{
			msg.ranges[index_anticlockwise] = range;
		}
		else
		{
			if (range < msg.ranges[index_anticlockwise])
			{
				msg.ranges[index_anticlockwise] = range;
			}
		}
		msg.intensities[index_anticlockwise] = intensity;
	}

	laser_pub.publish(msg);
}

void CallBackMsg(int msgtype, void *param, int length)
{
	switch (msgtype)
	{
	// 实时雷达点云数据
	case 1:
	{
		// UserData *pointdata = (UserData *)param;
		// //printf("frame idx:%d  %s\t%d \t num:%d \n", pointdata->idx, pointdata->connectArg1, pointdata->connectArg2, pointdata->framedata.N);

		// if (g_lasttimestamp.sec == 0)
		// {
		// 	g_lasttimestamp = ros::Time::now();
		// 	return;
		// }
		// int N = pointdata->framedata.N;
		// ros::Time timestamp = ros::Time::now();
		// sensor_msgs::LaserScan msg;
		// msg.header.stamp.sec = timestamp.sec;
		// msg.header.stamp.nsec = timestamp.nsec;
		// double ti = g_lasttimestamp.sec + g_lasttimestamp.nsec / 1000000000.0;
		// double tx = timestamp.sec + timestamp.nsec / 1000000000.0;
		// msg.scan_time = tx - ti;
		// msg.time_increment = msg.scan_time / N;

		// g_lasttimestamp.sec=timestamp.sec;
		// g_lasttimestamp.nsec=timestamp.nsec;

		// //printf("%ld %ld\n",timestamp.sec,timestamp.nsec);
		// msg.header.frame_id = argdata.frame_id;
		// msg.range_min = argdata.min_dist;
		// msg.range_max = argdata.max_dist;

		// msg.angle_min = pointdata->framedata.points[0].angle/18000.0*M_PI;
		// msg.angle_max = pointdata->framedata.points[pointdata->framedata.N-1].angle/18000.0*M_PI;
		// DEBUG("%f %f\n",msg.angle_min,msg.angle_max);
		// msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
		// msg.intensities.resize(N);
		// msg.ranges.resize(N);
		// //int idx=0;
		// for (int idx = 0; idx < N; idx++)
		// {
		// 	LidarPoint_LD point = pointdata->framedata.points[idx];
		// 	double d = point.distance / 1000.0;
		// 	if (d == 0 || d>argdata.max_dist || d < argdata.min_dist)
		// 	{
		// 		msg.ranges[idx] = std::numeric_limits<float>::infinity();
		// 	}
		// 	else
		// 	{
		// 		msg.ranges[idx] = d;
		// 	}
		// 	msg.intensities[idx] = point.confidence;
		// 	//idx++;
		// }
		//  for (int i = 0; i <1; i++)
		//  {
		//  	printf("%s\t%d \t%f\t%d\t%d\t%d\n", pointdata->connectArg1, pointdata->connectArg2,  pointdata->framedata.points[i].angle, pointdata->framedata.points[i].distance, pointdata->framedata.points[i].confidence,pointdata->idx);
		//  }
		// g_laser_pubs.publish(msg);
		break;
	}
	// 打印信息(也可以作为日志写入)
	case 8:
	{
		char result[512];
		memcpy(result, param, length);
		printf("info: %s\n", result);
		break;
	}
	case 9:
	{
		char result[512];
		memcpy(result, param, length);
		printf("error: %s\n", result);
		break;
	}
	}
	fflush(stdout);
}
int main(int argc, char **argv)
{
	ROS_INFO("ROS VERSION:%s\n", BLUESEA2_VERSION);

	ros::init(argc, argv, "bluesea2_laser_publisher");
	ros::NodeHandle node_handle;
	ros::NodeHandle priv_nh("~");
	// init launch arg
	ArgData argdata;
	ProfileInit(priv_nh, argdata);
	// create topic
	ros::Publisher laser_pubs, cloud_pubs, cloud2_pubs;
	if (argdata.output_cloud)
	{
		cloud_pubs = node_handle.advertise<sensor_msgs::PointCloud>(argdata.connectargs.cloud_topics, 50);
	}
	if (argdata.output_cloud2)
	{
		cloud2_pubs = node_handle.advertise<sensor_msgs::PointCloud2>(argdata.connectargs.cloud_topics, 50);
	}
	if (argdata.output_scan)
	{
		laser_pubs = node_handle.advertise<sensor_msgs::LaserScan>(argdata.connectargs.scan_topics, 50);
	}

	BlueSeaLidarSDK *lidarSDK = BlueSeaLidarSDK::getInstance();
	int lidarID = lidarSDK->addLidar(argdata.connectargs.arg1, argdata.connectargs.arg2);
	// lidarSDK->setCallBackPtr(lidarID, CallBackMsg);
	lidarSDK->connect(lidarID);
	while (ros::ok())
	{
		ros::spinOnce();

		int ret = lidarSDK->GetAllFans(lidarID);
		RunConfig *cfg = lidarSDK->getLidar(lidarID);
		// DEBUG("%d\n",ret);
		if (ret == 1)
		{

			PublishLaserScan(laser_pubs, cfg->framedata, argdata);
			cfg->framedata.N = 0;
			// DEBUG("fabu %d \n",cfg->framedata.points[0].distance);
		}
		else if (ret == -1)
		{
			cfg->framedata.N = 0;
		}
		else
			ros::Duration(0.001).sleep();
	}
	return 0;
}
