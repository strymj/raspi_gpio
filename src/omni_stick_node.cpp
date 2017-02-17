#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
#include <string>
#include <tf/transform_datatypes.h>   // quaternion -> rpy

//#include <tf/transform_listener.h>
//tf::TransformListener listener;

using namespace std;

bool markerflag = false;
bool poseflag = false;
bool tactswflag = false;
bool sensorflag = false;
double pose[3] = {};
double motion[3] = {};
double sensorvalue[9] = {};

Omni omni;

//void correctionCallback(const std_msgs::Float32MultiArray& msg)
//{
//	correctioncmd[0] = msg.data[0];
//	correctioncmd[1] = msg.data[1];
//	correctioncmd[2] = msg.data[2];
//	correctioncmdflag = true;
//}


void markerflagCallback(const std_msgs::Bool& msg)
{
	markerflag = msg.data;
}

void markerposeCallback(const geometry_msgs::PoseStamped& msg)
{
	tf::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	tf::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	pose[0] = msg.pose.position.x;
	pose[1] = msg.pose.position.y;
	pose[2] = yaw;
	//cout<<"pose_x : "<<pose[0]<<endl;
	//cout<<"pose_y : "<<pose[1]<<endl;
	//cout<<"pose_t : "<<pose[2]<<endl;
	poseflag = true;
}

void tactswCallback(const std_msgs::Bool& msg)
{
	tactswflag = msg.data;
}

void sensorCallback(const std_msgs::Float32MultiArray& msg)
{
	// accel (z ^+ v-)
	sensorvalue[0] = msg.data[0];
	sensorvalue[1] = msg.data[2];
	sensorvalue[2] = msg.data[1];
	// gyro
	sensorvalue[3] = msg.data[3];
	sensorvalue[4] = msg.data[5];
	sensorvalue[5] = msg.data[4];
	// magne
	sensorvalue[6] = msg.data[6];
	sensorvalue[7] = msg.data[7];
	sensorvalue[8] = msg.data[8];

	sensorflag = true;
}

void motion_detect()
{
	//static double accelgain = 1.2;
	//motion[0] = accelgain * sensorvalue[0];
	//motion[1] = accelgain * sensorvalue[1];
	static double accelgain = 1.5;
	double sensor_x = sensorvalue[0];
	if (1.0 < sensor_x) sensor_x = 1.0;
	if (sensor_x <-1.0) sensor_x =-1.0;
	double sensor_y = sensorvalue[1];
	if (1.0 < sensor_y) sensor_y = 1.0;
	if (sensor_y <-1.0) sensor_y =-1.0;
	
	double move_x = accelgain * -(acos(sensor_x)-M_PI/2);
	double move_y = accelgain * -(acos(sensor_y)-M_PI/2);
	double norm = sqrt(move_x*move_x + move_y+move_y);
	if(1<norm) {
		move_x /= norm;
		move_y /= norm;
	}
	
	motion[0] = move_x;
	motion[1] = move_y;
	motion[2] = 0.0;
	
	if(1.0 < motion[0]) motion[0] = 1.0;
	else if(motion[0] < -1.0) motion[0] = -1.0;
	if(1.0 < motion[1]) motion[1] = 1.0;
	else if(motion[1] < -1.0) motion[1] = -1.0;
}

void motion_correction(double* motion, double* pose)
{
	//static tf::StampedTransform transform;
	//try{
	//	listener.lookupTransform(lookup_tf_header_, lookup_tf_child_, ros::Time(0), transform);
	//}
	//catch (tf::TransformException ex){
	//	ROS_ERROR("%s",ex.what());
	//}

	//static double roll, pitch, yaw;
	//transform.getBasis().getRPY(roll, pitch, yaw);
	//double error_x = transform.getOrigin().x();
	//double error_y = transform.getOrigin().y();
	//double error_t = yaw;

	double mvcmdvecnorm = sqrt(motion[0]*motion[0] + motion[1]*motion[1]);
	double error_x = pose[0];
	double error_y = pose[1];
	double error_t = pose[2];

	double theta = atan2(motion[1], motion[0]);
	double int_x = (tan(theta)*error_x - error_y) / (tan(theta) + 1.0/tan(theta));
	double int_y = -1.0/tan(theta) * int_x;

	double forwarderr_x = error_x - int_x; 
	double forwarderr_y = error_y - int_y; 
	double sidewayerr_x = int_x; 
	double sidewayerr_y = int_y; 

	static double forwardgain = 0.1;
	static double sidewaygain = 6.0; 
	static double rotationgain = 3.0;  
	double correction_x = -forwardgain * forwarderr_x -sidewaygain * sidewayerr_x;
	double correction_y = -forwardgain * forwarderr_y -sidewaygain * sidewayerr_y;
	double correction_t = -rotationgain * error_t;
	//cout<<"cor_x = "<<correction_x<<endl;
	//cout<<"cor_y = "<<correction_y<<endl;
	//cout<<"cor_t = "<<correction_t<<endl;
	motion[0] += correction_x * mvcmdvecnorm;
	motion[1] += correction_y * mvcmdvecnorm;
	motion[2] += correction_t;
	
	if(1.0 < motion[0]) motion[0] = 1.0;
	else if(motion[0] < -1.0) motion[0] = -1.0;
	if(1.0 < motion[1]) motion[1] = 1.0;
	else if(motion[1] < -1.0) motion[1] = -1.0;
	if(1.0 < motion[2]) motion[2] = 1.0;
	else if(motion[2] < -1.0) motion[2] = -1.0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_("~");

	int looprate_;
	bool correction_flag_;
	double emstop_time_;
	std::string flag_topic_, pose_topic_, tactsw_topic_, sensor_topic_, lookup_tf_header_, lookup_tf_child_;
	node_.param("looprate", looprate_, 30);
	node_.param("markerflag_topic_", flag_topic_, std::string("/marker_detection/MarkerFlag"));
	node_.param("markerpose_topic_", pose_topic_, std::string("/marker_detection/MarkerPoseReversed"));
	node_.param("tactsw_topic", tactsw_topic_, std::string("/MPU9250_input/tactswitch"));
	node_.param("sensor_topic", sensor_topic_, std::string("/MPU9250_input/sensordata"));
	node_.param("lookup_tf_header", lookup_tf_header_, string("/marker"));
	node_.param("lookup_tf_child", lookup_tf_child_, string("/cam"));
	node_.param("correction", correction_flag_, true);
	ros::Subscriber markerflagSub = node_.subscribe(flag_topic_, 1, markerflagCallback);
	ros::Subscriber markerposeSub = node_.subscribe(pose_topic_, 1, markerposeCallback);
	ros::Subscriber TactSwSub = node_.subscribe(tactsw_topic_, 1, tactswCallback);
	ros::Subscriber SensorSub = node_.subscribe(sensor_topic_, 1, sensorCallback);
	ros::Rate looprate(looprate_);

	omni.GpioInit();
	omni.PwmCreateSetup();
	omni.pinModeInputSetup();
	//omni.wiringPiISRSetup();
	wiringPiISR(SIG1A, INT_EDGE_BOTH, pin1A_changed);
	wiringPiISR(SIG1B, INT_EDGE_BOTH, pin1B_changed);
	wiringPiISR(SIG2A, INT_EDGE_BOTH, pin2A_changed);
	wiringPiISR(SIG2B, INT_EDGE_BOTH, pin2B_changed);
	wiringPiISR(SIG3A, INT_EDGE_BOTH, pin3A_changed);
	wiringPiISR(SIG3B, INT_EDGE_BOTH, pin3B_changed);

	while(ros::ok())
	{
		if(sensorflag) { 
			cout<<"ac_x  : "<<sensorvalue[0]<<endl;
			cout<<"ac_y  : "<<sensorvalue[1]<<endl;
			cout<<"gy_z  : "<<sensorvalue[5]<<endl;
			cout<<endl;
			
			if (tactswflag) {
				motion_detect();
				if (correction_flag_ && markerflag && poseflag) {
					motion_correction(motion, pose);
				}
				omni.movecmd_write(motion[0], motion[1], motion[2]);
			}
			else {
				omni.movecmd_write(0,0,0);
			}
			omni.output();

		}
		else {
			omni.stop();
		}
		//omni.dispstatus();
		omni.pulseReset();
		markerflag = false;
		poseflag = false;
		tactswflag = false;
		sensorflag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
