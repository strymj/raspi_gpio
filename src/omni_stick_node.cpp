#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
#include <string>
#include <tf/transform_datatypes.h>   // quaternion -> rpy

//#include <tf/transform_listener.h>
//tf::TransformListener listener;

// tact switch setup
#define TACTSW 5

using namespace std;

bool markerflag = false;
bool poseflag = false;
bool sensorflag = false;


//void correctionCallback(const std_msgs::Float32MultiArray& msg)
//{
//	correctioncmd[0] = msg.data[0];
//	correctioncmd[1] = msg.data[1];
//	correctioncmd[2] = msg.data[2];
//	correctioncmdflag = true;
//}

void markerflagCallback(const std_msgs::Bool& msg)
{
	if(msg.data) markerflag = true;
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
	poseflag = true;
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

void movecmd_detect(double* movecmd, double* sensorvalue)
{
	//static double accelgain = 0.5;
	//static double gyrogain = 0.004;
	static double accelgain = 1.2;
	//movecmd[0] += -accelgain * sensorvalue[0];
	//movecmd[1] += -accelgain * sensorvalue[1];
	//movecmd[2] +=  -gyrogain * sensorvalue[5];
	movecmd[0] = accelgain * sensorvalue[0];
	movecmd[1] = accelgain * sensorvalue[1];
	movecmd[2] = 0;
}

void movecmd_correction(double* movecmd, double* pose)
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

	double error_x = pose[0];
	double error_y = pose[1];
	double error_t = pose[2];
	cout<<"error_t : "<<error_t<<endl;

	double theta = atan2(movecmd[1], movecmd[0]);
	double int_x = (tan(theta)*error_x - error_y) / (tan(theta) + 1.0/tan(theta));
	double int_y = -1.0/tan(theta) * int_x;

	double forwarderr_x = error_x - int_x; 
	double forwarderr_y = error_y - int_y; 
	double sidewayerr_x = int_x; 
	double sidewayerr_y = int_y; 

	static double forwardgain = 0.08;
	static double sidewaygain = 0.8; 
	static double rotationgain = 1.0;  
	movecmd[0] += -forwardgain * forwarderr_x -sidewaygain * sidewayerr_x;
	movecmd[1] += -forwardgain * forwarderr_y -sidewaygain * sidewayerr_y;
	movecmd[2] = -rotationgain * error_t;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_("~");

	int looprate_;
	double emstop_time_;
	std::string flag_topic_, pose_topic_, sensor_topic_, lookup_tf_header_, lookup_tf_child_;
	node_.param("looprate", looprate_, 30);
	node_.param("markerflag_topic_", flag_topic_, std::string("/marker_detection/MarkerFlag"));
	node_.param("markerpose_topic_", pose_topic_, std::string("/marker_detection/MarkerPoseReversed"));
	node_.param("sensor_topic", sensor_topic_, std::string("/MPU9250_input/sensordata"));
	node_.param("lookup_tf_header", lookup_tf_header_, string("/marker"));
	node_.param("lookup_tf_child", lookup_tf_child_, string("/cam"));
	ros::Subscriber markerflagSub = node_.subscribe(flag_topic_, 1, markerflagCallback);
	ros::Subscriber markerposeSub = node_.subscribe(pose_topic_, 1, markerposeCallback);
	ros::Subscriber SensorSub = node_.subscribe(sensor_topic_, 1, sensorCallback);
	ros::Rate looprate(looprate_);

	GpioInit();
	PwmCreateSetup();
	pinModeInputSetup();
	pinMode(TACTSW, INPUT);
	wiringPiISRSetup();

	while(ros::ok())
	{
		if(sensorflag) { 
			cout<<"ac_x  : "<<sensorvalue[0]<<endl;
			cout<<"ac_y  : "<<sensorvalue[1]<<endl;
			cout<<"gy_z  : "<<sensorvalue[5]<<endl;
			if (digitalRead(TACTSW)) {
				movecmd_detect(movecmd, sensorvalue);
				if (markerflag && poseflag) {
					movecmd_correction(movecmd, pose);
				}
			}
			else {
				movecmd[0] = 0;
				movecmd[1] = 0;
				movecmd[2] = 0;
			}

			calc_targetpulse(targetpulse, movecmd, ratio);
			calc_motorout(motorout, pulse, targetpulse, gain);
		}
		else {
			stop(motorout);
		}
		PWMwrite(motorout);
		dispstatus(movecmd, pulse, targetpulse, motorout);
		pulseReset(pulse);
		markerflag = false;
		poseflag = false;
		sensorflag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
