/*
 * MeasurementBridge.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: sofie
 */
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>

#include <sys/time.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <palcom.h>
#ifdef __cplusplus
}
#endif

#include <pthread.h>
#include <signal.h>

const int PUBLISH_FREQ = 20.0;

class MeasurementBridge {
public:
	static char *deviceId;
	static palcomServiceID_t sid ;

	palcomDevice_t *dev_p_;
	palcomService_t *serv_p_;
	struct pollfd pfd[3];

	ros::NodeHandle n_;
	ros::Subscriber odometry_sub_; // subscribe to base odometry

	double time_for_init_;

	MeasurementBridge();
	void init(char*);
	void run();
	void base_measurement_cb(const nav_msgs::Odometry::ConstPtr &odom_msg);
	void setInitValues();
	~MeasurementBridge();
	int exit_;
};

char* MeasurementBridge::deviceId = (char*)"X:SN102";
palcomServiceID_t MeasurementBridge::sid ={
		{(char*)"X:SN102", (char*)"X:SN102", (char*)0, (char*)0}, // the device id create, update (total 4 supported)
		{(char*)"1", (char*)"1", (char*)0, (char*)0} // the creation id of each device id
};

// wrapper function used as call-back from control service
void palcom_ctr_cmd_handler_wrapper(const palcomIncoming_t *event, const char *command, void *userdata){
	//pal_dash->palcom_ctr_cmd_handler(event, command, userdata);
}
/**
 * Create device, service and assign commands.
 */
void MeasurementBridge::init(char* net){
	// --------create palcom components (device and service)------------
	dev_p_ = palcomInitDevice(deviceId, "MeasurementBridge_node_device", net, 5000);
	serv_p_ = palcomInstallService(dev_p_, &sid, "MeasurementBridge_service", "0.1", "MeasurementBridge", "service for passing action commands to ros", (palcomServiceCallback_t *)&palcom_ctr_cmd_handler_wrapper, 0);

	odometry_sub_ = n_.subscribe("/base_controller/odometry", 1, &MeasurementBridge::base_measurement_cb,this);

	ROS_INFO("assigning palcom commands");
	// -------------assign palcom commands to service---------
	palcomDescribeCommand(serv_p_, "velocity", PALCOM_OUT, 3, "x_velocity", "text/int",
			"y_velocity", "text/int", "rot_velocity", "text/int");

	ROS_INFO("palcom ok for base");


	palcomDeviceStatus(dev_p_, 'G');
	palcomServiceStatus(serv_p_, 'G', 0);
}

MeasurementBridge::MeasurementBridge() {
	time_for_init_ = 0.0;
	exit_=0;
}

MeasurementBridge::~MeasurementBridge() {
	exit_=1;
	// TODO make sure robot stops (should do due to deadman button)
}

/*!
 * \brief Executes the callback for robot base odometry measurements.
 *
 * Gets the robot platform measurement
 *
 * \param nav_msg Odometry
 */
void MeasurementBridge::base_measurement_cb(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
	double x,y, rot, vx, vy, vrot;

	// read measurement values
	// in SI units
	x = odom_msg->pose.pose.position.x;
	y = odom_msg->pose.pose.position.y;
	rot = odom_msg->pose.pose.orientation.z;

	vx = odom_msg->twist.twist.linear.x;
	vy = odom_msg->twist.twist.linear.y;
	vrot = odom_msg->twist.twist.angular.z;

	// send to palcom
	// convert to string rep TODO not completely safe with size conversion.
	char vel_x[6];
	snprintf(vel_x, sizeof(vel_x), "%d", (int)(vx*1000));
	char vel_y[6];
	snprintf(vel_y, sizeof(vel_y), "%d", (int)(vy*1000));
	char vel_rot[6];
	snprintf(vel_rot, sizeof(vel_rot), "%d", (int)(vrot*180/3.14));

	// assign velocity data to palcom command
	palcomOutgoing_t* message = palcomNewEvent("velocity");
	palcomSetParam(message, "x_velocity", "text/int", vel_x, strlen(vel_x));
	palcomSetParam(message, "y_velocity", "text/int", vel_y, strlen(vel_y));
	palcomSetParam(message, "rot_velocity", "text/int", vel_rot, strlen(vel_rot));
	palcomGenerate(message, serv_p_);
}


pthread_t pal_to_ros_thread;
MeasurementBridge* teleop_p;

void terminate(int i){

	printf("terminating\n");
	teleop_p->~MeasurementBridge();
	//pthread_join(pal_to_ros_thread,0);
	exit(0);
}

void MeasurementBridge::run(){
	void (*prev_fn)(int);
	prev_fn = signal(SIGINT,terminate);
	if(prev_fn==SIG_ERR){printf("signal assignment error");}
	struct pollfd pfd[3];
	int timer;
	struct itimerspec its = {{1, 0}, {1, 0}};
	char dummy[8];
	timer = timerfd_create(CLOCK_MONOTONIC, 0);
	if(timer < 0) err(1, "timerfd_create");
	if(timerfd_settime(timer, 0, &its, 0)) err(1, "timerfd_settime");

	while(exit_==0) {
		///printf("running pthread\n");
		pfd[2].fd = timer;
		pfd[2].events = POLLIN;
		// prepare poss sets the fd pointers to bcsocket and uc socket and gets timout time
		if(poll(pfd, 3, palcomPreparePoll(dev_p_, pfd)) < 0) err(1, "poll");
		if(pfd[2].revents & POLLIN) {
			read(timer, dummy, 8);
			//tick(serv_p_);
		}
		palcomHandlePoll(dev_p_, pfd);
	}
	printf("exiting pthread\n");
}


// wrapper function used as call-back in pal_to_ros_thread
void pal_to_ros_thread_run(void*obj){
	teleop_p->run();
}

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc,char **argv)
{

	ros::init(argc,argv,"teleop");
	ROS_INFO("ros init done");
	MeasurementBridge teleop;
	ROS_INFO("teleop created");
	teleop_p = (MeasurementBridge*)&teleop;
	void (*prev_fn)(int);
	ROS_INFO("about to assign sign interrupt function");
	prev_fn = signal(SIGINT,terminate);
	if(prev_fn==SIG_ERR){printf("signal assignment error");}
	/*for(int i =0; i<argc; i++){
	ROS_INFO("arg %s", argv[i]);
	}
	ROS_INFO("rpinted args");
	if(argc != 2) errx(1, "Usage: %s NetworkInterface", argv[0]);
	ROS_INFO("args %d", argc);*/
	ROS_INFO("assigned sign interrupt function");
	teleop.init("eth0"); //argv[1]);

	// create palcom tick thread, needed to make palcom units apear on the network.
	int i = pthread_create(&pal_to_ros_thread, 0, (void*(*)(void*))&pal_to_ros_thread_run, 0);
	if(i<0)
		ROS_WARN("palcom to ros thread could not be started");
	ROS_WARN("palcom to ros thread started");
	ros::Rate loop_rate(PUBLISH_FREQ); //Hz
	while(teleop.n_.ok())
	{
		ros::spinOnce();
		//teleop.update();
		loop_rate.sleep();
	}

	exit(0);
	return(0);
}
