
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sys/time.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <palcom.h>
//#include <signal.h>
#ifdef __cplusplus
}
#endif

#include <pthread.h>
#include <signal.h>

#include <ros/time.h>
#include <ros/rate.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void palcom_ctr_cmd_handler_wrapper(const palcomIncoming_t *, const char *, void *);
void pal_to_ros_thread_run(void*);
void terminate(int);
/**
 * This class communicates with other nodes via the action lib.
 *
 */
class DashboardPalcom{
public:
	static char *deviceId;
	static palcomServiceID_t sid ;

	palcomDevice_t *dev_p_;
	palcomService_t *serv_p_;
	struct pollfd pfd[3];

	MoveBaseClient *ac_;

	DashboardPalcom();
	void init(char*);
	void update();
	void run();
	void palcom_ctr_cmd_handler(const palcomIncoming_t *, const char *, void *);
	~DashboardPalcom();
	int exit_;
private:
	pthread_mutex_t goal_state_lock_;
	pthread_mutex_t init_state_lock_;
	int goal_reached_;
	int init_;
};

char* DashboardPalcom::deviceId = (char*)"X:SN102";
palcomServiceID_t DashboardPalcom::sid ={
		{(char*)"X:SN102", (char*)"X:SN102", (char*)0, (char*)0}, // the device id create, update (total 4 supported)
		{(char*)"1", (char*)"1", (char*)0, (char*)0} // the creation id of each device id
};

DashboardPalcom::DashboardPalcom(){
	//tell the action client that we want to spin a thread by default
	ac_ = new MoveBaseClient("move_base", true);
	init_=0;
	exit_=0;
}

DashboardPalcom::~DashboardPalcom(){
	exit_=1;

	pthread_mutex_lock(&goal_state_lock_);
	if(goal_reached_==0)
		ac_->stopTrackingGoal();
	pthread_mutex_lock(&goal_state_lock_);
	ac_->~SimpleActionClient();
	pthread_mutex_destroy(&goal_state_lock_);
	pthread_mutex_destroy(&init_state_lock_);
}

void DashboardPalcom::init(char* net){

	dev_p_ = palcomInitDevice(deviceId, "DESIRE dashboard node Device", "0.1", net, 5000);

	serv_p_ = palcomInstallService(dev_p_, &sid, "dashboard command service", "0.1", "test_proto", "service for passing action commands to ros", (palcomServiceCallback_t *)&palcom_ctr_cmd_handler_wrapper, 0);
	// -------------assign commands to service---------
	// out command
	palcomDescribeCommand(serv_p_, "reached target signal", PALCOM_OUT, 0);
	// in command
	palcomDescribeCommand(serv_p_, "target pos", PALCOM_IN, 3, "x", "text/int", "y", "text/int", "th", "text/int");

	palcomDeviceStatus(dev_p_, 'G');
	palcomServiceStatus(serv_p_, 'G', 0);

	//wait for the action server to come up
	while(!ac_->waitForServer(ros::Duration(5.0))){
		if(exit_==1){return;}
		ROS_INFO("Testing-------Waiting for the move_base action server to come up");
	}
	pthread_mutex_lock(&init_state_lock_);
	init_=1;
	pthread_mutex_unlock(&init_state_lock_);
	pthread_mutex_lock(&goal_state_lock_);
	goal_reached_=1;
	pthread_mutex_unlock(&goal_state_lock_);

}
void DashboardPalcom::palcom_ctr_cmd_handler(const palcomIncoming_t *event, const char *command, void *userdata){
	printf("repl\n");
	double x,y,th;
	if(!strcmp(command, "target pos")) {
		x = strtod(palcomGetParam(event, "x", 0, 0),0);
		y = strtod(palcomGetParam(event, "y", 0, 0),0);
		th = strtod(palcomGetParam(event, "th", 0, 0),0);

		printf("target position received: %f, %f, %f\n", x,y,th);

		// check if ros side is setup
		pthread_mutex_lock(&init_state_lock_);
		if(init_==0){
			// if setup is not finished, return without passing on command
			pthread_mutex_unlock(&init_state_lock_);
			ROS_WARN("not init\n");
			return;
		}
		pthread_mutex_unlock(&init_state_lock_);

		// pass on command to ros if the last goal is reached,
		// in case of that the robot is already working on last goal,
		// do nothing //TODO cancel old goal
		pthread_mutex_lock(&goal_state_lock_);
		if(goal_reached_){

			ROS_INFO("sending new goal");
			move_base_msgs::MoveBaseGoal goal;

			//we'll send a goal to the robot to move 1 meter forward
			goal.target_pose.header.frame_id = "/map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = x;
			goal.target_pose.pose.position.y = y;
			goal.target_pose.pose.orientation.w = th;

			ROS_INFO("Sending goal");
			ac_->sendGoal(goal);
			goal_reached_=0;

		}else{
			ROS_INFO("working on old goal\n");
		}
		pthread_mutex_unlock(&goal_state_lock_);
	}
}

void DashboardPalcom::run(){
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
			int r = read(timer, dummy, 8);
			if(r !=8){
				ROS_INFO("failed to read 8 bytes from palcom");
			}
		}
		palcomHandlePoll(dev_p_, pfd);
	}
	printf("exiting pthread\n");
}

void DashboardPalcom::update(){
	// check if ros side is setup
	pthread_mutex_lock(&init_state_lock_);
	if(init_==0){
		// if setup is not finished, return without passing on command
		pthread_mutex_unlock(&init_state_lock_);
		return;
	}
	pthread_mutex_unlock(&init_state_lock_);

	pthread_mutex_lock(&goal_state_lock_);
	if(goal_reached_==1){
		return;
	}
	pthread_mutex_unlock(&goal_state_lock_);
	ac_->waitForResult();

	if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Hooray, the base moved 1 meter forward");
		pthread_mutex_lock(&goal_state_lock_);
		goal_reached_=1;
		pthread_mutex_unlock(&goal_state_lock_);
		palcomOutgoing_t *message = palcomNewEvent("reached target signal");
		palcomGenerate(message, serv_p_);
	}
}

DashboardPalcom* pal_dash;
pthread_t pal_to_ros_thread;

// wrapper function used as call-back in pal_to_ros_thread
void pal_to_ros_thread_run(void*obj){
	pal_dash->run();
}

// wrapper function used as call-back from control service
void palcom_ctr_cmd_handler_wrapper(const palcomIncoming_t *event, const char *command, void *userdata){
	pal_dash->palcom_ctr_cmd_handler(event, command, userdata);
}

void terminate(int i){

	printf("terminating\n");
	pal_dash->~DashboardPalcom();
	//pthread_join(pal_to_ros_thread,0);
	exit(0);
}

int main(int argc,char **argv){

	ros::init(argc,argv,"palcom_dashboard");
	void (*prev_fn)(int);
	//prev_fn = signal(SIGTERM,terminate);
	//if(prev_fn==SIG_ERR){printf("signal assignment error");}
	prev_fn = signal(SIGINT,terminate);
	if(prev_fn==SIG_ERR){printf("signal assignment error");}
	//prev_fn = signal(SIGHUP,terminate);
	//if(prev_fn==SIG_ERR){printf("signal assignment error");}

	printf("signal assignment done\n");

	DashboardPalcom pal_dashboard;
	pal_dash = (DashboardPalcom*)&pal_dashboard;
	if(argc != 2) errx(1, "Usage: %s NetworkInterface", argv[0]);
	pal_dash->init(argv[1]);

	if(pal_dash->exit_==0){
		printf("init done\n");
		int i = pthread_create(&pal_to_ros_thread, 0, (void*(*)(void*))&pal_to_ros_thread_run, 0);

		if(i<0)
			ROS_WARN("palcom to ros thread could not be started");

		//pal_dash->run();
		while(pal_dash->exit_==0){
			pal_dash->update();
		}
	}
	printf("exiting\n");


	exit(0);
	return 0;
}
