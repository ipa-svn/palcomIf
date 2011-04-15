/*
 * Palcom_teleop.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: sofie
 */
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

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

class Palcom_teleop {
public:
	static char *deviceId;
	static palcomServiceID_t sid ;

	palcomDevice_t *dev_p_;
	palcomService_t *serv_p_;
	struct pollfd pfd[3];
	struct joint_module{
		std::string key;
		std::vector<std::string> joint_names;
		std::vector<double> req_joint_pos_;
		std::vector<double> req_joint_vel_;
		std::vector<double> steps;
		ros::Publisher module_publisher_;
		ros::Publisher module_publisher_brics_;
	};

	std::map<std::string,joint_module> joint_modules_; //std::vector<std::string> module_names;

	struct base_module_struct{
		std::vector<double> req_vel_;
		std::vector<double> vel_old_; //vy_old_,(vy_old_,)vth_old_;
		std::vector<double> max_vel_; //max_vx_,(max_vy_,)max_vth_;
		std::vector<double> max_acc_; //max_ax_,(max_ay_,)max_ath_;
		std::vector<std::string> command_names_;

		ros::Publisher base_publisher_;
	} base_module_;

	bool has_base_module_;

	int lower_neck_button_,upper_neck_button_; //buttons
	int tray_button_;
	int axis_vx_,axis_vy_,axis_vth_;
	int arm_joint12_button_;
	int arm_joint34_button_;
	int arm_joint56_button_;
	int arm_joint7_button_;
	//signs
	int up_down_,left_right_;   //sign for movements of upper_neck and tray

	//common
	int deadman_button_,run_button_;
	bool joy_active_,stopped_;
	double run_factor_, run_factor_param_;


	ros::NodeHandle n_;
	ros::Subscriber joy_sub_;  //subscribe topic joy
	ros::Subscriber joint_states_sub_;  //subscribe topic joint_states

	bool got_init_values_;
	double time_for_init_;

	struct combined_joints_struct{
		std::vector<std::string> joint_names_;
		std::vector<double> joint_init_values_;
		std::vector<joint_module*> module_ref_;
	}combined_joints_;
	std::vector<std::string> joint_names_;
	std::vector<double> joint_init_values_;

	Palcom_teleop();
	void waitForParameters();
	void getConfigurationFromParameters();
	void init(char*);
	void joy_cb(const joy::Joy::ConstPtr &joy_msg);
	void joint_states_cb(const sensor_msgs::JointState::ConstPtr &joint_states_msg);
	void update();
	void update_joint_modules();
	void update_base();
	void run();
	void setInitValues();
	~Palcom_teleop();
	int exit_;
private:
	bool assign_joint_module(std::string,XmlRpc::XmlRpcValue);
	bool assign_base_module(XmlRpc::XmlRpcValue);
};

char* Palcom_teleop::deviceId = (char*)"X:SN102";
palcomServiceID_t Palcom_teleop::sid ={
		{(char*)"X:SN102", (char*)"X:SN102", (char*)0, (char*)0}, // the device id create, update (total 4 supported)
		{(char*)"1", (char*)"1", (char*)0, (char*)0} // the creation id of each device id
};

void Palcom_teleop::waitForParameters()
{
	while(!n_.hasParam("/teleop/modules"))
	{
		ROS_WARN("no modules parameter list loaded");
		sleep(1); // sleep 1 s while waiting for parameter to be loaded
	} // block until robot modules are loded

	if(!n_.hasParam("modules/base"))
	{
		//sleep(1); // sleep 1 s while waiting for parameter to be loaded
		ROS_WARN("no base module loaded");
	}

	// get the list with modules that should be loaded
	/*XmlRpc::XmlRpcValue module_list;
	n_.getParam("/robot_config/robot_modules",module_list);
	ROS_ASSERT(module_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i=0;i<module_list.size();i++)
	{
		ROS_ASSERT(module_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		std::string s((std::string)module_list[i]);
		ROS_DEBUG("searching for module = %s", s.c_str());

		// block until required module is loaded
		while(!n_.hasParam("modules/"+s))
		{
			sleep(1); // sleep 1 s while waiting for parameter to be loaded
			ROS_WARN("required module not loaded");
		}
	}
*/
	ROS_INFO("module list found");
}

// wrapper function used as call-back from control service
void palcom_ctr_cmd_handler_wrapper(const palcomIncoming_t *event, const char *command, void *userdata){
	//pal_dash->palcom_ctr_cmd_handler(event, command, userdata);
}
/**
 * Create device, service and assign commands.
 */
void Palcom_teleop::init(char* net){
	// --------create palcom components (device and service)------------
	dev_p_ = palcomInitDevice(deviceId, "palcom_teleop_node_device", net, 5000);
	serv_p_ = palcomInstallService(dev_p_, &sid, "palcom_teleop_service", "0.1", "palcom_teleop", "service for passing action commands to ros", (palcomServiceCallback_t *)&palcom_ctr_cmd_handler_wrapper, 0);

	// ---------------joystick configuration-------------------
	// common
	n_.param("run_factor",run_factor_param_,1.5);

	// assign buttons
	n_.param("deadman_button",deadman_button_,5);
	n_.param("run_button",run_button_,7);

	// assign axis
	n_.param("axis_vx",axis_vx_,1);
	n_.param("axis_vy",axis_vy_,0);
	n_.param("axis_vth",axis_vth_,2);
	n_.param("up_down",up_down_,5); //axis[5] tray--up/down; tilt--front/back, here we just name up_down
	n_.param("left_right",left_right_,4);  //axis[4] pan--left/right

	// output for debugging
	ROS_DEBUG("init::axis_vx: %d",axis_vx_);
	ROS_DEBUG("init::axis_vy: %d",axis_vy_);
	ROS_DEBUG("init::axis_vth: %d",axis_vth_);
	ROS_DEBUG("init::up_down: %d",up_down_);
	ROS_DEBUG("init::left_right: %d",left_right_);

	joy_sub_ = n_.subscribe("/joy",1,&Palcom_teleop::joy_cb,this);


	waitForParameters();
	getConfigurationFromParameters(); // assign configuration and subscribe to topics

	ROS_INFO("adding all found joint names to vector");
	// add all found joint names to joint_names_vector, which is used to pass values to the state aggregator
	for(std::map<std::string,joint_module>::iterator module_it=joint_modules_.begin();module_it!=joint_modules_.end();++module_it){
		std::vector<std::string> names = (module_it->second).joint_names;
		for(int i=0; i<names.size();i++){
			joint_names_.push_back(names[i]);
			combined_joints_.joint_names_.push_back(names[i]); // puch joit name to combined collection
			combined_joints_.joint_init_values_.push_back(0.0); // be sure that a init value is related to the joint name
			combined_joints_.module_ref_.push_back((joint_module*)(&(module_it->second))); // store a reference to the module containing the joint
		}
	}
	joint_init_values_.resize(joint_names_.size());

	palcomDeviceStatus(dev_p_, 'G');
	palcomServiceStatus(serv_p_, 'G', 0);


}
Palcom_teleop::Palcom_teleop() {
	got_init_values_ = false;
	time_for_init_ = 0.0;
	joy_active_ = false;
	run_factor_ = 1.0;
	exit_=0;
}

Palcom_teleop::~Palcom_teleop() {
	exit_=1;
	// TODO make sure robot stops (should do due to deadman button)
}

void Palcom_teleop::getConfigurationFromParameters()
{
	ROS_INFO("getting modules");
	//std::map<std::string,joint_module> joint_modules; //std::vector<std::string> module_names;
	if(n_.hasParam("modules"))
	{
		XmlRpc::XmlRpcValue modules;
		ROS_INFO("modules found ");
		n_.getParam("modules", modules);
		if(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			ROS_INFO("modules are of type struct with size %d",(int)modules.size());

			for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=modules.begin();p!=modules.end();++p)
			{
				std::string mod_name = p->first;
				ROS_INFO("module name: %s",mod_name.c_str());
				XmlRpc::XmlRpcValue mod_struct = p->second;
				if(mod_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
					ROS_WARN("invalid module, name: %s",mod_name.c_str());
				// search for joint_name parameter in current module struct to determine which type of module
				// only joint mods or wheel mods supported
				// which mens that is no joint names are found, then the module is a wheel module
				// TODO replace with build in find, but could not get it to work
				if(!assign_joint_module(mod_name, mod_struct))
				{
					// add wheel module struct
					ROS_INFO("wheel module found");
					assign_base_module(mod_struct);
				}
			}
		}
		else
		{
			ROS_WARN("modules not a struct");
		}
	}
	else
	{
		ROS_WARN("no param modules");
	}
}

/**
 * Tries to read joint module configurations from XmlRpcValue object.
 * If the module is a joint module, it contains a joint name array.
 * A typical joint module has the following configuration structure:
 * struct {
 * 	  joint_names: ['head_pan_joint','head_tilt_joint'],
 * 	  joint_step: 0.075
 * }
 * @param mod_struct configuration object struct
 * @return true if the configuration object hols a joint module config, else false
 */
bool Palcom_teleop::assign_joint_module(std::string mod_name, XmlRpc::XmlRpcValue mod_struct)
{
	if(mod_name.compare("base")==0){
		ROS_INFO("base module checked");
		return false;
	}else if(mod_name.compare("head")==0){
		ROS_INFO("head module checked");
		return true;
	}else{
		ROS_INFO("strange module checked");
	}
	// search for joint_name parameter in current module struct to determine which type of module
	// only joint mods or wheel mods supported
	// which mens that is no joint names are found, then the module is a wheel module
	// TODO replace with build in find, but could not get it to work
	bool is_joint_module = false;
	joint_module tempModule;
	for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator ps=mod_struct.begin();ps!=mod_struct.end();++ps)
	{
		std::string par_name = ps->first;
		ROS_DEBUG("par name: %s",par_name.c_str());

		if(par_name.compare("joint_names")==0)
		{
			ROS_DEBUG("joint names found");
			XmlRpc::XmlRpcValue joint_names = ps->second;

			ROS_ASSERT(joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
			ROS_DEBUG("joint_names.size: %d \n", joint_names.size());
			for(int i=0;i<joint_names.size();i++)
			{
				ROS_ASSERT(joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
				std::string s((std::string)joint_names[i]);
				ROS_DEBUG("joint_name found = %s",s.c_str());
				tempModule.joint_names.push_back(s);
			}
			// set size of other vectors according to the joint name vector
			tempModule.req_joint_pos_.resize(joint_names.size());
			tempModule.req_joint_vel_.resize(joint_names.size());

			is_joint_module = true;
			//break; // no need to continue searching if joint names are found
		}else if(par_name.compare("joint_step")==0){
			ROS_DEBUG("joint steps found");
			XmlRpc::XmlRpcValue joint_steps = ps->second;

			ROS_ASSERT(joint_steps.getType() == XmlRpc::XmlRpcValue::TypeArray);
			ROS_DEBUG("joint_steps.size: %d \n", joint_steps.size());
			for(int i=0;i<joint_steps.size();i++)
			{
				ROS_ASSERT(joint_steps[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double step((double)joint_steps[i]);
				ROS_DEBUG("joint_step found = %f",step);
				tempModule.steps.push_back(step);
			}
		}
	}
	if(is_joint_module)
	{
		// assign publisher
		tempModule.module_publisher_ = n_.advertise<trajectory_msgs::JointTrajectory>(("/"+mod_name+"_controller/command"),1);
		//tempModule.module_publisher_brics_ = n_.advertise<brics_actuator::JointVelocities>(("/"+mod_name+"_controller/command_vel"),1);
		// store joint module in collection
		ROS_DEBUG("head module stored");
		joint_modules_.insert(std::pair<std::string,joint_module>(mod_name,tempModule));
	}
	return is_joint_module;
}
/**
 * Tries to read base module configurations from XmlRpcValue object.
 * A base module is supposed to contain vectors 3 element vectors (x,y,th)
 * with max acceleration and velocity. Example:
 * struct {
 * 	   max_velocity: [0.3, 0.2, 0.3],
 * 	  max_acceleration: [0.5, 0.5, 0.7]
 * }
 * @param mod_struct configuration object struct
 * @return true no check is currently performed TODO check
 */
bool Palcom_teleop::assign_base_module(XmlRpc::XmlRpcValue mod_struct)
{
	for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator ps=mod_struct.begin();ps!=mod_struct.end();++ps)
	{
		std::string par_name = ps->first;
		ROS_DEBUG("par name: %s",par_name.c_str());

		if(par_name.compare("max_velocity")==0)
		{
			ROS_DEBUG("max vel found");
			XmlRpc::XmlRpcValue max_vel = ps->second;

			ROS_ASSERT(max_vel.getType() == XmlRpc::XmlRpcValue::TypeArray);
			if(max_vel.size()!=3 && max_vel.size()!=2){ROS_WARN("invalid base parameter size");}
			ROS_DEBUG("max_vel.size: %d \n", max_vel.size());
			for(int i=0;i<max_vel.size();i++)
			{
				ROS_ASSERT(max_vel[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double val = (double)max_vel[i];
				ROS_INFO("max vel value = %f",val);
				base_module_.max_vel_.push_back(val);
			}
		}
		else if(par_name.compare("max_acceleration")==0)
		{
			ROS_DEBUG("max acc found");
			XmlRpc::XmlRpcValue max_acc = ps->second;

			ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
			if(max_acc.size()!=3 && max_acc.size()!=2){ROS_DEBUG("invalid base parameter size");}
			ROS_DEBUG("max_acc.size: %d \n", max_acc.size());
			for(int i=0;i<max_acc.size();i++)
			{
				ROS_ASSERT(max_acc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double val = (double)max_acc[i];
				ROS_DEBUG("max acc value = %f", val);
				base_module_.max_acc_.push_back(val);
			}
		}
		else
		{
			ROS_WARN("unsupported base module parameter read");
		}
	}

	int l = base_module_.max_acc_.size();
	if(l!=base_module_.max_vel_.size()){
		ROS_FATAL("error in teleop/robot.yaml file, parameter length mismatch ");
	}
	// make all the vectors the same length
	// the vector size is not completely safe since only warning is
	// issued if max value arrays has the wrong length
	base_module_.req_vel_.resize(l);
	base_module_.vel_old_.resize(l);


	ROS_INFO("assigning palcom commands");
	// -------------assign palcom commands to service---------
	palcomDescribeCommand(serv_p_, "command_x_velocity", PALCOM_OUT, 1, "velocity", "text/int16");
	ROS_INFO("assigning one palcom command");
	base_module_.command_names_.push_back("command_x_velocity");
	ROS_INFO("pushed one palcom command");
	if(l==3){
		palcomDescribeCommand(serv_p_, "command_y_velocity", PALCOM_OUT, 1, "velocity", "text/int16");
		base_module_.command_names_.push_back("command_y_velocity");
	}
	palcomDescribeCommand(serv_p_, "command_rot_velocity", PALCOM_OUT, 1, "velocity", "text/int16");
	base_module_.command_names_.push_back("command_rot_velocity");

	ROS_INFO("palcom ok for base");

	base_module_.base_publisher_ = n_.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	ROS_INFO("base module stored");
	has_base_module_ = true;
	return true;
}
/*!
 * \brief Sets initial values for target velocities.
 */
void Palcom_teleop::setInitValues()
{
	ROS_DEBUG("setting initial values");

	// loop trough all the joints in the combined collection
	for(int i=0; i<combined_joints_.joint_init_values_.size();i++){
		//loop trough all the joints in module containing joint settings,
		//and try to find the one with a name matching the currently browsed joint
		//in the combined collection
		for(int j=0; j<combined_joints_.module_ref_[i]->joint_names.size();j++){
			// if the matching joint is found, assign value to pos command and stop looking for this name
			if(combined_joints_.module_ref_[i]->joint_names[j].compare(combined_joints_.joint_names_[i])==0){
				combined_joints_.module_ref_[i]->req_joint_pos_[j] = combined_joints_.joint_init_values_[i];
				combined_joints_.module_ref_[i]->req_joint_vel_[j] = 0.0; // initalize velocity cmd to 0
				break; // node found, break the search
			}
		}
	}

	// base init values (velocities) are already set to 0 by default

	got_init_values_ = true;
}
/*!
 * \brief Executes the callback from the joystick topic.
 *
 * Gets the configuration
 *
 * \param joy_msg Joy
 */
void Palcom_teleop::joy_cb(const joy::Joy::ConstPtr &joy_msg)
{
	// deadman button to activate joystick
	if(deadman_button_>=0 && deadman_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[deadman_button_]==1)
	{
		if (!joy_active_)
		{
			ROS_INFO("joystick is active");
			joy_active_ = true;
			got_init_values_ = false;
		}
	}
	else
	{
		ROS_DEBUG("joystick is not active");
		joy_active_ = false;
		return;
	}

	// run button
	if(run_button_>=0 && run_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[run_button_]==1)
	{
		run_factor_ = run_factor_param_;
	}
	else //button release
	{
		run_factor_ = 1.0;
	}

	// TODO add map for buttons

	// head
	if(joint_modules_.find("head")!=joint_modules_.end())
	{
		if(upper_neck_button_>=0 &&
				upper_neck_button_<(int)joy_msg->buttons.size() &&
				joy_msg->buttons[upper_neck_button_]==1)
		{
			//pan (turn)
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["head"].req_joint_vel_[0] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[0]*run_factor_;//upper_pan_step_*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["head"].req_joint_vel_[0] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[0]*run_factor_;//upper_pan_step_*run_factor_;
			else
				joint_modules_["head"].req_joint_vel_[0]= 0.0;
			ROS_DEBUG("cb::upper neck pan velocity: %f",joint_modules_["head"].req_joint_vel_[0]);

			//tilt (nod)
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["head"].req_joint_vel_[1] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[1]*run_factor_;//upper_tilt_step_*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["head"].req_joint_vel_[1] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[1]*run_factor_;//upper_tilt_step_*run_factor_;
			else
				joint_modules_["head"].req_joint_vel_[1] = 0.0;
			ROS_DEBUG("cb::upper neck tilt velocity: %f",joint_modules_["head"].req_joint_vel_[1]);

		}
		else
		{
			joint_modules_["head"].req_joint_vel_[0] = 0.0;
			joint_modules_["head"].req_joint_vel_[1] = 0.0;
		}
	}

	//================base================
	if(has_base_module_ && base_module_.req_vel_.size()>=2)
	{
		int i = 0; // to get it right both for length 2 and 3
		if(axis_vx_>=0 && axis_vx_<(int)joy_msg->get_axes_size())
			base_module_.req_vel_[i] = joy_msg->axes[axis_vx_]*base_module_.max_vel_[i]*run_factor_;
		else
			base_module_.req_vel_[i] = 0.0;
		i++;

		if(base_module_.req_vel_.size()==3){
			if(axis_vy_>=0 && axis_vy_<(int)joy_msg->get_axes_size())
				base_module_.req_vel_[i] = joy_msg->axes[axis_vy_]*base_module_.max_vel_[i]*run_factor_;//req_vy_ = joy_msg->axes[axis_vy_]*max_vy_*run_factor_;
			else
				base_module_.req_vel_[i] = 0.0; //req_vy_ = 0.0;
			i++;
		}
		//uggly fix to make it work also with new joystick
		if((int)joy_msg->get_axes_size()>6) // new joystick
		{
			// with the new joystick, the base turn axes has index 4 instead of 3
			if(axis_vth_>=0 && axis_vth_<(int)joy_msg->get_axes_size())
				base_module_.req_vel_[i] = joy_msg->axes[3]*base_module_.max_vel_[i]*run_factor_;//req_vth_ = joy_msg->axes[axis_vth_]*max_vth_*run_factor_;
			else
				base_module_.req_vel_[i] = 0.0; //req_vth_ = 0.0;
		}
		else // old joystick
		{
			if(axis_vth_>=0 && axis_vth_<(int)joy_msg->get_axes_size())
				base_module_.req_vel_[i] = joy_msg->axes[axis_vth_]*base_module_.max_vel_[i]*run_factor_;//req_vth_ = joy_msg->axes[axis_vth_]*max_vth_*run_factor_;
			else
				base_module_.req_vel_[i] = 0.0; //req_vth_ = 0.0;
		}
	}

}//joy_cb


/*!
 * \brief Main routine for updating all components.
 */
void Palcom_teleop::update()
{

	if (!joy_active_)
	{
		ROS_DEBUG("joy not active");
		if (!stopped_)
		{
			// stop components: send zero for one time
			for(std::map<std::string,joint_module>::iterator module_it=joint_modules_.begin();module_it!=joint_modules_.end();++module_it)
			{
				ROS_DEBUG("stopping joint modules");
				for(int i=0; i<module_it->second.req_joint_vel_.size();i++)
				{
					module_it->second.req_joint_vel_[i] = 0.0;
				}
			}

			if(has_base_module_)
			{
				ROS_DEBUG("stopping base module");
				for(int i=0; i<3; i++){
					base_module_.req_vel_[i]=0;
					base_module_.vel_old_[i]=0;
				}
			}

			//update_joint_modules();
			update_base();
			stopped_ = true;
			ROS_INFO("stopped all components");
		}
		return;
	}

	ROS_DEBUG("active");
	// set initial values
	if(!got_init_values_)
	{
		if (time_for_init_ < 1.0) // wait for 1 sec, then set init values to 0.0
		{
			ROS_DEBUG("still waiting for initial values, time_for_init_ = %f",time_for_init_);
			time_for_init_ = time_for_init_ + 1.0/PUBLISH_FREQ;
			return;
		}
		else
		{
			ROS_WARN("Timeout waiting for /joint_states message. Setting all init values to 0.0");
			setInitValues();
		}
	}

	//update_joint_modules();
	update_base();
	stopped_ = false;
}

void Palcom_teleop::update_joint_modules()
{
	double dt = 1.0/double(PUBLISH_FREQ);
	double horizon = 3.0*dt;

	joint_module* jointModule;
	for(std::map<std::string,joint_module>::iterator it = joint_modules_.begin();it!=joint_modules_.end();++it)
	{
		jointModule = (joint_module*)(&(it->second));

		trajectory_msgs::JointTrajectory traj;
		traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
		traj.points.resize(1);
		//brics_actuator::JointVelocities cmd_vel;
		//brics_actuator::JointValue joint_vel;
		//joint_vel.timeStamp = traj.header.stamp;
		//joint_vel.unit = "rad";
		for( int i = 0; i<jointModule->joint_names.size();i++)
		{
			// as trajectory message
			traj.joint_names.push_back(jointModule->joint_names[i]);
			traj.points[0].positions.push_back(jointModule->req_joint_pos_[i] + jointModule->req_joint_vel_[i]*horizon);
			traj.points[0].velocities.push_back(jointModule->req_joint_vel_[i]);  //lower_neck_pan
			// as brics message
			//joint_vel.value = jointModule->req_joint_vel_[i];
			//joint_vel.joint_uri = jointModule->joint_names[i];
			//cmd_vel.velocities.push_back(joint_vel);
			// update current positions
			jointModule->req_joint_pos_[i] += jointModule->req_joint_vel_[i]*horizon;
		}

		traj.points[0].time_from_start = ros::Duration(horizon);

		jointModule->module_publisher_.publish(traj); // TODO, change
		//jointModule->module_publisher_brics_.publish(cmd_vel);
	}
}

/*!
 * \brief Routine for updating the base commands.
 */
void Palcom_teleop::update_base()
{
	if(!has_base_module_)
		return;
	double dt = 1.0/double(PUBLISH_FREQ);


	double v[] = {0.0,0.0,0.0};

	for( int i =0; i<base_module_.command_names_.size(); i++)
	{
		// filter v with ramp
		if ((base_module_.req_vel_[i]-base_module_.vel_old_[i])/dt > base_module_.max_acc_[i])
		{
			v[i] = base_module_.vel_old_[i] + base_module_.max_acc_[i]*dt;
		}
		else if((base_module_.req_vel_[i]-base_module_.vel_old_[i])/dt < -base_module_.max_acc_[i])
		{
			v[i] = base_module_.vel_old_[i] - base_module_.max_acc_[i]*dt;
		}
		else
		{
			v[i] = base_module_.req_vel_[i];
		}
		base_module_.vel_old_[i] = v[i];
		int vel=0;
		ROS_INFO("about to check command name");
		char* s = (char*)base_module_.command_names_[i].c_str();

		ROS_INFO("prosessing command name %s ", s);
		if(base_module_.command_names_[i].compare("command_rot_velocity")==0){//size()-1)
			vel=(int)(v[i]*180/6.28); // convert to degrees
			ROS_INFO("convertion ang vel to deg vel %d from %f", vel, v[i]);
		}else
			vel = (int)(v[i]*1000); // convert to mm/s
		ROS_INFO("vel %d ", vel);
		// convert to string rep
		char vels[6];
		snprintf(vels, sizeof(vels), "%d", vel);

		// assign velocity data to palcom command
		//std::string s = base_module_.command_names_[i];
		//char* sz = new char[base_module_.command_names_[i].length() + 1];
		//strcpy(sz, base_module_.command_names_[i].c_str());

		palcomOutgoing_t* message = palcomNewEvent((char*)base_module_.command_names_[i].c_str());
		palcomSetParam(message, "velocity", "text/int16", vels, strlen(vels));
		palcomGenerate(message, serv_p_);
	}

	if(base_module_.max_acc_.size()==3){

		geometry_msgs::Twist cmd;
		cmd.linear.x = v[0]; //vx;
		cmd.linear.y = v[1]; //vy;
		cmd.angular.z = v[2]; //vth;

		base_module_.base_publisher_.publish(cmd);
	}else if(base_module_.max_acc_.size()==2){
		geometry_msgs::Twist cmd;
		cmd.linear.x = v[0]; //vx;
		cmd.linear.y = 0.0; //vy;
		cmd.angular.z = v[1]; //vth;

		base_module_.base_publisher_.publish(cmd);
	}
}
pthread_t pal_to_ros_thread;
Palcom_teleop* teleop_p;

void terminate(int i){

	printf("terminating\n");
	teleop_p->~Palcom_teleop();
	//pthread_join(pal_to_ros_thread,0);
	exit(0);
}

void Palcom_teleop::run(){
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
	Palcom_teleop teleop;
	ROS_INFO("teleop created");
	teleop_p = (Palcom_teleop*)&teleop;
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
		teleop.update();
		loop_rate.sleep();
	}

	exit(0);
	return(0);
}
