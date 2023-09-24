#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// use pragma diagnostic to hide warnings related to unused parameters in included files
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#pragma GCC diagnostic pop

#include <planner_interfacing/SetPrecision.h>

//////////////////////////////////////////////////////////////////////////////

class DWAPlanner
{
	public:
		DWAPlanner(tf2_ros::Buffer *tfBuffer, costmap_2d::Costmap2DROS *costmap);
		void loop();

		// callbacks
		void current_waypoint_cb(const geometry_msgs::Point &waypoint);
		void pose_cb(const geometry_msgs::Pose &pose);
		bool enabled_cb(std_srvs::SetBool::Request &bool_req, std_srvs::SetBool::Response &bool_resp);
		bool precision_cb(planner_interfacing::SetPrecision::Request &prec_req, planner_interfacing::SetPrecision::Response &prec_resp);

	private:
		// memory
		geometry_msgs::Twist cmd_vel;
		geometry_msgs::Point current_position;
		double current_yaw;
		geometry_msgs::Point global_wp;
		std::vector<geometry_msgs::PoseStamped> path;
		ros::Time start_move_time;
		dwa_local_planner::DWAPlannerROS planner;

		// ROS things
		ros::NodeHandle nh;
		ros::Subscriber current_waypoint_sub;
		ros::Subscriber pose_sub;
		ros::Publisher cmd_vel_pub;
		ros::Publisher current_status_pub;
		ros::ServiceServer enabled_service;
		ros::ServiceServer precision_service;

		// static params
		int local_wp_dist;
		int update_freq;
		std::string robot_frame;

		// dynamic params
		bool enabled;
		double has_location;
		double has_waypoint;
		double stuck;
		double precision;

		// local functions
		void publish_status();
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dwa_path_planner");

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	costmap_2d::Costmap2DROS costmap("costmap", tfBuffer);
	DWAPlanner dp(&tfBuffer, &costmap);
	dp.loop();
}

////// constructor ///////////////////////////////////////////////////////////

DWAPlanner::DWAPlanner(tf2_ros::Buffer *tfBuffer, costmap_2d::Costmap2DROS *costmap)
{
	////// initialize DWA ////////////////////////////////////////////////////

	this->planner.initialize("dwa_node", tfBuffer, costmap);

	////// local variables ///////////////////////////////////////////////////

	this->enabled = false;
	this->has_location = false;
	this->has_waypoint = false;
	this->stuck = false;

	ros::param::get("~default_precision", this->precision);
	ros::param::get("~update_frequency", this->update_freq);
	ros::param::get("~local_waypoint_distance", this->local_wp_dist);
	ros::param::get("~robot_frame", this->robot_frame);

	////// connect to ROS ////////////////////////////////////////////////////

	std::string current_waypoint_topic; ros::param::get("~current_waypoint_topic", current_waypoint_topic);
	std::string local_position_topic; ros::param::get("~local_position_topic", local_position_topic);
	std::string cmd_vel_topic; ros::param::get("~cmd_vel_topic", cmd_vel_topic);
	std::string current_status_topic; ros::param::get("~current_status_topic", current_status_topic);
	std::string enabled_service_name; ros::param::get("~enabled_service", enabled_service_name);
	std::string precision_service_name; ros::param::get("~precision_service", precision_service_name);

	this->current_waypoint_sub = this->nh.subscribe(current_waypoint_topic, 1, &DWAPlanner::current_waypoint_cb, this);
	this->pose_sub = this->nh.subscribe(local_position_topic, 1, &DWAPlanner::pose_cb, this);
	this->cmd_vel_pub = this->nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
	this->current_status_pub = this->nh.advertise<std_msgs::String>(current_status_topic, 1);
	this->enabled_service = this->nh.advertiseService(enabled_service_name, &DWAPlanner::enabled_cb, this);
	this->precision_service = this->nh.advertiseService(precision_service_name, &DWAPlanner::precision_cb, this);
}

////// callbacks /////////////////////////////////////////////////////////////

void DWAPlanner::current_waypoint_cb(const geometry_msgs::Point &waypoint)
{
	this->global_wp = waypoint;
	this->has_waypoint = true;
	this->publish_status();
}

void DWAPlanner::pose_cb(const geometry_msgs::Pose &pose)
{
	tf2::Quaternion q(
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z,
		pose.orientation.w
	);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	this->current_position = pose.position;
	this->current_yaw = yaw;
	this->has_location = true;

	// TODO: check if we are stuck (PID Path Planner code below)
	// # add position to recent position window
	// self.recent_positions_window.append(StampedPose(pose_msg, rospy.Time.now()))

	// # remove positions too old to be in recent position window
	// updated_window = []
	// for stamped_pose in self.recent_positions_window:
	// 	if stamped_pose.time + self.not_moving_delay > rospy.Time.now():
	// 		updated_window.append(stamped_pose)

	// self.recent_positions_window = updated_window

	// # check if we are not moving by comparing current position to average position of position window
	// positions_in_window = [stamped_pose.pose.position for stamped_pose in updated_window]
	// x_mean = mean([position.x for position in positions_in_window] or [0])
	// y_mean = mean([position.y for position in positions_in_window] or [0])
	// z_mean = mean([position.z for position in positions_in_window] or [0])
	// average_position = Point(x_mean, y_mean, z_mean)
	// self.not_moving = dist2d(average_position, pose_msg.position) * 2 < self.not_moving_threshold
}

bool DWAPlanner::enabled_cb(std_srvs::SetBool::Request &bool_req, std_srvs::SetBool::Response &bool_resp)
{
	bool enabled = bool_req.data;
	bool being_disabled = this->enabled && !enabled;
	bool being_enabled = !this->enabled && enabled;
	this->enabled = enabled;
	this->publish_status();

	if (being_disabled)
	{
		this->cmd_vel.linear.x = 0;
		this->cmd_vel.linear.y = 0;
		this->cmd_vel.linear.z = 0;
		this->cmd_vel.angular.x = 0;
		this->cmd_vel.angular.y = 0;
		this->cmd_vel.angular.z = 0;
		this->cmd_vel_pub.publish(this->cmd_vel);
	}

	if (being_enabled)
	{
		ros::Time now = ros::Time::now();
		this->start_move_time.sec = now.sec;
		this->start_move_time.nsec = now.nsec;
	}

	bool_resp.success = true;
	bool_resp.message = "Updated DWA path follower's enable status";
	return true;
}

bool DWAPlanner::precision_cb(planner_interfacing::SetPrecision::Request &prec_req, planner_interfacing::SetPrecision::Response &prec_resp)
{
	this->precision = prec_req.precision;
	prec_resp.success = true;
	prec_resp.message = "Updated DWA path follower's precision";
	return true;
}

////// local functions ///////////////////////////////////////////////////////

void DWAPlanner::publish_status()
{
	// +------------+----------+-------------+
	// |            | waypoint | no waypoint |
	// +------------+-----------+------------+
	// | ability    | active   | idle        |
	// | no ability | blocked  | inactive    |
	// +------------+----------+-------------+

	bool has_waypoint = this->has_waypoint;
	bool has_ability = this->enabled && this->has_location;
	std_msgs::String status;

	if (has_ability && has_waypoint)
	{
		if (this->stuck)
			status.data = "stuck";
		else
			status.data = "active";
	}
	else if (has_ability && !has_waypoint)
		status.data = "idle";
	else if (not has_ability and has_waypoint)
		status.data = "blocked";
	else if (not has_ability and not has_waypoint)
		status.data = "inactive";
	this->current_status_pub.publish(status);
}

////// loop //////////////////////////////////////////////////////////////////

void DWAPlanner::loop()
{
	ros::Rate rate(this->update_freq);
	rate.sleep();
	while (ros::ok())
	{

		// if there are points in the path, go to them
		if (this->enabled && this->has_location && this->has_waypoint && this->planner.isInitialized())
		{

			// calculate error to global waypoint
			double rise = this->global_wp.y - this->current_position.y;
			double run = this->global_wp.x - this->current_position.x;
			double dist_to_global_wp = sqrt(pow(rise, 2) + pow(run, 2));
			double angle_to_global_wp = atan2(rise, run);
			double angle_error = angle_to_global_wp - this->current_yaw;
			if (angle_error > M_PI)
				angle_error -= 2*M_PI;
			else if (angle_error < -M_PI)
				angle_error += 2*M_PI;

			// if distance to next point is less than the set precision, we have arrived
			if (dist_to_global_wp < this->precision)
			{
				this->has_waypoint = false;
				this->path.clear();
				this->planner.setPlan(this->path);
				this->publish_status();
				continue;
			}

			// keep a local waypoint nearby the robot in the direction of the global waypoint
			geometry_msgs::Pose local_wp;
			local_wp.position.x = cos(angle_error) * this->local_wp_dist;
			local_wp.position.y = sin(angle_error) * this->local_wp_dist;
			local_wp.position.z = 0;

			geometry_msgs::PoseStamped local_wp_pose_st;
			local_wp_pose_st.header.stamp = ros::Time::now();
			local_wp_pose_st.header.frame_id = this->robot_frame;
			local_wp_pose_st.pose = local_wp;

			this->path.clear();
			this->path.push_back(local_wp_pose_st);
			this->planner.setPlan(this->path);

			// TODO: check for stuck

			// TODO: check for exceedingly bad angle, since DWA planner won't rotate in place

			// calculate and publish a command velocity using the DWA planner
			this->planner.computeVelocityCommands(this->cmd_vel);
			this->cmd_vel_pub.publish(this->cmd_vel);
			this->publish_status();
		}

		rate.sleep();
		ros::spinOnce();
	}
}
