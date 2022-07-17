#include <ros/ros.h>

// pose message
#include <geometry_msgs/Pose.h>
// stamped pose message
#include <geometry_msgs/PoseStamped.h>

// marker visualization message
#include <visualization_msgs/Marker.h>

// for odom message
#include <nav_msgs/Odometry.h>

///////////////////
// Odometry class
///////////////////

class OdomSubscriber
{
public:
  OdomSubscriber(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
  {
	ROS_DEBUG("Initializing odometry subscriber");
  	// Subscribe to published point
  	odom_sub_ = nh_.subscribe("/odom", 1, &OdomSubscriber::subscriberCallback,this);
	received_odom_= false;
  };
  bool getOdomStatus() { return received_odom_; }
  void setOdomStatus(bool new_status) { received_odom_ = new_status; }
  // return a reference to the odometry coordinates
  std::pair<double, double> &getOdomCoords() { return odom_coords_; }

private:
  ros::NodeHandle nh_;
  // subscriber to odom
  ros::Subscriber odom_sub_;
  // Set odometry coordinates from odometry message
  void subscriberCallback(const nav_msgs::Odometry& msg)
  {
	odom_coords_.first = msg.pose.pose.position.x;
  	odom_coords_.second = msg.pose.pose.position.y;
  	ROS_DEBUG("Pos from Odom received: (%f,%f)", odom_coords_.first, odom_coords_.second);
  	received_odom_ = true;
  }
  // flag to keep track whether odometry has been received or not
  bool received_odom_;
  // keep track of odometry coordinates
  std::pair<double, double> odom_coords_;
};

///////////////////
// Marker Publisher
///////////////////

class MarkerPublisher
{
public:
    MarkerPublisher(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
    {
	ROS_INFO("Initializing Marker Publisher");
  	marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  	// test/wait to make sure a subscriber is currently connected to this Publisher
  	// because it requires time to create a connection between nodes, the connection
  	// is confirmed in order to avoid having to enter a loop to publish a marker
  	// or risk that te message is sent but not received
  	while (marker_pub.getNumSubscribers() < 1)
  	{
    		ROS_WARN_ONCE("Waiting until Rviz Marker publisher has subscriber (e.g. add display in Rviz)");
    		sleep(1);
  	}
    }

    // build and publish visualization marker
    void publish(visualization_msgs::Marker &marker, std::string action)
    {
	if ( action == "ADD") {
      		marker.action = visualization_msgs::Marker::ADD;
  	}
  	else if ( action == "DELETE") {
      		marker.action = visualization_msgs::Marker::DELETE;
  	}
  	else {
      		ROS_ERROR("No (or wrong) action field set to specify how the marker behaves (options are ADD, DELETE)");
  	}
  	marker_pub.publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub;
};

//
// Utility functions
//

/* Calculate euclidean distance between two points */
double pose2XYdistance(
	const geometry_msgs::PoseStamped& global_pose,
	double goal_x,
    	double goal_y
)
{
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
}

/* Check if current pose is equal to desired position considering a tolerance */
bool isDesiredPointReached(
	const geometry_msgs::PoseStamped& global_pose,
	const geometry_msgs::Pose& dp_pose
)
{
    double xy_goal_tolerance = 0.40; // set tolerance as required
    double goal_x = dp_pose.position.x; // goal_pose.pose.position.x;
    double goal_y = dp_pose.position.y; // goal_pose.pose.position.y;

    //ROS_INFO("Dist=%f curr_x=%f curr_y=%f goal_x=%f goal_y=%f",
	// pose2XYdistance(global_pose, goal_x, goal_y),
	// global_pose.pose.position.x, global_pose.pose.position.y,
	// goal_x, goal_y);
    //  check to see if we've reached the waypoint position
    if (pose2XYdistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
	return true;
    }
    return false;
}

// Create Marker
void createMarker(
	visualization_msgs::Marker & marker,
	int id,
	double x, double y, double z, double w)
{
  // Set the frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = w;


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.3f;
  marker.color.g = 0.5f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); // create ROS node
  ros::NodeHandle n; // start node

  // initialize odometry subscriber (topic: /odom)
  OdomSubscriber odomSub(&n);

  // current pose
  geometry_msgs::PoseStamped current_pose;

  // desired position
  geometry_msgs::Pose desired_position;

  // init MarkerPublisher object
  MarkerPublisher rvizMarkerPub(&n);

  // Create marker
  visualization_msgs::Marker marker_at_pickup;
  visualization_msgs::Marker marker_at_dropoff;
  createMarker(marker_at_pickup, 0, 1.0, -4.0, 0.0, 1.0);
  createMarker(marker_at_dropoff, 1, 1.0, 4.0, 0.0, -1.0);

  // variable used to cycle trough different tasks
  uint8_t task = 0;

  // make best effort at maintaining a particular rate for a loop
  ros::Rate rate(0.5);// in Hz

  while(ros::ok())
  {
  	// get newest data for all subscribers
  	ros::spinOnce();
  	// update global pose header
  	current_pose.header.stamp = ros::Time::now();
  	// update odometry
  	current_pose.pose.position.y = -odomSub.getOdomCoords().first;
  	current_pose.pose.position.x = odomSub.getOdomCoords().second;

  	switch (task)
    	{
    	case 0:
      		ROS_INFO_ONCE("At start location");
      		// build a Rviz marker message and publish it
      		rvizMarkerPub.publish(marker_at_pickup, "ADD");
      		task = 1;
      		break;
    
    	case 1:
      		ROS_INFO_ONCE("Moving to pick up place");
      		desired_position.position.x = marker_at_pickup.pose.position.x;
      		desired_position.position.y = marker_at_pickup.pose.position.y;

      		if (isDesiredPointReached(current_pose, desired_position))
      		{
        		task = 2;
      		}
      		break;
    
    	case 2:
      		ROS_INFO_ONCE("At pick up place, Sleep for 5 sec");
      		// Wait 5 seconds to simulate a pickup
      		// construct a ros::Duration object, then call its sleep() method
      		ros::Duration(5, 0).sleep();
      		// build a Rviz marker message and publish it
      		rvizMarkerPub.publish(marker_at_pickup, "DELETE");
      		// move to next task
      		task = 3;
      		break;
    
    	case 3:
      		ROS_INFO_ONCE("Moving to drop off zone");
      		// check if the drop off zone is reached
      		desired_position.position.x = marker_at_dropoff.pose.position.x;
      		desired_position.position.y = marker_at_dropoff.pose.position.y;
      		if (isDesiredPointReached(current_pose, desired_position))
      		{
        		// move to next task
        		task = 4;
      		}
      		break;
    
    	case 4:
      		ROS_INFO_ONCE("At drop off zone, drop marker");
      		// build a Rviz marker message and publish it
      		rvizMarkerPub.publish(marker_at_dropoff, "ADD");
      		break;
    }
    // track time since last rate.sleep() and stop the right amount of time to match the rate()
    rate.sleep();
  }
  return 0;
}
