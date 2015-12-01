//  ///////////////////////////////////////////////////////////
//
// prm.cpp
// ME 597 Lab 3 Code: Probabalistic Roadmap Algorithm
//
//
// //////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

ros::Publisher marker_pub;

#define TAGID 0
#define numSamples 10 //Number of Samples
#define nhDistance 10.0 //Neighborhood Distance


// Used to display milestones & samples on RVIZ
void draw_points(geometry_msgs::Point points_data[])
{
	visualization_msgs::Marker points;
	points.header.frame_id = "/map";
	points.ns = "Points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.g = 1.0f;
	points.color.a = 1.0;

	for(int i = 0; i < numSamples; i++ ) {
		points.points.push_back(points_data[i]);
	}

	marker_pub.publish(points);
}


// Euclidian Distance between 2 points
double dist(geometry_msgs::Point p1,geometry_msgs::Point p2) {
	return pow(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2), 0.5);
}


// Generate the graph connections for each milestone
// TODO: ensure milestones array passed in is correct
void gen_connections(geometry_msgs::Point points[])
{
	int mS = 3;
	// TODO: don't hardcode the size? Doesn't compile if a var is passed in...
	Matrix<int, 3, 3> map;
	map.fill(0);

	for(int i = 0; i < mS; i++) {
		double norms[mS];
		std::cout << points[i].x << std::endl;

		for (int j = 0; j < mS; j++) {
			// Set neighbors based on proximity
			double d = dist(points[i],points[j]);
			if ( i != j && d > 0.0  && d < nhDistance) {
				// TODO: collision check before adding edge
				map(i,j) = 1;
				map(j,i) = 1;
			}
		}
	}

	std::cout << map << std::endl;
}


// Generate an array of milestones, display on RVIZ
void gen_milestones()
{
		//Set up map bounds
		//TODO: verify bounds
		Vector2d xMax(9,5);
		Vector2d xMin(-1,-5);
		Vector2d xR;
		xR = xMax - xMin;

		// Set up goals
		Vector3d w1(5,5,0);
		Vector3d w2(1,0,3.14);
		Vector3d w3(1.5,4.5,-1.57);
		Vector3d w4(3,0.5,1.57);

		// TODO: get obstacles

		// Get milestones
		Vector2d samples [numSamples];
		geometry_msgs::Point p_data [numSamples];

		for(int i = 0; i < numSamples; i++) {
			double r1 = ((double) rand() / (RAND_MAX));
			double r2 = ((double) rand() / (RAND_MAX));

			// TODO: Vector to Point conversion?
			//For RVIZ
			geometry_msgs::Point p;
			p.x = xR(0)*r1 + xMin(0);
			p.y = xR(1)*r2 + xMin(1);
			p.z = 0;
			p_data[i] = p;

			Vector2d s(xR(0)*r1 + xMin(0), xR(1)*r2 + xMin(1));
			samples[i] = s;
			// std::cout << s << std::endl;
		}

		Vector3d milestones [3];
		milestones[0] = w1;
    std::cout << milestones[0](0) << std::endl;

		// TODO: select valid milestones

		draw_points(p_data);
}


//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	// std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}


//Example of drawing a curve
void drawCurve(int k)
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p);

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);
   }

   //publish new curve
   marker_pub.publish(lines);

}


void drawLineSegment(int k, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "line_segments";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   lines.points.push_back(start_point);
   lines.points.push_back(end_point);

   //publish new line segment
   marker_pub.publish(lines);
}


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    geometry_msgs::Point start_, end_;

    start_.x = 12.0;
    start_.y = 12.0;
    start_.z = 0.0;
    end_.x = -12.0;
    end_.y = -12.0;
    end_.z = 0.0;

		//Test points
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Point p3;
		p1.x = 1.0;
		p1.y = 1.0;
		p1.z = 0.0;
		p2.x = 3.0;
		p2.y = 3.0;
		p2.z = 0.0;
		p3.x = 10.0;
		p3.y = 10.0;
		p3.z = 0.0;
		geometry_msgs::Point points[3];
		points[0] = p1;
		points[1] = p2;
		points[2] = p3;

		// gen_milestones();
		gen_connections(points);

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

			//Draw Curves
			// drawCurve(1);
			// drawCurve(2);
			// drawCurve(4);
	    // drawLineSegment(5,start_,end_);

    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity

    }
    return 0;
}
