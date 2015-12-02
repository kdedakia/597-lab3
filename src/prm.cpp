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
#include <vector>

using namespace Eigen;
using geometry_msgs::Point;
using geometry_msgs::PoseWithCovarianceStamped;
using std::vector;

ros::Publisher marker_pub;

#define SIMULATION // change this to live if necessary

#define TAGID 0
#define COLS 100
#define ROWS 100
#define numMilestones 95 //Number of Samples
#define nhDistance 10.0 //Neighborhood Distance
#define obsThreshold 50

vector<int8_t> gMap;
bool initial_pose_found = false;
vector<Point> milestones;
Point startPoint;
Point currPoint;
vector<Point> wayPoints;
vector<Point> path;

int8_t getGridVal(uint8_t x, uint8_t y) {
	return gMap[(y*10)*COLS + x*10];
}

// Used to display milestones & samples on RVIZ
void draw_points(vector<Point> points_vector) {
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
	points.points = points_vector;
	marker_pub.publish(points);
}

// Euclidian Distance between 2 points
double dist(Point p1,Point p2) {
	return pow(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2), 0.5);
}

// Generate the graph connections for each milestone
// TODO: ensure milestones array passed in is correct
// void gen_connections(Point points[])
// {
// 	int mS = 3;
// 	// TODO: don't hardcode the size? Doesn't compile if a var is passed in...
// 	Matrix<int, 3, 3> map;
// 	map.fill(0);

// 	for(int i = 0; i < mS; i++) {
// 		double norms[mS];
// 		std::cout << points[i].x << std::endl;

// 		for (int j = 0; j < mS; j++) {
// 			// Set neighbors based on proximity
// 			double d = dist(points[i],points[j]);
// 			if ( i != j && d > 0.0  && d < nhDistance) {
// 				// TODO: collision check before adding edge
// 				map(i,j) = 1;
// 				map(j,i) = 1;
// 			}
// 		}
// 	}

// 	std::cout << map << std::endl;
// }

// Generate an array of milestones, display on RVIZ
void gen_milestones() {	
	while (milestones.size() < numMilestones) {
		int x = rand() % COLS;
		int y = rand() % ROWS;
		int z = 0;
		if (getGridVal(x,y) < obsThreshold) {
			Point p; p.x = (double)x; p.y = (double)y; p.z = (double)z;
			milestones.push_back(p);
		}
	}
	draw_points(milestones);
}

//Callback function for the Position topic (LIVE)
void pose_callback(const PoseWithCovarianceStamped &msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	if (!initial_pose_found) {
		startPoint.x = X;
		startPoint.y = Y;
		startPoint.z = Yaw;
	}
	initial_pose_found = true;
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
       Point p;
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

void drawLineSegment(int k, Point start_point, Point end_point)
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
void map_callback(const nav_msgs::OccupancyGrid& msg) {
	gMap = msg.data;
}

bool find_shortest_path() {
	// TODO
	return false;
}

void init_map() {
    for (int i = 0; i < ROWS*COLS; i++) {
        gMap.push_back(0);
    }
}

void init_vectors() {
	milestones.clear();
	wayPoints.clear();
	path.clear();

	// For simulation - insert waypoints
#ifdef SIMULATION
	Point p1; p1.x = 4.0; p1.y = 0.0; p1.z = 0.0;
	Point p2; p2.x = 8.0; p2.y = -4.0; p2.z = 3.14;
	Point p3; p3.x = 8.0; p3.y = 0.0; p3.z = -1.57;
	wayPoints.push_back(p1);
	wayPoints.push_back(p2);
	wayPoints.push_back(p3);
	milestones.push_back(p1);
	milestones.push_back(p2);
	milestones.push_back(p3);
#else
	// For live
	Point p1; p1.x = 5.0; p1.y = 5.0; p1.z = 0.0;
	Point p2; p2.x = 1.0; p2.y = 0.0; p2.z = 3.14;
	Point p3; p3.x = 1.5; p3.y = 4.5; p3.z = -1.57;
	Point p4; p4.x = 3.0; p4.y = 0.5; p4.z = 1.57;
	wayPoints.push_back(p1);
	wayPoints.push_back(p2);
	wayPoints.push_back(p3);
	wayPoints.push_back(p4);
	milestones.push_back(p1);
	milestones.push_back(p2);
	milestones.push_back(p3);
	milestones.push_back(p4);
#endif
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    srand(time(NULL));
    init_map();
    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
	ROS_INFO("Subscribed to everything! Waiting for initial pose.");

	ros::Rate loop_rate(20);    //20Hz update rate
	while (!initial_pose_found) {
		loop_rate.sleep();
		ros::spinOnce();
	}
	ROS_INFO("Receieved initial pose!");

	wayPoints.push_back(startPoint);

	ROS_INFO("Initial waypoints and milestones updated!");

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    do {
    	init_vectors();
		gen_milestones();			// updates milestones global vector
		// gen_connections(); 			// updates global Matrix of size numMilestones x numMilestones
    } while (find_shortest_path());
	
    while (ros::ok()) {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }
    return 0;
}
