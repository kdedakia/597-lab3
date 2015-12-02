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
ros::Publisher line_pub;

#define SIMULATION // change this to live if necessary

#define TAGID 0
#define COLS 100
#define ROWS 100
#define numMilestones 95 //Number of Samples
#define nhDistance 2.0 //Neighborhood Distance
#define obsThreshold 50
#define angleThreshold 10
#define distThreshold 2


vector<int8_t> gMap;
bool ad_grid[numMilestones][numMilestones] = {false};
bool initial_pose_found = false;
vector<Point> milestones;
Point startPoint;
Point currPoint;
vector<Point> wayPoints;
vector<Point> path;


int8_t getGridVal(uint8_t x, uint8_t y) {
	return gMap[y*COLS + x];
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

void drawLineSegments(vector<Point> lines_vector)
{
   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = 0; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_LIST;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "line_segments";
   lines.scale.x = 0.04;
   lines.color.r = 1.0;
   lines.color.b = 0.2;
   lines.color.a = 1.0;

   lines.points = lines_vector;

   //publish new line segment
   line_pub.publish(lines);
}

// Euclidian Distance between 2 points
double dist(Point p1,Point p2) {
	return pow(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2), 0.5);
}

int indexX(double x){
 return (x + 1) * 10;
}

int indexY(double y){
 return (y + 5) * 10;
}

short sgn(int x) { return x >= 0 ? 1 : -1; }

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

bool isFeasible (Point node1, Point node2){
	vector<int> x;
	vector<int> y;
	int x0 = indexX(node1.x);
	int y0 = indexY(node1.y);
	int x1 = indexX(node2.x);
	int y1 = indexY(node2.y);
	bresenham (x0,y0,x1,y1,x,y);
	int radius = 2;
	for (int i =0; i < x.size(); i ++){
		for (int j = 0; j <= radius; j++){
			if (getGridVal(x.at(i),y.at(i)-j) > obsThreshold ||
				getGridVal(x.at(i),y.at(i)+j) > obsThreshold ||
				getGridVal(x.at(i)+j,y.at(i)) > obsThreshold ||
				getGridVal(x.at(i)-j,y.at(i)) > obsThreshold ||
				getGridVal(x.at(i)-j,y.at(i)-j) > obsThreshold ||
				getGridVal(x.at(i)+j,y.at(i)+j) > obsThreshold){
				return false;
			}
		}
	}
	return true;
}

// Generate the graph connections for each milestone
// TODO: ensure milestones array passed in is correct
void gen_connections()
{	vector<Point> lines_vector;
	for(int i = 0; i < numMilestones; i++) {
		for (int j = 0; j < numMilestones; j++) {
			float d = dist(milestones[i],milestones[j]);
			if (i != j &&
				d > 0.0 &&
				d < nhDistance &&
				isFeasible(milestones[i], milestones[j])) {
				ad_grid[i][j] = true;
				ad_grid[j][i] = true;
				lines_vector.push_back(milestones[i]);
				lines_vector.push_back(milestones[j]);
			}
		}
	}
	drawLineSegments(lines_vector);
}

double actualX(int x) {
	return ((double)x/10) - 1.0;
}

double actualY(int y) {
	return ((double)y/10) - 5.0;
}

// Generate an array of milestones, display on RVIZ
void gen_milestones() {
	while (milestones.size() < numMilestones) {
		int x = rand() % COLS;
		int y = rand() % ROWS;
		if (getGridVal(x,y) < obsThreshold) {
			Point p; p.x = actualX(x); p.y = actualY(y); p.z = 0.0;
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
	currPoint.x = X;
	currPoint.y = Y;
	currPoint.z = Yaw;
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

float get_angle(Point dest) {
	float x1 = cos(currPoint.z);
	float y1 = sin(currPoint.z);
	float x2 = dest.x - currPoint.x;
	float y2 = dest.y - currPoint.y;
	float dot = x1*x2 + y1*y2;
	float det = x1*y2 - y1*x2;
	return fabs(atan2(det, dot) * 180 / 3.14);
}

bool navigate(Point dest,ros::Publisher velocity_publisher) {
	float delta_angle = get_angle(dest);
	float delta_dist = dist(currPoint,dest);

	geometry_msgs::Twist vel;

	std::cout << "DELTA DIST: " << delta_dist << " DELTA ANGLE: " << delta_angle << std::endl;

	if (delta_dist > distThreshold) {
		delta_dist = dist(currPoint,dest);
		delta_angle = get_angle(dest);

		if(delta_angle > angleThreshold) {
			std::cout << "ROTATE: " << delta_angle << std::endl;
			vel.linear.x = 0.0;
			vel.angular.z = 0.3; // rotate CW
    	velocity_publisher.publish(vel);
		} else {
			std::cout << "FORWARD:" << delta_dist << " | " << delta_angle << std::endl;
			vel.linear.x = 0.1; // move forward
    	vel.angular.z = 0.0;
    	velocity_publisher.publish(vel);
		}
	} else {
		vel.linear.x = 0.0; // stop
		vel.angular.z = 0.0;
		velocity_publisher.publish(vel);
		return true;
	}

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

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
		ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    line_pub = n.advertise<visualization_msgs::Marker>("visualization_line", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    do {
    	init_vectors();
		gen_milestones();			// updates milestones global vector
		gen_connections(); 			// updates global Matrix of size numMilestones x numMilestones
    } while (find_shortest_path());

		Point dest;
		dest.x = 0.311;
		dest.y = -5.3;

    while (ros::ok()) {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

			// navigate(dest,velocity_publisher);
    }
    return 0;
}
