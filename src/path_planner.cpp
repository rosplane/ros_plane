#include <ros/ros.h>
#include <ros_plane/Waypoint.h>

#define num_waypoints 5

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_plane_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<ros_plane::Waypoint>("waypoint_path",10);

    float Va = 10;//11;
    float wps[5*num_waypoints] = {
                0, 0, -40, 0*M_PI/180, Va,
                0, -20, -40, 0*M_PI/180, Va,
                90, -60, -40, 0*M_PI/180, Va,
                0, -100, -40, 0, Va,
                -80, -60, -40, 0*M_PI/180, Va,
               };

    for(int i(0);i<num_waypoints;i++)
    {

        ros_plane::Waypoint new_waypoint;

        new_waypoint.w[0] = wps[i*5 + 0];
        new_waypoint.w[1] = wps[i*5 + 1];
        new_waypoint.w[2] = wps[i*5 + 2];
        new_waypoint.chi_d = wps[i*5 + 3];

        new_waypoint.chi_valid = false;//false;
        new_waypoint.Va_d = wps[i*5 + 4];

        waypointPublisher.publish(new_waypoint);

        ros::Duration(0.5).sleep();
    }

    return 0;
}
