#include <ros/ros.h>
#include <ros_plane/Waypoint.h>

#define num_waypoints 9

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_plane_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<ros_plane::Waypoint>("waypoint_path",10);

    float Va = 15;//11;
    float wps[5*num_waypoints] = {
                0, 0, -60, 0*M_PI/180, Va,
                10, 50, -60, 0*M_PI/180, Va,
                80, 20, -60, 0, Va, 
                110, -50, -60, 0*M_PI/180, Va,
                80, -120, -60, 0, Va,
                10, -150, -60, 0, Va,
                -60, -120, -60, 0, Va, 
                -90, -50, -60, 0, Va,
                -60, 20, -60, 0*M_PI/180, Va,
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
