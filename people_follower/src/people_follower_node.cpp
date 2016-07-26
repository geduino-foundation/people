#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <people_follower/PeopleFollowerConfig.h>

ros::Subscriber * positionMeasurementArraySubscriberPtr;

ros::Publisher * cmdVelPublisherPtr;

tf::TransformListener * transformListenerPtr;

std::string baseFrame;

double transformTimeout;

double x_vel_gain, x_vel_min, x_vel_max;
double th_vel_gain, th_vel_min, th_vel_max;
double d_thres, th_tres;

double limitGain(double gain, double value, double min, double max) {

    // Compute gained value
    double gained = gain * value;

    if (std::abs(gained) < min) {

        // Return min value
        return (gained > 0) ? min : -min;

    } else if (std::abs(gained) > max) {

        // Return max value
        return (gained > 0) ? max : -max;

    } else {
        return gained;
    }

}

void dynamicReconfigureCallback(people_follower::PeopleFollowerConfig & config, uint32_t level) {
    x_vel_gain = config.x_vel_gain;
    x_vel_min = config.x_vel_min;
    x_vel_max = config.x_vel_max;
    th_vel_gain = config.th_vel_gain;
    th_vel_min = config.th_vel_min;
    th_vel_max = config.th_vel_max;
    d_thres = config.d_thres;
    th_tres = config.th_thres;
    transformTimeout = config.transform_timeout;
}

void positionMeasurementArrayCallback(const people_msgs::PositionMeasurementArrayConstPtr & positionMeasurementArrayMsg) {

    if (positionMeasurementArrayMsg->people.size() > 0) {

        // Create stamped point from person position
        geometry_msgs::PointStamped personPoint;
        personPoint.header = positionMeasurementArrayMsg->people[0].header;
        personPoint.point = positionMeasurementArrayMsg->people[0].pos;

        std::string errorMsg;

        if (transformListenerPtr->waitForTransform(baseFrame,
                                               personPoint.header.frame_id,
                                               personPoint.header.stamp,
                                               ros::Duration(transformTimeout),
                                               ros::Duration(0.01),
                                               & errorMsg)) {

            geometry_msgs::PointStamped personPointInBaseFrame;

            // Transform from person frame to base frame
            transformListenerPtr->transformPoint(baseFrame, personPoint, personPointInBaseFrame);

            // Calculate distance and angular difference
            double d = std::sqrt(std::pow(personPointInBaseFrame.point.x, 2) + std::pow(personPointInBaseFrame.point.y, 2));
            double th = (personPointInBaseFrame.point.x != 0) ? std::atan(personPointInBaseFrame.point.y / personPointInBaseFrame.point.x) : 0;

            double x_vel = 0;

            if (d > d_thres) {

                // Compute x velocity
                x_vel = limitGain(x_vel_gain, d, x_vel_min, x_vel_max);

            }

            double th_vel = 0;

            if (std::abs(th) > th_tres) {

                // Compute angular velocity
                th_vel = limitGain(-th_vel_gain, th, th_vel_min, th_vel_max);

            }

            // Compute velocity command
            geometry_msgs::Twist cmdVelMsg;
            cmdVelMsg.linear.x = x_vel;
            cmdVelMsg.angular.z = th_vel;

            // Publish cdm vel
            cmdVelPublisherPtr->publish(cmdVelMsg);


        } else {

            // Log
            ROS_ERROR_THROTTLE(1.0, "Cannot transform from %s -> %s at %f: %s",
                               baseFrame.c_str(),
                               personPoint.header.frame_id.c_str(),
                               personPoint.header.stamp.toSec(),
                               errorMsg.c_str());

        }

    }


}

int main(int argc, char** argv) {

    // Init ros
    ros::init(argc, argv, "people_follower");

    // Create node handle
    ros::NodeHandle nodeHandle;

    // Create private node handle
    ros::NodeHandle privateNodeHandle("~");

    // Get params
    privateNodeHandle.param<std::string>("base_frame", baseFrame, "base_link");

    // Setup dynamic reconfigure server
    dynamic_reconfigure::Server<people_follower::PeopleFollowerConfig> dynamicReconfigureServer;
    dynamic_reconfigure::Server<people_follower::PeopleFollowerConfig>::CallbackType dynamicReconfigureCallbackFunction;
    dynamicReconfigureCallbackFunction = boost::bind(& dynamicReconfigureCallback, _1, _2);
    dynamicReconfigureServer.setCallback(dynamicReconfigureCallbackFunction);

    // Subscribe position measurement array
    ros::Subscriber positionMeasurementArraySubscriber = nodeHandle.subscribe("/people_tracker_measurements", 5, positionMeasurementArrayCallback);
    positionMeasurementArraySubscriberPtr = & positionMeasurementArraySubscriber;

    // Advertise cmd vel
    ros::Publisher cmdVelPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    cmdVelPublisherPtr = & cmdVelPublisher;

    // Create transform listener
    tf::TransformListener transformListener;
    transformListenerPtr = & transformListener;

    // Spin
    ros::spin();

    return 0;

}
