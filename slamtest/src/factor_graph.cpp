#include <iostream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
using namespace std;
using namespace gtsam;
class ROSFactorGraph
{
private:
    // Create graph, prior factor noise model, and odometry noise model
    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas((Vector(6) << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1).finished());
    noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1).finished());
    noiseModel::Diagonal::shared_ptr measurementModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 0.05, 0.01, 0.01, 0.01).finished());
    // Create node handle for publishers/subscribers
    ros::NodeHandle n;
    ros::Subscriber odomSub = n.subscribe("odometry/filtered", 1000, &ROSFactorGraph::odomCallback, this);
    ros::Subscriber aprilTagSub = n.subscribe("tag_detections", 1000, &ROSFactorGraph::tagCallback, this);

    // Make counter for odometry values to be labeled as symbols in map
    int counter = 1;

    // Get starting time, use as timer (time is in seconds)
    ros::Time timer = ros::Time::now();

    // Create initial values variable
    Values initials;
    GaussNewtonParams parameters;

    // Keep track of last odometry message passed in
    gtsam::Point3 last_position = gtsam::Point3(0, 0, 0);
    gtsam::Quaternion last_quat = gtsam::Quaternion(0, 0, 0, 1);

    std::vector<int> tags_list{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    std::vector<int> seen_tags;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;

public:
    // Constructor
    ROSFactorGraph()
    {
        // Initialize priorfactor in constructor
        gtsam::Rot3 rot(0, 0, 0, 1);
        gtsam::Point3 point(0, 0, 0);
        graph.add(PriorFactor<Pose3>(Symbol('x', 0), Pose3(rot, point), priorModel));
        initials.insert(Symbol('x', 0), Pose3(rot, point));
        parameters.setVerbosity("ERROR");
        tfListener = new tf2_ros::TransformListener(tfBuffer);
    }

    // make callback for odometry to add it to
    void odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {

        // Make a case to get the priorfactor if its the first time it receives an odometry message

        // Get quaternion and point data from odom message
        gtsam::Quaternion quat(odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y,
                               odom->pose.pose.orientation.z,
                               odom->pose.pose.orientation.w);

        gtsam::Rot3 rot(quat);
        gtsam::Point3 point(odom->pose.pose.position.x,
                            odom->pose.pose.position.y,
                            odom->pose.pose.position.z);

        gtsam::Pose3 current_guess(rot, point);

        // Get odometry by getting difference in position and rotation
        gtsam::Point3 delta_position = point - last_position;
        gtsam::Quaternion delta_quat = last_quat * quat.inverse();
        gtsam::Pose3 pose3(Rot3(delta_quat), delta_position);

        // Add odometry between factor and initial guess
        graph.add(BetweenFactor<Pose3>(Symbol('x', counter - 1), Symbol('x', (counter)), pose3, odomModel));
        initials.insert(Symbol('x', counter), current_guess);

        // print out graph every 5 seconds
        if ((ros::Time::now() - timer).toSec() > 5)
        {
            // graph.print("\nFactor Graph: \n");
            timer = ros::Time::now();

            GaussNewtonOptimizer optimizer(graph, initials, parameters);
            // Values results = optimizer.optimize();
            // results.print("\nFinal Result:\n");
        }

        // Update timer/counting variables
        last_quat = quat;
        last_position = point;
        counter++;
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray &detections_message)
    {
        ros::Time ts = ros::Time(detections_message.header.stamp);
        for (unsigned int i = 0; i < detections_message.detections.size(); i++)
        {
            apriltag_ros::AprilTagDetection tag = detections_message.detections[i];
            int id = tag.id[0];

            if (std::find(tags_list.begin(), tags_list.end(), id) != tags_list.end())
            { // Found valid tag
                if ((abs(tag.pose.pose.pose.position.x) <= 0.001) && (abs(tag.pose.pose.pose.position.y) <= 0.001) && (abs(tag.pose.pose.pose.position.z) <= 0.001))
                {
                    // std::cout << "found a zeroed out tag" << std::endl;
                }
                else
                {
                    gtsam::Point3 t(tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z);
                    gtsam::Rot3 R(tag.pose.pose.pose.orientation.x, tag.pose.pose.pose.orientation.y, tag.pose.pose.pose.orientation.z,
                                  tag.pose.pose.pose.orientation.w);
                    gtsam::Pose3 pose(R, t);
                    addTagFactor(pose, id, ts);
                }
            }
        }
    }

    void addTagFactor(const gtsam::Pose3 &pose, int id, ros::Time ts)
    {
        geometry_msgs::TransformStamped tf_stamped;

        try
        {
            tf_stamped = tfBuffer.lookupTransform("base_link", "camera", ts);
            auto trans = tf_stamped.transform.translation;
            auto rot = tf_stamped.transform.rotation;
            gtsam::Point3 ptc(trans.x,trans.y,trans.z);
            gtsam::Rot3 pRc(gtsam::Quaternion(rot.w,rot.x,rot.y,rot.z));
            gtsam::Pose3 pTc(pRc, ptc);
            gtsam::Pose3 cTt = pose; //tag in camera frame
            gtsam::Pose3 pTt = pTc * cTt; // tag in platform frame

            graph.add(BetweenFactor<Pose3>(Symbol('x', counter), Symbol('l', id), pTt, measurementModel));

            // Determine if we've seen the tag before, and if not, add an initial value to the landmark along with the Between Factor
            if (!(std::find(seen_tags.begin(), seen_tags.end(), id) != seen_tags.end()))
            {
                cout << "Adding tag id: " << id << endl;
                seen_tags.push_back(id);

                // from tag in robot frame, get tag in world.
                // -----------------

                //Get last estimate of where robot is in world frame, easy using odometry messages
                gtsam::Pose3 wTp;

                //get tag in world frame estimate
                gtsam::Pose3 wTt = wTp * pTt;
                // -----------------
                initials.insert(Symbol('l', id), wTt);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ros::Duration(0.1).sleep();
        }
    }
};

int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "FactorGraph");
    ROSFactorGraph graph;
    cout << "Hola, mundo!" << endl;
    // Start spin to allow subscriber to continue
    ros::spin();
    return 0;
}