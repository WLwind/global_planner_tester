#include <memory>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>

namespace global_planner_tester
{
/**
 * @class GlobalPlannerTester
 * @brief A tool to test global planner plugins.
 */
class GlobalPlannerTester
{
public:
    /**
     * @brief Constructor
     */
    GlobalPlannerTester();
    /**
     * @brief Destructor
     */
    virtual ~GlobalPlannerTester(){};
    /**
     * @brief Test the global planner from m_last_point to goal
     * @param goal The goal pose for the global planner
     * @return True if the global planner made the plan successfully, false otherwise
     */ 
    bool testPlan(const geometry_msgs::PoseStamped& goal);
private:
    tf::TransformListener m_tf{ros::Duration(10)};//tf listerner
    std::unique_ptr<costmap_2d::Costmap2DROS> m_costmap2d_ros_ptr;//costmap
    geometry_msgs::PoseStamped m_last_point;//last input point or pose
    boost::shared_ptr<nav_core::BaseGlobalPlanner> m_global_planner_ptr;//global planner plugin
    bool m_first_point{true};
};
}//end namespace
