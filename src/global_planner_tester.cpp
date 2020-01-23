#include <vector>
#include <cmath>
#include <pluginlib/class_loader.h>
#include <global_planner_tester/global_planner_tester.h>

namespace global_planner_tester
{
GlobalPlannerTester::GlobalPlannerTester()
{
    ros::NodeHandle private_nh("~");//private node handle to get the plugin's name
    std::string global_planner;//name of the plugin
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> global_planner_loader("nav_core", "nav_core::BaseGlobalPlanner");//nav_core interface
    m_costmap2d_ros_ptr.reset(new costmap_2d::Costmap2DROS("global_costmap",m_tf));
    m_costmap2d_ros_ptr->pause();
    try
    {
        m_global_planner_ptr=global_planner_loader.createInstance(global_planner);//load plugin
        m_global_planner_ptr->initialize(global_planner_loader.getName(global_planner),m_costmap2d_ros_ptr.get());
        ROS_INFO("The global planner %s is ready to be tested.",global_planner_loader.getName(global_planner).c_str());
    }
    catch(const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(1);
    }
    m_costmap2d_ros_ptr->start();
}

bool GlobalPlannerTester::testPlan(const geometry_msgs::PoseStamped& goal)
{
    bool result{false};
    if(!m_first_point)
    {
        clock_t begin,end;
        double sec,length{0.0},x,y;
        std::vector<geometry_msgs::PoseStamped> plan;//stores the path generated from the planner
        begin=clock();
        result=m_global_planner_ptr->makePlan(m_last_point,goal,plan);
        end=clock();
        sec=static_cast<double>(end-begin)/CLOCKS_PER_SEC;//calculates the cost time of planning the global plan
        for(auto& pose:plan)//calculate the distance between every two sequential point
        {
            if(&pose!=std::addressof(*plan.begin()))
            {
                length+=hypot(x-pose.pose.position.x,y-pose.pose.position.y);
            }
            x=pose.pose.position.x;
            y=pose.pose.position.y;
        }
        ROS_INFO("It took the planner %f s to finish planning. Size of the plan is %lu. The length of the generated path is %f m.",sec,plan.size(),length);
    }
    else
    {
        std::cout<<"Please click the 2nd point."<<std::endl;
        m_first_point=false;
    }
    m_last_point=goal;//store the last pose as the next start pose
    return result;
}
}//end namespace
