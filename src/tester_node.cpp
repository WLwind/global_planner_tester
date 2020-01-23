#include <geometry_msgs/PointStamped.h>
#include <global_planner_tester/global_planner_tester.h>

void ClickPointCB(const geometry_msgs::PointStamped::ConstPtr& click_point_ptr,global_planner_tester::GlobalPlannerTester* planner_tester)
{
    geometry_msgs::PoseStamped pose;
    pose.header=click_point_ptr->header;
    pose.pose.position=click_point_ptr->point;
    pose.pose.orientation.z=1.0;//set a quaurnion (0,0,0,1)
    bool result=planner_tester->testPlan(pose);
    std::cout<<"testPlan result is "<<result<<std::endl;
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_planner_tester");
    ros::NodeHandle nh;
    global_planner_tester::GlobalPlannerTester tester;
    ros::Subscriber click_sub{nh.subscribe<geometry_msgs::PointStamped>("/clicked_point",10,boost::bind(&ClickPointCB,_1,&tester))};//pass the tester into the callback function
    ros::spin();
    return 0;
}