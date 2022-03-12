#include "ros/ros.h"
#include "ros/package.h"
#include "sim2real/sim2real_srv.h"
#include "geometry_msgs/Pose2D.h"
#include <fstream>
#include <iostream>
#include "std_msgs/String.h"
#include "sim2real/Navcore.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <atomic>
#include "tr1/memory"


class ServiceCaller{
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    ros::NodeHandle nh;
    ros::ServiceClient client;
    sim2real::sim2real_srv srv{};
    // std::tr1::shared_ptr<boost::thread> thread_ptr_;
    std::atomic_bool srvCalling_{},srvFinished_{};

public:
    ServiceCaller();
    ~ServiceCaller() = default;
    void worker(int SrvRequestType,Eigen::Vector3d transformation=Eigen::Vector3d(0,0,0));
    bool callSrv(int SrvRequestType,Eigen::Vector3d transformation=Eigen::Vector3d(0,0,0));
    const sim2real::sim2real_srv::Response & getSrvResponseStatus()const {return srv.response;};
    bool srvCalling() const { return srvCalling_;};
    bool srvFinished() {bool temp{srvFinished_};srvFinished_=false;
        return temp;};
};

class EP_Nav{
private:
    struct targetPose{
        geometry_msgs::Pose2D pose;
        enum TargetAction
        {
            ARRIVAL,
            UNARRIVAL,
        } targetAction{};
    };
    NavCore *navCore;
    bool nav_on{},nav_pause{},newGoal{true};
    std::vector<targetPose>targetPoseArray{};
    std::vector<targetPose>::iterator iter;
    ros::Publisher log_pub;
    ServiceCaller* serviceCaller;
    void checkSrvFinish();
    void getPoseArray();
    void setGoal(const geometry_msgs::Pose2D &goal2d);
    void setGoalInOrder();
public:

    void run();
    EP_Nav(const std::string& base_foot_print,std::string odom_frame,std::string map_frame,std::string serial_addr,bool publish_tf)
    {
        // baseController = new BaseController(serial_addr,B115200,base_foot_print,std::move(odom_frame),publish_tf);
        navCore = new NavCore(base_foot_print,std::move(map_frame));
        serviceCaller = new ServiceCaller;
    }
    ~EP_Nav()
    {

        delete serviceCaller;
        delete navCore;

    }
};



    typedef enum{
        SERVICE_STATUS_SUCCEED,
        SERVICE_STATUS_EMPTY,         //无法调用服务
    }ServiceStatus;

ServiceCaller::ServiceCaller()
{
    client =nh.serviceClient<sim2real::sim2real_srv>("manipulate");
}
void ServiceCaller::worker(int SrvRequestType,Eigen::Vector3d transformation)
{
    srvCalling_=true;
    srv.request.type=SrvRequestType;
	srv.request.transformation={transformation[0],transformation[1],transformation[2]};
    if (client.call(srv))
    {
        std::cout<<"The error code is "<<srv.response.status<<": "<<srv.response.status_string<<std::endl;
        srvFinished_=true;
    }
    else
    {
        ROS_ERROR("Failed to call service detect_once");
        srv.response.status = SERVICE_STATUS_EMPTY;
		srvFinished_=true;
    }
    srvCalling_=false;
}
bool ServiceCaller::callSrv(int SrvRequestType,Eigen::Vector3d transformation)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&ServiceCaller::worker, this, SrvRequestType,transformation)));
	thread_ptr_->detach();
}

void EP_Nav::getPoseArray()
{
    const std::string pose_addr{ros::package::getPath("sim2real")+"/config/NavTarget.txt"};
    std::ifstream input_file(pose_addr.c_str());
    if(input_file.is_open())
    {
        targetPoseArray.clear();
        std::string str;
        targetPose target_pose{};
        while(getline(input_file,str)&&!str.empty())
        {
            std::istringstream stringGet(str);
            int temp;
            stringGet>>temp>>target_pose.pose.x>>target_pose.pose.y>>target_pose.pose.theta;
            target_pose.targetAction=targetPose::TargetAction(temp);
            targetPoseArray.push_back(target_pose);
            printf("%f",target_pose.pose.x);
        }
        input_file.close();
        std_msgs::String log_string;
        log_string.data="The total number of targets is"+std::to_string(targetPoseArray.size());
        log_pub.publish(log_string);

    }
}



void EP_Nav::setGoalInOrder()
{
    if(nav_on && !targetPoseArray.empty()&&newGoal)
    {
        if(iter != targetPoseArray.end())
        {
            ROS_INFO_STREAM("now goal: x is "<<(*iter).pose.x<<" y is "<<(*iter).pose.y<<" theata is "<<(*iter).pose.theta);
            navCore->setGoal((*iter).pose);
            newGoal=false;
        }
        else
        {
            getPoseArray();
            iter = targetPoseArray.begin();
            ROS_INFO("Reach the end of the goal list, reload tree target");
            nav_on=false;
            newGoal=true;
        }
    }
}
void EP_Nav::checkSrvFinish()
{
    if(serviceCaller->srvFinished())
    {
        if(serviceCaller->getSrvResponseStatus().status!=SERVICE_STATUS_EMPTY)
        {
            navCore->cancelAllGoals();
            newGoal=false;
        }
        else
        {
            iter++;
            newGoal=true;
        }
    }
}
void EP_Nav::run()
{

    getPoseArray();
    iter = targetPoseArray.begin();
    if(iter!=targetPoseArray.end())
    {
        if(navCore->isGoalPassed((*iter).pose))
            iter++;
        newGoal = true;
        nav_pause=false;
    }
    checkSrvFinish();
    setGoalInOrder();
    // bool flag = client.call(srv);
    // if (flag)
    // {
    //     ROS_INFO("call success");
    // }
    // else
    // {
    //     ROS_INFO("call failed");
    //     return 1;
    // }

}

int main(int argc, char** argv)
{
    ros::init(argc , argv, "sim2real_client");
    ros::NodeHandle nh_;
    // ros::ServiceClient client = nh.serviceClient<sim2real::sim2real_srv>("pipeline");
    // ros::service::waitForService("pipeline");
    ros::Rate loop_rate(30);
    std::string base_foot_print,odom_frame,map_frame,serial_addr;
    bool publish_tf;
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
    nh_.param("odom_frame",odom_frame,(std::string)"odom");
    nh_.param("map_frame",map_frame,(std::string)"map");
    nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS1");
    nh_.param("publish_tf",publish_tf,(bool)false);
    EP_Nav nav(base_foot_print,odom_frame,map_frame,serial_addr,publish_tf);

    while(ros::ok())
    {
        nav.run();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}