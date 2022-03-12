#include "ros/ros.h"
#include "sim2real/sim2real_srv.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/Core>

struct epSrv{
    int srv_type;
	Eigen::Vector3d transformation;
    int srv_status;
};

epSrv epSrv{};

class ep_sim
{
    public:
    int executeService(int serviceType);
};

int ep_sim::executeService (int serviceTpye)
{

}

bool pipline(sim2real::sim2real_srv::Request& req,
          sim2real::sim2real_srv::Response& res)
{
    epSrv.srv_type = req.type;
    if(!req.transformation.empty())
	{
		epSrv.transformation[0]=req.transformation[0];
		epSrv.transformation[1]=req.transformation[1];
		epSrv.transformation[2]=req.transformation[2];
	}
    res.status=epSrv.srv_status;
    ROS_INFO("service execute successfully!");
}

int main(int argc, char** argv)
{
    ros::init(argc , argv, "sim2real_service");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("pipeline",pipline);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ep_sim ep;
    while(ros::ok())
    {
        ep.executeService(epSrv.srv_type);
    }

    return 0;
}