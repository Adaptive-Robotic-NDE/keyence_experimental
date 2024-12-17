#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <pcl/common/projection_matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class KeyencePointCloudReconstruction : public rclcpp::Node
{
    public:
        KeyencePointCloudReconstruction() : Node("keyence_point_cloud_reconstruction")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "profiles", 10, std::bind(&KeyencePointCloudReconstruction::profiles_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keyence_point_cloud_reconstruction", 10);
        RCLCPP_INFO(this->get_logger(), "KeyencePointCloudReconstruction node started");
        //cloud_to_mesh_.setVoxelFilterSize(0.001);
    }

    private:
        void profiles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr profile)
        {
            if (profile->data.size() > 0){
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromROSMsg(*profile, *pc);

                //cloud_to_mesh_.setInput(pc);
                //if (cloud_to_mesh_.computeMesh())
                //{                
                //    visualization_msgs::Marker mesh_marker;
                //    meshToMarkerMsg(cloud_to_mesh_.getMesh() ,mesh_marker);
                //    pub_.publish(mesh_marker);
                //}
                //else{
                //    RCLCPP_WARN(this->get_logger(), "Could not generate mesh for point cloud!");
                //}
            }
            else{
                RCLCPP_INFO(this->get_logger(), "We got an empty point cloud");
            }
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
        // CloudToMesh<pcl::PointXYZ, pcl::PointNormal> cloud_to_mesh_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyencePointCloudReconstruction>());
    rclcpp::shutdown();
    return 0;
}