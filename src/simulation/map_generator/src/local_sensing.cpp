#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
bool has_map = false;
bool has_odom = false;

// 地图参数
double resolution, x_size, y_size, z_size;
Eigen::Vector3d local_range;

// 当前UAV位置
Eigen::Vector3d current_position;

// 发布器
ros::Publisher local_map_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    has_odom = true;
}

void mockMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!has_map) {
        pcl::fromROSMsg(*msg, *full_cloud);
        kdtree.setInputCloud(full_cloud);
        has_map = true;
        ROS_INFO("[local_sensing] Mock map received with %lu points.", full_cloud->points.size());
    }
}

void pubLocalMap() {
    if (!has_map || !has_odom) return;

    pcl::PointCloud<pcl::PointXYZ> localMap;
    pcl::PointXYZ center(current_position.x(), current_position.y(), current_position.z());

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    double sensing_radius = local_range.norm() / 2.0; // 简单近似

    if (kdtree.radiusSearch(center, sensing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            localMap.points.push_back(full_cloud->points[pointIdxRadiusSearch[i]]);
        }

        localMap.width = localMap.points.size();
        localMap.height = 1;
        localMap.is_dense = true;

        sensor_msgs::PointCloud2 localMapMsg;
        pcl::toROSMsg(localMap, localMapMsg);
        localMapMsg.header.frame_id = "world";
        localMapMsg.header.stamp = ros::Time::now();

        local_map_pub.publish(localMapMsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_sensing_node");
    ros::NodeHandle nh("~");

    // 获取地图参数
    nh.param("sdf_map/resolution", resolution, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/local_update_range_x", local_range(0), -1.0);
    nh.param("sdf_map/local_update_range_y", local_range(1), -1.0);
    nh.param("sdf_map/local_update_range_z", local_range(2), -1.0);

    // 订阅 /sim/odom 和 /mock_map
    ros::Subscriber odom_sub = nh.subscribe("/sim/odom", 1, odomCallback);
    ros::Subscriber map_sub = nh.subscribe("/mock_map", 1, mockMapCallback);

    // 发布 /sim/local_map
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/sim/local_map", 1);

    ROS_INFO("[local_sensing] Node initialized. Subscribing to /sim/odom and /mock_map, publishing to /sim/local_map");

    ros::Rate rate(10.0);

    while (ros::ok()) {
        ros::spinOnce();
        pubLocalMap();
        rate.sleep();
    }

    return 0;
}
