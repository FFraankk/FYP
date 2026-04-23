// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <livox_ros_driver2/CustomMsg.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

// using namespace sensor_msgs;
// using namespace livox_ros_driver2;
// using namespace message_filters;

// class SyncNode {
// public:
//     SyncNode(ros::NodeHandle& nh) : sync(MySyncPolicy(10), image_sub, lidar_sub) {
//         // Get parameters from the parameter server
//         std::string image_topic, lidar_topic, synced_image_topic, synced_lidar_topic;
//         double publish_rate;

//         nh.getParam("image_topic", image_topic);
//         nh.getParam("lidar_topic", lidar_topic);
//         nh.getParam("synced_image_topic", synced_image_topic);
//         nh.getParam("synced_lidar_topic", synced_lidar_topic);
//         nh.getParam("publish_rate", publish_rate);

//         image_sub.subscribe(nh, image_topic, 10);
//         lidar_sub.subscribe(nh, lidar_topic, 10);

//         image_pub = nh.advertise<Image>(synced_image_topic, 10);
//         lidar_pub = nh.advertise<CustomMsg>(synced_lidar_topic, 10);

//         sync.registerCallback(boost::bind(&SyncNode::callback, this, _1, _2));

//         // Set up a timer to publish synchronized messages at the specified rate
//         ros::Duration period(1.0 / publish_rate); // Convert Hz to seconds
//         timer = nh.createTimer(period, &SyncNode::timerCallback, this);
//     }

// private:
//     ros::NodeHandle nh;
//     Subscriber<Image> image_sub;
//     Subscriber<CustomMsg> lidar_sub;
//     typedef sync_policies::ApproximateTime<Image, CustomMsg> MySyncPolicy;
//     Synchronizer<MySyncPolicy> sync;
//     ros::Publisher image_pub;
//     ros::Publisher lidar_pub;
//     ros::Timer timer;

//     Image last_image_msg;
//     CustomMsg last_lidar_msg;
//     bool new_image_received = false;
//     bool new_lidar_received = false;

//     void callback(const ImageConstPtr& img_msg, const CustomMsgConstPtr& lidar_msg) {
//         // Store the latest synchronized messages
//         last_image_msg = *img_msg;
//         last_lidar_msg = *lidar_msg;
//         new_image_received = true;
//         new_lidar_received = true;
//     }

//     void timerCallback(const ros::TimerEvent& event) {
//         if (new_image_received && new_lidar_received) {
//             image_pub.publish(last_image_msg);
//             lidar_pub.publish(last_lidar_msg);
//             new_image_received = false;
//             new_lidar_received = false;
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "sync_node");
//     ros::NodeHandle nh("~");

//     SyncNode node(nh);

//     ros::spin();

//     return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <livox_ros_driver2/CustomMsg.h>

#include <deque>
#include <mutex>
#include <cmath>

class LidarAnchorSync
{
public:
    LidarAnchorSync(ros::NodeHandle& nh)
    {
        nh.param<std::string>("image_topic", image_topic_, "/camera/color/image_raw");
        nh.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");

        // Renamed aligned output topics
        nh.param<std::string>("aligned_image_topic", aligned_image_topic_, "/camera/image_aligned");
        nh.param<std::string>("aligned_lidar_topic", aligned_lidar_topic_, "/livox/lidar_aligned");

        nh.param<double>("time_threshold", time_threshold_, 0.05); // 50 ms

        image_sub_ = nh.subscribe(image_topic_, 100, &LidarAnchorSync::imageCallback, this);
        lidar_sub_ = nh.subscribe(lidar_topic_, 10, &LidarAnchorSync::lidarCallback, this);

        image_pub_ = nh.advertise<sensor_msgs::Image>(aligned_image_topic_, 10);
        lidar_pub_ = nh.advertise<livox_ros_driver2::CustomMsg>(aligned_lidar_topic_, 10);

        ROS_INFO("[LiDAR-Anchor Sync] Publishing:");
        ROS_INFO("  LiDAR %s", aligned_lidar_topic_.c_str());
        ROS_INFO("  Image %s", aligned_image_topic_.c_str());
    }

private:
    ros::Subscriber image_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher image_pub_;
    ros::Publisher lidar_pub_;

    std::string image_topic_;
    std::string lidar_topic_;
    std::string aligned_image_topic_;
    std::string aligned_lidar_topic_;

    std::deque<sensor_msgs::ImageConstPtr> image_buffer_;
    std::mutex buffer_mutex_;

    double time_threshold_;

    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        image_buffer_.push_back(img_msg);

        if (image_buffer_.size() > 200)
            image_buffer_.pop_front();
    }

    void lidarCallback(const livox_ros_driver2::CustomMsgConstPtr& lidar_msg)
    {
        sensor_msgs::ImageConstPtr best_image = nullptr;
        double min_dt = 1e9;
        ros::Time lidar_time = lidar_msg->header.stamp;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            for (const auto& img : image_buffer_)
            {
                double dt = fabs((img->header.stamp - lidar_time).toSec());
                if (dt < min_dt)
                {
                    min_dt = dt;
                    best_image = img;
                }
            }

            while (!image_buffer_.empty() &&
                   image_buffer_.front()->header.stamp < lidar_time - ros::Duration(0.2))
            {
                image_buffer_.pop_front();
            }
        }

        if (best_image && min_dt < time_threshold_)
        {
            image_pub_.publish(*best_image);
            lidar_pub_.publish(*lidar_msg);
        }
        else
        {
            lidar_pub_.publish(*lidar_msg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_anchor_sync");
    ros::NodeHandle nh("~");

    LidarAnchorSync node(nh);
    ros::spin();
    return 0;
}
