#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// HSV -> RGB変換
void hsv2rgb(float h, float s, float v, int &_r, int &_g, int &_b) {

    float r = static_cast<float>(v);
    float g = static_cast<float>(v);
    float b = static_cast<float>(v);
    if (s > 0.0f) {
        h *= 6.0f;
        const int i = (int) h;
        const float f = h - (float) i;
        switch (i) {
            default:
            case 0:
                g *= 1 - s * (1 - f);
                b *= 1 - s;
                break;
            case 1:
                r *= 1 - s * f;
                b *= 1 - s;
                break;
            case 2:
                r *= 1 - s;
                b *= 1 - s * (1 - f);
                break;
            case 3:
                r *= 1 - s;
                g *= 1 - s * f;
                break;
            case 4:
                r *= 1 - s * (1 - f);
                g *= 1 - s;
                break;
            case 5:
                g *= 1 - s;
                b *= 1 - s * f;
                break;
        }
    }
    _r = static_cast<int>(r * 255);
    _g = static_cast<int>(g * 255);
    _b = static_cast<int>(b * 255);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_point_publisher");
    ros::NodeHandle nh, pnh("~");

    ros::Rate rate(1.0);
    std::string topic_name = "points";
    float range = 3.0;
    int number_of_points = 10000;

    // pcl:PointCloud型のPublisher
    // 実際にTopicとして流れるのは sensor_msgs::PointCloud2 になる
    // テンプレートの中身を変えればXYZIとかXYZとかに変更可能
    ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(topic_name.c_str(),10);

    while (ros::ok()) {

        // ダミー点群の準備
        pcl::PointCloud<pcl::PointXYZRGB> dummy_cloud;

        for (int i = 0; i < number_of_points; i++) {

            pcl::PointXYZRGB new_point;
            new_point.x = range - (rand() * range * 2) / RAND_MAX;
            new_point.y = range - (rand() * range * 2) / RAND_MAX;
            new_point.z = range - (rand() * range * 2) / RAND_MAX;
            float distance = std::sqrt(new_point.x * new_point.x + new_point.y * new_point.y + new_point.z * new_point.z);

            int r, g, b;
            hsv2rgb(std::fmod(distance / 3.0, 1.0), 1.0, 1.0, r, g, b);
            new_point.r = r;
            new_point.g = g;
            new_point.b = b;

            // pcl::PointCloudはpush_backで足せば良いだけなので楽ちん
            dummy_cloud.points.push_back(new_point);
        }

        auto msg = dummy_cloud.makeShared();

        // ヘッダ情報はココでつめる
        msg->header.frame_id = "lidar";
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        pc_pub.publish(msg);

        rate.sleep();
    }

    return 0;
}