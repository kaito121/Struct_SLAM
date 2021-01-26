#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(10000);

  float f = 0.0;
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points,  line_list;
    points.header.frame_id =  line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "lines";
    points.action =  line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
    line_list.id = 2;

//マーカーの種類
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

// 大きさを決める
    // POINTS マーカは、幅と高さをそれぞれ x と y スケールを使用しています。
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST マーカは、線幅に対してスケールの x 成分のみを使用します。
    line_list.scale.x = 0.1;


//ここで色をつける
    // Points are green(点は緑)
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line list is red(ラインリストは赤)
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // 点と線の頂点を作成する
    //ここのfor文のマックスを変数にすると個数を増やせる
    for (uint32_t i = 0; i < 5; ++i)
    {
      //float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      //float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      float y=1;
      float z=1;

      geometry_msgs::Point p;
      p.x = 1;
      p.y = y;
      p.z = z;

      points.points.push_back(p);

      // ラインリストは、各ラインに2点必要
      line_list.points.push_back(p);
      p.z += 1.0;//ここで線の長さを変えられる
      p.x +=f;
      p.y +=f;
      line_list.points.push_back(p);
    }
// %EndTag(HELIX)%

    marker_pub.publish(points);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}