#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS マーカは、幅と高さをそれぞれ x と y スケールを使用しています。
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST マーカは、線幅に対してスケールの x 成分のみを使用します。
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green(点は緑)
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue(ラインストリップは青)
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red(ラインリストは赤)
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(HELIX)%
    // 点と線の頂点を作成する
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // ラインリストは、各ラインに2点必要
      line_list.points.push_back(p);
      p.z += 10.0;
      line_list.points.push_back(p);
    }
// %EndTag(HELIX)%

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}