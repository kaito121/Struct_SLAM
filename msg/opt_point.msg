#ROSヘッダ
std_msgs/Header header
#マップサイズ
#実測サイズ
std_msgs/Float32 width#
std_msgs/Float32 height#
std_msgs/Float32 res#
#ピクセルサイズ
std_msgs/Int32 widthInt
std_msgs/Int32 heightInt
#マップの中心点（センサ座標）
geometry_msgs/Point cp
#オプティカルフローデータ(緑の点のやつ）
std_msgs/Int32[] size 
std_msgs/Int32[] index
geometry_msgs/Point[] pt_curr
geometry_msgs/Point[] pt_prev
