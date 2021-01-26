#mask image data
#
#ROSヘッダ
std_msgs/Header header
#画像サイズ
std_msgs/Int32 width#
std_msgs/Int32 height#
#マップサイズ
#実測サイズ
std_msgs/Float32 mapWidth
std_msgs/Float32 mapHeight
std_msgs/Float32 mapRes
#ピクセルサイズ
std_msgs/Int32 mapWidthInt
std_msgs/Int32 mapHeightInt
#インデックスデータ
std_msgs/Int32[] index
#マップデータ#method　cv　point2iってやつがpt
#マップデータっていうのが画像座標上の距離とか
geometry_msgs/Point[] pt
