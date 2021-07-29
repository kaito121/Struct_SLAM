#classfication elements
#
#各クラスタデータ
std_msgs/Int32 size #クラスタに含めたマップセルの数
std_msgs/Int32 dens #点の数
geometry_msgs/Point gc #gravity center : 重心
std_msgs/Int32[] index #配列番号 -> マップセル位置
geometry_msgs/Point[] pt #クラスタ内のマップセル位置データ