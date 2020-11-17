#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

    int i,j,tmp;
    int theta[5]={10,20,30,30,10,20};
    int A[ ];
    t=1;

    /* 数値を昇順にソート */
  for (i=0; i<5; ++i) {
    for (j=i+1; j<5; ++j) {
      if (theta[i] > theta[j]) {
        tmp =  theta[i];
        theta[i] = theta[j];
        theta[j] = tmp;
      }
    }
  }

  /* 昇順ソートした数値を出力 */
  printf("昇順ソートした数値\n");
  for (i=0; i<5; ++i)
    printf("%d\n", theta[i]);
}
    return 0;
}
