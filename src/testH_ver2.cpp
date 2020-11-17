#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

    int i,j,t,p;
    double theta[10]={10.25,20,30,30,10,0.5,10.5,10,10,50};
    double A[100][100],B,tmp;
    int c[10];

    /* 数値を昇順にソート */
  for (i=0; i<10; ++i) {
    for (j=i+1;j<10; ++j) {
      if (theta[i] > theta[j]) {
        tmp =  theta[i];
        theta[i] = theta[j];
        theta[j] = tmp;
      }
    }
  }
    /* 昇順ソートした数値を出力 */
  printf("昇順ソートした数値\n");
  for (i=0; i<10; ++i){ printf("%f\n", theta[i]); }
  
  A[0][0]=theta[0];
  B=theta[0];
  c[0]=1;
  t=0,j=1;p=0;

  for(i=0;i<9;i++){
    //前の番号と同じ数値
      if(B==theta[i+1]){
          A[t][j]=theta[i+1];//代入
          j=j+1;//配列カウント
          c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
      }
    //前の番号と異なる数値
      else{
        if(theta[i+1]-B>0.5){//前の角度との差が0.5より大きい
          t=t+1,j=0;//配列繰り上がり、ｊリセット
          A[t][j]=theta[i+1];//代入
          B=theta[i+1];//基準の更新
          j=j+1;//配列カウント
          c[t]=1;}//配列要素数初期値

        if(theta[i+1]-B<=0.5){//前の角度との差が0.5以下
          A[t][j]=theta[i+1];//代入
          B=theta[i+1];//基準の更新
          j=j+1;//配列カウント
          c[t]=c[t]+1;//要素数（同じ数値は何個あるか)
      }


      } 
  }
  
  for(j=0;j<=t;j++){
    std::cout <<"C["<<j<<"]= "<<c[j]<< std::endl;
      for(i=0;i<c[j];i++){  
          std::cout <<"A["<<j<<"]["<<i<<"]= "<<A[j][i]<< std::endl;
      }
  }

    

    return 0;
}