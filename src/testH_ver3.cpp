#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

    int i,j,t,p,n;
    double theta[10]={10.25,20,30,30,10,0.5,10.5,10,10,50};
    double line[4][10]={{1,2,3,4,5,6,7,8,9,10},{11,12,13,14,15,16,17,18,19,20},{21,22,23,24,25,26,27,28,29,30},{31,32,33,34,35,36,37,38,39,40}};
    double  depth[2][10]={{0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0},{1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0}};
    double theta1[10]={10,11,12,13,20,21,22,23,24,30};
    double A[100][100],B,tmp;
    double P[10][100][2][4];
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
          for(n=0;n<10;n++){
            std::cout <<"theta1["<<n<<"]= "<<theta1[n]<< std::endl;
            std::cout <<"A["<<j<<"]["<<0<<"]= "<<A[j][0]<< std::endl;
            std::cout <<"A["<<j<<"]["<<c[j]-1<<"]= "<<A[j][c[j]]<< std::endl;
            if(theta1[n]>= A[j][0] && theta1[n]<= A[j][c[j]-1]){
              P[j][n][0][0]=line[0][n];   //pt1 x座標
              P[j][n][0][1]=line[1][n];   //pt1 y座標
              P[j][n][0][2]=depth[0][n];  //pt1 z座標(三次元距離)  
              P[j][n][0][3]=theta1[n];    //pt1 角度

              P[j][n][1][0]=line[2][n];   //pt2 x座標
              P[j][n][1][1]=line[3][n];   //pt2 y座標
              P[j][n][1][2]=depth[1][n];  //pt2 z座標(三次元距離)
              P[j][n][1][3]=theta1[n];    //pt2 角度

              std::cout <<"pt1 角度  P["<<j<<"]["<<n<<"][0][3]= "<<P[j][n][0][3]<< std::endl;
              std::cout <<"pt1 x座標 P["<<j<<"]["<<n<<"][0][0]= "<<P[j][n][0][0]<< std::endl;
              std::cout <<"pt1 y座標 P["<<j<<"]["<<n<<"][0][1]= "<<P[j][n][0][1]<< std::endl;
              std::cout <<"pt1 z座標 P["<<j<<"]["<<n<<"][0][2]= "<<P[j][n][0][2]<< std::endl;
        
              std::cout <<"pt2 角度  P["<<j<<"]["<<n<<"][1][3]= "<<P[j][n][1][3]<< std::endl;
              std::cout <<"pt2 x座標 P["<<j<<"]["<<n<<"][1][0]= "<<P[j][n][1][0]<< std::endl;
              std::cout <<"pt2 y座標 P["<<j<<"]["<<n<<"][1][1]= "<<P[j][n][1][1]<< std::endl;
              std::cout <<"pt2 z座標 P["<<j<<"]["<<n<<"][1][2]= "<<P[j][n][1][2]<< std::endl;
              
            
          
      }}}
  

    

    return 0;
}