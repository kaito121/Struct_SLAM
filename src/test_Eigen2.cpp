//20210429 関数内のGenIとmain文内のMの値が異なる理由がわからない。
//これがわかれば溶けるかもしれない
//→計算式が異なるから
#include "ros/ros.h"
#include <iostream>
#include <Eigen/Core> // 基本演算のみ
#include <Eigen/Eigen> // 固有値
using namespace Eigen;
using namespace std;

typedef Matrix<double,3,1> Vector3d;
typedef Matrix<double,3,3> Matrix3d;

 // n × n 対称行列 M のランク r の一般逆行列
 template<class MatT>
 MatT GenInv(const MatT& M, int r)//多分TとMは同値
 {
 if (M.cols() != M.rows()) abort(); // 正方行列でない

 int n = M.cols(); // サイズ
  cout << "12M00000:\n" << M << endl;//main文のfor文後のMと同値

 // 自己随伴行列 (実数行列では対称放列) の固有値のソルバー
 SelfAdjointEigenSolver<MatT> ES(M);
 cout << "13ES(M)=\n" << ES.eigenvectors() << endl;//ES(T)と同値

 if (ES.info() != Success) abort(); // 失敗したら終了
 
 // 一般逆行列用の行列を零行列に初期化
 MatT GenI = MatT::Zero();
 cout << "14GenI=\n" << GenI << endl;//GenIの要素がすべてゼロ

 // 一般逆行列の計算
 for (int i = n - r; i < n; i++)
 {
 cout << "15for文の中のGenI1=\n" << GenI << endl;//for文開始時は要素がすべて0
 GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() / ES.eigenvalues()(i);
 cout << "GenI += ES.eigenvectors().col("<<i<<") * ES.eigenvectors().col("<<i<<").transpose() / ES.eigenvalues()("<<i<<");\n" << endl;
 cout << GenI <<"+= "<<ES.eigenvectors().col(i)<<" * "<<ES.eigenvectors().col(i).transpose()<<" / "<<ES.eigenvalues()(i)<<"\n" << endl;

 cout << "16for文の中のGenI2=\n" << GenI << endl;//fmain文のfor内のMと値が異なる
 cout << "17for文の中のES(固有ベクトル)=\n" << ES.eigenvectors() << endl;//main文のfor内のESと同値(3☓3の行列)
 cout << "18for文の中のES.col(固有値に対する固有ベクトル)("<<i<<")=\n" << ES.eigenvectors().col(i) << endl;//3☓1の行列_col(i)は列を表してる
 cout << "19for文の中のES(固有値)("<<i<<")=\n" << ES.eigenvalues()(i) << endl;//ESの１要素i=1の時は(1,1),i=2の時は(1,2)
 }
 cout << "20Original␣matrix␣GenI␣=␣\n" << GenI << endl;

 // 戻り
 return GenI;
 }

 int main()
 {
 Vector3d v; // 初期化用ベクトル
 Matrix3d M; // 対象とする行列
 Matrix3d Mi; // 一般逆行列

 // ランク 5 の対称行列作成
 //Matrix3d T, T2;
 //T = Matrix3d::Random(); // ランダムな要素の行列を生成
 Matrix3d T,T2;
   T << 1, 7, 3,
        7, 4, -5,
        3, -5, 6;
 cout << "1Here is the matrix T:\n" << T << endl;
 T2 = (T + T.transpose()) / 2.0; // 対称行列化
 T = T2 * T2.transpose(); // 正値化
 cout << "2正規化T:\n" << T << endl;
 cout << "3M0:\n" << M << endl;//この段階でMには値が入っていない

 SelfAdjointEigenSolver<Matrix3d> ES(T);
 M = Matrix3d::Zero(); // 零行列に初期化
 cout << "4Here is the matrix M:\n" << M << endl;//Mにはゼロが入ってる
 cout << "5ES(T)=\n" << ES.eigenvectors() << endl;//Tの値を変えるとESも変動するからTの影響を受けている(固有ベクトル)

 for (int i = 1; i < 3; i++)
 {
 cout << "6for文の中のM1=\n" << M << endl;  //for文開始時は要素がすべて0
 M += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() * ES.eigenvalues()(i);//MとGenIとで計算式が若干異なる
 cout << "M += ES.eigenvectors().col("<<i<<") * ES.eigenvectors().col("<<i<<").transpose() * ES.eigenvalues()("<<i<<");\n" << endl;
 cout << M <<"+= "<<ES.eigenvectors().col(i)<<" * "<<ES.eigenvectors().col(i).transpose()<<" * "<<ES.eigenvalues()(i)<<"\n" << endl;
 cout << "7for文の中のM2=\n" << M << endl;//M=M+ESになってる
 cout << "8for文の中のES(固有ベクトル)=\n" << ES.eigenvectors() << endl;//3☓3の行列_値は変わらない固定値_GenIのESと同値
 cout << "9for文の中のES.col(固有値に対する固有ベクトル)("<<i<<")=\n" << ES.eigenvectors().col(i) << endl;//3☓1の行列_col(i)は列を表してる
 cout << "10for文の中のES(固有値)("<<i<<")=\n" << ES.eigenvalues()(i) << endl;//ESの１要素i=1の時は(1,1),i=2の時は(1,2)
 }
 cout << "11Original␣matrix␣M␣=␣\n" << M << endl;//12のM0000と同値

 // ランク 5 の一般逆行列
 Mi = GenInv(M, 2);
 cout << "21Generalized␣inverse␣of␣M␣with␣rank␣5␣=␣\n" << Mi << endl;//for文内のGenIと同値
  
}

/*#include "ros/ros.h"
#include <iostream>
#include <Eigen/Core> // 基本演算のみ
#include <Eigen/Eigen> // 固有値
using namespace Eigen;
using namespace std;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

 // n × n 対称行列 M のランク r の一般逆行列
 template<class MatT>
 MatT GenInv(const MatT& M, int r)
 {
 if (M.cols() != M.rows()) abort(); // 正方行列でない

 int n = M.cols(); // サイズ

 // 自己随伴行列 (実数行列では対称放列) の固有値のソルバー
 SelfAdjointEigenSolver<MatT> ES(M);

 if (ES.info() != Success) abort(); // 失敗したら終了

 // 一般逆行列用の行列を零行列に初期化
 MatT GenI = MatT::Zero();

 // 一般逆行列の計算
 for (int i = n - r; i < n; i++)
 {
 GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() / ES.eigenvalues()(i);
 }

 // 戻り
 return GenI;
 }

 int main()
 {
 Vector6d v; // 初期化用ベクトル
 Matrix6d M; // 対象とする行列
 Matrix6d Mi; // 一般逆行列

 // ランク 5 の対称行列作成
 Matrix6d T, T2;
 T = Matrix6d::Random(); // ランダムな要素の行列を生成
 T2 = (T + T.transpose()) / 2.0; // 対称行列化
 T = T2 * T2.transpose(); // 正値化

 SelfAdjointEigenSolver<Matrix6d> ES(T);
 M = Matrix6d::Zero(); // 零行列に初期化
 for (int i = 1; i < 6; i++)
 {
 M += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() * ES.eigenvalues()(i);
 }
 cout << "Original␣matrix␣M␣=␣\n" << M << endl;

 // ランク 5 の一般逆行列
 Mi = GenInv(M, 5);
 cout << "Generalized␣inverse␣of␣M␣with␣rank␣5␣=␣\n" << Mi << endl;
  



}*/