#include <iostream>

//メイン関数
int main(int argc,char **argv)
{
    int iIdx= 0;

    for(iIdx = 0; iIdx< argc; iIdx++)
    {
        //printf("[%d]%s\n", iIdx, argv[iIdx]);
        std::cout <<"["<<iIdx<<"]"<<argv[iIdx]<< std::endl;
    }
 return 0;
}