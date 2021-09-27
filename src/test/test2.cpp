#include <iostream>

//メイン関数
int main(int argc,char **argv)
{
    int iIdx= 0;

    std::cout << "TEST2" << std::endl;
    for(iIdx = 0; iIdx< argc; iIdx++)
    {
        //printf("[%d]%s\n", iIdx, argv[iIdx]);
        std::cout <<"["<<iIdx<<"]"<<argv[iIdx]<< std::endl;
    }
 return 0;
}
