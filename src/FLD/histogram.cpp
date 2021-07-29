#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#define DATA_NUM 4  // データ数
//#define DATA_NUM 56  // データ数
#define MAX_DATA 6.0  // データの最大値
//#define MAX_DATA 2  // データの最大値


#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define SIZE_OF_ARRAY(array)    (sizeof(array)/sizeof(array[0]))

int compareInt(const void* a, const void* b)
{
    int aNum = *(int*)a;
    int bNum = *(int*)b;

    return aNum - bNum;
}

/*
    最頻値を求める

    引数
        array: 対象の配列
        size:  array の要素数
    戻り値
        最頻値
*/
int get_mode(const int* array, size_t size)
{
    assert(array != NULL);
    assert(size > 0);

    // [!] 配列が昇順にソートされていることを前提にしている

    int mode;              // これまでに調べた中での最頻値
    int count_max = 0;     // これまでに調べた登場回数の中で最大のもの

    for (size_t i = 0; i < size; ) {

        int value = array[i];
        int count = 1;
    
        // ソートされているのだから、
        // 同じ値があるとすれば、後続に連続して並んでいるはず。
        // その回数をカウントする。
        for (i = i + 1; i < size; ++i) {
            if (value == array[i]) {
                ++count;
            }
            else {
                // 違う値が登場したら終わり
                break;
            }
        }

        // これまでの最大の登場回数よりも多かったら、更新する
        if (count_max < count) {
            count_max = count;
            mode = value;
        }
    }

    return mode;
}

int main(void)
{
    int array[] = { 7, 2, 6, 2, 2, 6, 1, 3};

    // 配列を昇順にソートする
    qsort(array, SIZE_OF_ARRAY(array), sizeof(int), compareInt);

    printf( "%d\n", get_mode(array, SIZE_OF_ARRAY(array)));

    return 0;
}