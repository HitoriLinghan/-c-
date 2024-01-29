#include <stdio.h>
#define pi 3.14159
int main(void)
{
	float r;
	float s = pi * r * r;
	float c = 2 * pi * r;
	scanf("%f", &r);
	printf("please enter your:%s", r);
	printf("面积事:%f\n", s);
	printf("周长事:%f\n", c);
	//%f 表示的保留6位小数
	//%.2f\n 两位小数
	return 0;
} 
//标识符
/*1.不能使用系统关键字
2.子母下划线数字允许使用
3.不能数字开头
4.区分大小写*/


