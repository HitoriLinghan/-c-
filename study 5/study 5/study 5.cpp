#include <stdio.h>
#define c 10
int main()
{
	printf("please enter your need:");
	int a;
	scanf_s("%d", &a);
	short d = 20;
	long g = 30;
	long long t = 40;
	printf("%d\n", a);
	printf("%hd\n",d);
	printf("%ld\n", t);
	int b = c * a;
	printf("price is:%d",b);
	return	0;

}
