#include <stdio.h>
int sum;
int a;
int b;
int main()
{
	printf("num:");
	scanf_s("%d", &a);
	printf("\nnum:");
	scanf_s( "%d", &b);
	sum = a + b;
	printf("result is:%d\n", sum);
	return 0;
}