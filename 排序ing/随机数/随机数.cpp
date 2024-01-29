#include<stdio.h>
int jb(int arr[11])
{
	
	/*for (int i = 0; i <= 10; i++)
	{
		scanf_s("%d", &arr[i]);
	}*/
	for (int i = 0; i <= 10; i++)
	{
		for (int j = 0; j <= 9 - i; j++)
		{
			if (arr[j] < arr[j + 1])//怎么改了<就报错了？？？？？？？？？？？
			{
				int temp = arr[j + 1];
				arr[j + 1] = arr[j];
				arr[j] = temp;
			}

		}
		/*printf("%p\n", &arr[i]);*/

	}
	/*	for (int i = 0; i < 11; i++)
	{
		printf("\n%d\t", arr[i]);
	}*/

	return arr[11];

}
int main()
{
	int arr[] = { 1,4,6,5,6453,73,456,3456,3456,1234,1234 };
	
	int result = jb(arr);
	for (int i = 0; i <= 10; i++) {
		printf("%d\n", arr[i]);
	}
	return	0;

}