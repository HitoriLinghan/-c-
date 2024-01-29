#include <stdio.h>
 
extern int calculate(int n, long long* pa, long long* pm)
{
    int ret = 1;
 
    if( (1 <= n) && (n <= 20) )
    {
        int i = 0;
 
        *pa = 0;
        *pm = 1;
 
        for(i=1; i<=n; i++)
        {
            *pa = *pa + i;
            *pm = *pm * i;
        }
    }
    else
    {
        ret = 0;
    }
 
    return ret;
}
 
extern int main()
{
    long long ar = 0;
    long long mr = 0;
 
    if( calculate(5, &ar, &mr) )
        printf("ar = %lld, mr = %lld\n", ar, mr);
 
    return 0;
}