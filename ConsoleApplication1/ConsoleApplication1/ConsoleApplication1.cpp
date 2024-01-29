#include <stdio.h>
#include <math.h>
int prime(int num)
{

    for (int i=2;i<sqrt(num);i++)
    {
        if (num % i == 0)
        {
            return 0;
        }
    
    }
    return 1;
}
int main()
{
    int num = 50;
 
    for (int J = 3; J < num / 2; J++)
    {
        
        if (prime(J)&&prime(num-1))
        {
            
            printf("%d\t+%d", J, num-J);
            break;
        }
        
    };

    return 0;
}