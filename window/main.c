#include <stdio.h>
#include <stdlib.h>
#include "robot.h"

int main(){
    // printf("%lu\n",sizeof(LinkedList));

    // int a = 20;
    // LinkedList* LL= llinit(&a);
    // int* data = (int*) malloc(sizeof(int));
    // *data = 42;

    // llpb(LL, data);
    // llpb(LL, &a);

    // printf("%d\n", *(int*)llread(LL, 1));

    // lldel(LL);

    Stack* sta = stainit(STACK_N);
    Box* box = boxinit(0, EMPTY, 0);
    Box* box2 = boxinit(1, EMPTY, 0);

    stapush(sta, box);
    stapush(sta, box2);
    printf("%d\n", stapop(sta)->id);
    printf("%d\n", stapop(sta)->id);

    stadel(sta);

    return 0;
}
