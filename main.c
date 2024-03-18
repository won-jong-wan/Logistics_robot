#include <stdio.h>
#include <stdlib.h>
#include "robot.h"

int main(int, char**){
    printf("%d\n",sizeof(LinkedList));

    LinkedList* LL= initLL();
    int* data = (int*) malloc(sizeof(int));
    *data = 42;

    llpb(LL, data);

    printf("%d\n",*(int*)(LL->tailPtr->detail));
}
