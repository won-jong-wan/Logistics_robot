#include <stdlib.h>
#include "linkedList.h"

LinkedList* initLL(){
    LinkedList* LL = malloc(sizeof(LinkedList));
    LL->index = 0;
    LL->headPtr = 0;
    LL->tailPtr = 0;
    LL->detail = 0;
    LL->tag = 0;

    return LL;
}

void llpb(LinkedList* LL, void* data){
    LinkedList* tail = malloc(sizeof(LinkedList));
    tail->index = LL->index+1;
    tail->headPtr = LL;
    tail->tailPtr = 0;
    tail->detail = data;

    LL->tailPtr = tail;;
}