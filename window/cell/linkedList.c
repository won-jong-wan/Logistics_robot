#include <stdlib.h>
#include "linkedList.h"

LinkedList* initLL(){
    LinkedList* LL = malloc(sizeof(LinkedList));
    LL->index = 0;
    LL->length = 1;
    LL->headPtr = 0;
    LL->tailPtr = 0;
    LL->data = 0;
    LL->tag = 0;

    return LL;
}

LinkedList *lltail(LinkedList *LL)
{
    int i;
    LinkedList* tmp = LL->tailPtr;
    for(i=0; i<LL->length-1; i++){
        tmp = tmp->tailPtr;
    }
    return nullptr;
}

void llpb(LinkedList* LL, void* data){
    LinkedList* tail = malloc(sizeof(LinkedList));
    tail->index = LL->index+1;
    tail->headPtr = LL;
    tail->tailPtr = 0;
    tail->data = data;

    LL->tailPtr = tail;
    LL->length = LL->length+1;
}

void lldelA(LinkedList *LL){
    int i;
    LinkedList* tmp;
}
