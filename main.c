#include <stdio.h>
#include <stdlib.h>

typedef struct _LinkedList{
    unsigned int index;
    struct _LinkedList* headPtr;
    struct _LinkedList* tailPtr;
    void* detail;
    char* tag;
}LinkedList;

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

int main(){

    printf("%d\n",sizeof(LinkedList));

    LinkedList* LL= initLL();
    int* data = (int*) malloc(sizeof(int));
    *data = 42;

    llpb(LL, data);

    printf("%d\n",*(int*)(LL->tailPtr->detail));

    return 0;
}