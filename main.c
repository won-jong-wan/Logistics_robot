#include <stdio.h>
#include <stdlib.h>
#include "linkedList.h"

/*
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
}*/

typedef enum _RobMovM{
    STOP,
    MOVEX,
    MOVEY,
}RobMovM;

typedef enum _RobGraM{
    UNGRAB,
    GRAB,
}RobGraM;

typedef enum _GrabLev{
    ROBOT,
    FIRST,
    SECOND,
    THRID,
}GrabLev;

typedef enum _CellT{
    USABLE,
    BLOCKED,
    TARGET,
}CellT;

typedef struct _Cell{
    unsigned int x;
    unsigned int y;
    LinkedList* node;
    CellT state;        
}Cell;

typedef struct _Grid{
    Cell grid[4][3];
}

typedef struct _Robot{
    unsigned int rx;
    unsigned int ry;
    LinkedList* rPass;
    RobMovM rMovM;
    RobGraM rGrabM;
    GrabLev rGrabL;
    Cell rPostion;
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