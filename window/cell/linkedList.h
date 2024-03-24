#ifndef LINKEDLIST_H
#define LINKEDLIST_H

typedef struct _LinkedList{
    unsigned int index;
    unsigned int length;
    struct _LinkedList* headPtr;
    struct _LinkedList* tailPtr;
    void* data;
    char* tag;
}LinkedList;

typedef struct _Node{
    
}Node;

LinkedList* initLL();

LinkedList* lltail(LinkedList* LL);

void llpb(LinkedList* LL, void* data);
void lldel(LinkedList* LL);

#endif