#ifndef LINKEDLIST_H
#define LINKEDLIST_H

typedef struct _LinkedList{
    unsigned int index;
    struct _LinkedList* headPtr;
    struct _LinkedList* tailPtr;
    void* detail;
    char* tag;
}LinkedList;

LinkedList* initLL();

void llpb(LinkedList* LL, void* data);

#endif