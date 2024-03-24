#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#include <stdlib.h>
#include <stdio.h>

typedef struct _LinkedList{
    unsigned int length;
    struct _Node* headPtr;
}LinkedList;

typedef struct _Node{
    void* data;
    struct _Node* headPtr;
    struct _Node* tailPtr;
}Node;

Node* nodinit(void* data, Node* headPtr, Node* tailPtr);
LinkedList* llinit(void* data);
void lldel(LinkedList* ll);

Node* llfind(LinkedList* ll, unsigned int index);
void llp(LinkedList* ll, void* data, unsigned int headIndex);
//void lld(LinkedList* ll, unsigned int index);

Node* lltail(LinkedList* ll);
void llpb(LinkedList* ll, void* data);
void* llread(LinkedList* ll, unsigned int index);

#endif