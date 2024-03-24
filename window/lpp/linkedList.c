#include "linkedList.h"

Node *nodinit(void *data, Node *headPtr, Node *tailPtr)
{
    Node *nodPtr = (Node *)malloc(sizeof(Node));

    nodPtr->data = data;
    nodPtr->headPtr = 0;
    nodPtr->tailPtr = 0;
    return nodPtr;
}

LinkedList *llinit(void *data)
{
    LinkedList *ll;

    Node *nodPtr = nodinit(data, 0, 0);

    ll->headPtr = nodPtr;
    ll->length = 1;

    return ll;
}

void lldel(LinkedList *ll)
{
    int i;

    Node *tmp = ll->headPtr;
    for (i = 0; i < ll->length - 1; i++)
    {
        tmp = tmp->tailPtr;
        free(tmp->headPtr);
        printf("Node ptr free\n");
    }
    free(tmp);
    free(ll);

    printf("linkedList ptr free\n");
}

Node *llfind(LinkedList *ll, unsigned int index)
{
    if (index + 1 > ll->length)
    {
        printf("warning index over ll length! index: %d >= length: %d\n", index, ll->length);
        return 0;
    }

    int i;

    Node *tmp = ll->headPtr;
    for (i = 0; i < index; i++)
    {
        tmp = tmp->tailPtr;
    }
    return tmp;
}

void llp(LinkedList *ll, void *data, unsigned int headIndex)
{
    Node *head = llfind(ll, headIndex);
    Node *tail = 0;
    if(headIndex+1 < ll->length){
        Node *tail = llfind(ll, headIndex + 1);
    }

    Node *new = nodinit(data, head, tail);

    head->tailPtr =new;
    
    if(tail != 0){
        tail->headPtr = new;
    }

    ll->length = ll->length + 1;
}

//for llstack
/*void lld(LinkedList *ll, unsigned int index)
{
    Node* target = llfind(ll, index);

    if(index+1 < ll->length ){
        target->tailPtr->headPtr = target->headPtr;
    }
    if(index+1 <= ll->length){
        target->headPtr->tailPtr = target->tailPtr;
        ll->length = ll->length -1;
        free(target);
    }

    // target->tailPtr->headPtr = target->headPtr;
    // target->headPtr->tailPtr = target->tailPtr;
    // ll->length = ll->length -1;
    // free(target);
}*/

Node *lltail(LinkedList *ll)
{
    llfind(ll, ll->length-1);
}

void llpb(LinkedList *ll, void *data)
{
    llp(ll, data, ll->length-1);
}

void *llread(LinkedList *ll, unsigned int index)
{
    return llfind(ll, index)->data;
}
