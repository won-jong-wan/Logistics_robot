#include "stack.h"

Stack *stainit(int capacity)
{
    Stack* staPtr = (Stack*)malloc(sizeof(Stack));
    staPtr->top = -1;
    staPtr->capacity = capacity;
    staPtr->array = (Box**)malloc(sizeof(Box*)*staPtr->capacity);

    return staPtr;
}

// Stack *stainit()
// {
//     return stainit(STACK_N);
// }

void stadel(Stack *sta)
{
    int i;
    for(i =0; i< sta->capacity-1; i++){
        boxdel(sta->array[i]);
    }
    free(sta->array);
    free(sta);
    printf("Stack ptr free\n");
}

bool staisEm(Stack *sta)
{
    return sta->top == -1;
}

bool staisFu(Stack *sta)
{
    return sta->top == sta->capacity -1;
}

Box* stapeek(Stack *sta)
{
    if(staisEm(sta)){
        printf("Stack is empty. But you try to peek element. staisEm() = %d\n", staisEm(sta));
        return 0;
    }
    return sta->array[sta->top];
}

void stapush(Stack *sta, Box* box)
{
    if(staisFu(sta)){
        printf("Stack is Full. But you try to push element. staisFu() = %d\n", staisFu(sta));
        return ;
    }
    sta->top = sta->top + 1;
    sta->array[sta->top] = box;
}

Box *stapop(Stack *sta)
{
    if(staisEm(sta)){
        printf("Stack is empty. But you try to pop element. staisEm() = %d\n", staisEm(sta));
        return 0;
    }
    sta->top = sta->top - 1;
    return sta->array[sta->top+1];
}
