#ifndef STACK_H
#define STACK_H

#include "box.h"

#define STACK_N 3

typedef struct _Stack{
    int top;
    int capacity;
    Box** array;
}Stack;

Stack* stainit(int capacity);
// Stack* stainit();
void stadel(Stack* sta);

bool staisEm(Stack* sta);
bool staisFu(Stack* sta);

Box* stapeek(Stack* sta);
void stapush(Stack* sta, Box* box);
Box* stapop(Stack* sta);

#endif
