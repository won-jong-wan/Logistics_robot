#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#include <stdbool.h>
#include <stdlib.h>

typedef enum _BoxT{
  EMPTY,
  SOME,
  FULL,
}BoxT;

typedef enum _BoxC{
  UNKNOWN,
  CATEGORYA,
  CATEGORYB,
}BoxC;

typedef struct _Box{
    int id;
    unsigned int priority;
    BoxT load;
    BoxC category;
}Box;

int idSeed = 0;

Box* boxInit(int priority, BoxT load, BoxC category);
bool boxComp(Box* box0, Box* box1);
void delBox(Box* boxPtr);

#endif
