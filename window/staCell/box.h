#ifndef BOX_H
#define BOX_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

typedef enum _BoxS{
  EMPTY,
  SOME,
  FULL,
}BoxS;

/*typedef enum _BoxC{
  UNKNOWN,
  CATEGORYA,
  CATEGORYB,
}BoxC;*/

typedef struct _Box{
    int id;
    unsigned int priority;
    BoxS load;
    //BoxC category
    void* data;
}Box;

int idSeed;

Box* boxinit(int priority, BoxS load, void* data);
bool boxcomp(Box* box0, Box* box1);
void boxdel(Box* boxPtr);

#endif
