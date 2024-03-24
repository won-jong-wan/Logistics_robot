#include "box.h"

int idSeed =0;

Box* boxinit(int priority, BoxS load, void* data)
{
  Box* box = (Box*)malloc(sizeof(Box));

  idSeed++;
  box->id = idSeed;
  box->priority = priority;
  box->load = load;
  box->data = data;
  return box;
}

bool boxcomp(Box *box0, Box *box1)
{
  if(box0->load !=box1->id){
    return false;
  }
  if(box0->load ==box1->id){
    return true;
  }
}

void boxdel(Box *boxPtr)
{
  free(boxPtr->data);
  printf("Void ptr free\n");
  free(boxPtr);
  printf("Box ptr free\n");
}
