#include "box.h"

Box *boxInit(int priority, BoxT load, BoxC category)
{
  Box* box = (Box*)malloc(sizeof(Box));

  idSeed++;
  box->id = idSeed;
  box->priority = priority;
  box->load = load;
  box->category = category;
  return box;
}

bool boxComp(Box *box0, Box *box1)
{
  if(box0->load !=box1->id){
    return false;
  }
  if(box0->load ==box1->id){
    return true;
  }
}

void delBox(Box *boxPtr)
{
  free(boxPtr);
}
