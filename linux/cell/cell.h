#ifndef CELL_H
#define CELL_H

#include "linkedList.h"

typedef enum _CellT{
    USABLE,
    BLOCKED,
    TARGET,
}CellT;

typedef struct _Cell{
    unsigned int x;
    unsigned int y;
    LinkedList* node;
    CellT state;        
}Cell;

typedef struct _Grid{
    Cell grid[4][3];
}Grid;

#endif 