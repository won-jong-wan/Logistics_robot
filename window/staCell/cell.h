#ifndef CELL_H
#define CELL_H

typedef enum _CellT{
    USABLE,
    BLOCKED,
    TARGET,
}CellT;

typedef struct _Cell{
    int x;
    int y;
    CellT state;        
}Cell;

typedef struct _Grid{
    Cell** grid;
}Grid;

Cell ceinit(int x, int y, CellT state);
Grid* grinit(int row, int column, CellT state);

Cell ceread(Grid* gr);

#endif 