#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>

#define IsPath 2
#define IsOccpyed 1
#define IsEmpty 0

#define SUCCESS 1
#define FAIL 0

/*****values*****/

#define COLUMN 10
#define ROW 10
#define H_MUL 1

/****************/

#define CELL_DOMAIN_MAX COLUMN*ROW
#define DOMAIN_NULL CELL_DOMAIN_MAX*CELL_DOMAIN_MAX
#define INIT_CELL {0, 0, IsEmpty, NULL, DOMAIN_NULL, 0}
#define SWAP(x, y, t) ( (t) = (x), (x) = (y), (y) = (t) )

typedef struct _cell{
    int x;
    int y;
    
    char state;
    struct _cell* parentPtr;

    double f; //f = g + h //minimaize f
    double g; //g = cost of cell //h = heuristic value
}cell;

cell grid[COLUMN][ROW] = {0, };

cell startCell = {0, 0, IsEmpty, NULL, 0, 0};
cell endCell = {0, 0, IsEmpty, NULL, 0, 0};

cell cellDomain[CELL_DOMAIN_MAX] = {INIT_CELL, };
int CDHeader = 0;

void sort(); // 최소 탐색

double clc_f(cell nowCell);

double abs_cust(double x);

void pushCell(int x, int y, cell parent);

void push4(cell middleCell);

/*****Funtions*****/

void gridInit(int state);

char setDpp(int scol, int srow, int ecol, int erow);

cell lpp();

void readPath(cell tagCell, cell path[CELL_DOMAIN_MAX]);

/******************/

#endif