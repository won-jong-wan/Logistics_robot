#ifndef ROBOT_H
#define ROBOT_H

#include "staCell.h"

typedef enum _RobMovM{
    STOP,
    MOVEX,
    MOVEY,
}RobMovM;

typedef enum _RobGraM{
    UNGRAB,
    GRAB,
}RobGraM;

typedef enum _GrabLev{
    ROBOT,
    FIRST,
    SECOND,
    THRID,
}GrabLev;

typedef struct _Robot{
    unsigned int rx;
    unsigned int ry;
    LinkedList* rPass;
    RobMovM rMovM;
    RobGraM rGrabM;
    GrabLev rGrabL;
    Cell rPostion;
}Robot;

#endif