#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

// 각 격자 셀의 상태 정의
enum class CellType {
    START,
    USABLE,
    BLOCKED,
    TARGET
};

// 각 셀의 상태와 G, H, F 값을 저장하는 구조체
struct Cell {
    int x, y; //x=> y=^ 
    CellType type;
    int g, h, f;
    Cell* parent;

    Cell(int x, int y, CellType t) : x(x), y(y), type(t), g(0), h(0), f(0), parent(nullptr) {}

    // 셀 간 거리 계산 (맨해튼 거리 사용)
    int distance(Cell* other) const {
        return abs(x - other->x) + abs(y - other->y);
    }
    void updateFGH(int hBase, Cell* endGrid) {
        h = hBase - distance(endGrid);
        g = parent->g + 1;
        f = g + h;
    }
    bool operator<(const Cell c) const {
        return this->f < c.f;
    }
};



int main() {
    //grid 초기화
    vector<vector<Cell>> grid = {
        {Cell(0, 0, CellType::USABLE), Cell(0, 1, CellType::USABLE), Cell(0, 2, CellType::USABLE)},
        {Cell(1, 0, CellType::USABLE), Cell(1, 1, CellType::USABLE), Cell(1, 2, CellType::USABLE)}
    };

    //시작점, 도착점 설정
    Cell startGrid = grid[0][0], endGrid = grid[1][2];

    startGrid.type = CellType::START;
    endGrid.type = CellType::TARGET;

    //우선순위 큐 정의
    priority_queue<Cell> aStarQ;
    aStarQ.push(startGrid);

    //이웃 추가 및 f, g, h 계산 
    int hBase = startGrid.distance(&endGrid);
    startGrid.h = hBase - startGrid.distance(&endGrid);

    grid[0][1].parent = &startGrid;
    grid[0][1].updateFGH(hBase, &endGrid);

    aStarQ.push(grid[0][1]);

    cout <<aStarQ.top().f <<endl ;

    //이웃 고려
    Cell targetCell = startGrid;
    vector<bool> rlud = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; i++) {
         
    }


    return 0;
}
