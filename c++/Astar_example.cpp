#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

enum class CellT {
	USABLE,
	BLOCKED,
	TARGET,
};
struct Cell{
	int x, y;
	CellT type;
	Cell* parentP;
	Cell(int x = 0, int y = 0, CellT type = CellT::USABLE
		, Cell* parentP = nullptr) : x(x), y(y), type(type), parentP(parentP) {};
};

//Grid
class Grid{
public:
	Grid(int x, int y);
	~Grid();

	vector<Cell> operator[] (unsigned int idx1);
	void writeType(int x, int y, CellT type);
	void writeParent(int x, int y, int px, int py);
	Cell* ReadParent(int x, int y);
private:
	vector<vector<Cell>> cells;
};

Grid::Grid(int x, int y){
	cells.assign(y, vector<Cell>(x, Cell()));
	cout <<cells[0].size() <<" " << cells.size() << endl;
	for (int i = 0; i < y; i++) {
		for (int j = 0; j < x; j++) {
			cells[i][j].y = i;
			cells[i][j].x = j;
		}
	}
}

Grid::~Grid(){}

vector<Cell> Grid::operator[](unsigned int idx1) {
	return cells[idx1];
}

void Grid::writeType(int x, int y, CellT type){
	cells[y][x].type = type;
}

void Grid::writeParent(int x, int y, int px, int py){
	cells[y][x].parentP = &cells[py][px];
}

Cell* Grid::ReadParent(int x, int y)
{
	return cells[y][x].parentP;
}

//GridMaker
class GridMaker{
public:
	GridMaker(int x=3, int y=2) :x(x), y(y) {};
	~GridMaker();
	Grid& make();
	void deleteGrid(Grid* odd);
private:
	int x, y;
};

GridMaker::~GridMaker(){}

Grid& GridMaker::make()
{
	Grid* grid = new Grid(x, y);
	return (*grid);
}

void GridMaker::deleteGrid(Grid* odd)
{
	delete odd;
}


class GridPathFinder{
public:
	GridPathFinder(Grid* grid) : grid(grid) {};
	vector<Cell> pathPrint(int x, int y);

private:
	Grid* grid;
};

//캡슐화 이슈
vector<Cell> GridPathFinder::pathPrint(int x, int y) {
	Cell* testCell = grid->ReadParent(x,y);
	vector<Cell> path;
	path.push_back((*grid)[y][x]);

	while (testCell->parentP != nullptr) {
		path.push_back(*testCell);
		testCell = testCell->parentP;
	}

	return path;
}

/*class GridHandler
{
public:
	GridHandler(Grid* grid);
	~GridHandler();

	void gridWrite(int x, int y, CellT type = CellT::USABLE);
	void gridRead(int x, int y);
private:
	Grid* grid;
};

GridHandler::GridHandler(Grid* grid):grid(grid){}

GridHandler::~GridHandler(){}

void GridHandler::gridWrite(int x, int y, CellT type){
	(*grid)[y][x].type = type;
}
void GridHandler::gridRead(int x, int y)
{
	//cout << grid[y][x].x << endl;
}*/

int main() {
	GridMaker GM;

	Grid g0 = GM.make();

	cout<<g0[1][2].y<<endl;

	g0.writeType(1, 1, CellT::BLOCKED);
	if (g0[1][1].type != CellT::USABLE) {
		cout << "!" << endl;
	}
	else {
		cout << "?" << endl;
	}

	g0.writeParent(1, 1, 1, 0);
	cout << g0.ReadParent(1,1)->y << endl;

	GridPathFinder PF(&g0);

	vector<Cell> vc = PF.pathPrint(1, 1);

	cout << vc[0].y << endl;

	return 0;
}
