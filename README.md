# Line Pathplaning for stm32

## 일반적인 사용법
### 1. grid 초기화
전역 변수로 선언된 grid를 초기화한다.
```c
void gridInit(int state);
```
state는 총 3가지가 존재한다.
```c
#define IsPath 2
#define IsOccpyed 1
#define IsEmpty 0
```
전역 변수 grid는 아래와 같이 선언되어 있다.
```c
cell grid[COLUMN][ROW] = {0, };
```
### 2. 시작점, 도착점 설정
시작점과 도착점을 설정하여 grid에 저장한다.
```c
char setDpp(int scol, int srow, int ecol, int erow);
```
scol과 srow는 각각 시작점의 x, y이며 ecol과 erow는 각각 도착점의 x, y이다.

추가적으로 시작점과 도착점은 아래와 같은 전역 변수로도 선언되어 있다.
```c
cell startCell = {0, 0, IsEmpty, NULL, 0, 0};
cell endCell = {0, 0, IsEmpty, NULL, 0, 0};
```
### 3. 경로 찾아 마지막 cell 반환
1, 2번을 통해 얻은 정보를 이용해 경로 찾기를 시행한다.
```c
cell lpp();
```
반환되는 cell은 경로찾기에 실패했다면 아래에 해당되는 값을 반환하며
```c
{0, 0, IsEmpty, NULL, DOMAIN_NULL, 0}
```
성공했다면 endCell과 같은 좌표의 cell 값을 반환한다.
### 4. 선정된 경로 읽어오기
cell의 선언을 보면 아래와 같은데
```c
typedef struct _cell{
    int x;
    int y;
    
    char state;
    struct _cell* parentPtr;

    double f; //f = g + h //minimaize f
    double g; //g = cost of cell //h = heuristic value
}cell;
```
parentPtr은 해당 cell에 있기 직전의 cell의 주소를 가리킨다.

즉 최종 cell에서 시작해 startCell이 나올때까지 참조하면 어떠한 경로가 선정되었는지 알 수 있다.

------------------------------------------------------

사용되는 함수의 시그니처는 아래와 같다.
```c
void readPath(cell tagCell, cell path[CELL_DOMAIN_MAX]);
```
malloc을 사용하지 않기 위해 path가 담길 크기가 지정된 array를 요구한다.

## 메크로 설정
```c
#define COLUMN 10
#define ROW 10
#define H_MUL 1
```
COLUMN과 ROW는 단어가 뜻하는 것과 같이 grid의 크기를 지정하는 메크로이다.

H_MUL은 heurstic 항에 곱해지는 수인데 이 수가 클 수록 좀 더 도착지에 직선적으로 다가가는 경향이 있다.