

/*
    A* 알고리즘 기반 길찾기 프로그램 (C++11 이상)
    --------------------------------------------
    🧭 기능:
      - '@' : 시작 위치
      - '#' : 장애물 (통과 불가)
      - '.' : 빈 공간
      - '*' : 이동 경로
      - 'X' : 목표 지점
    --------------------------------------------
    사용 방법:
      1. 프로그램 실행 시 초기 맵이 출력됨
      2. 목표 위치 (row, col)을 입력하면
         A* 알고리즘이 최단 경로를 찾아 표시함
*/

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <windows.h>

using namespace std;

const int ROWS = 24;
const int COLS = 40;
bool buqlid = false;
bool bstaticObstac = true;
bool bastar = true;
int numofObstac = 0;
pair<int, int> startPos = { 0, 0 };
pair<int, int> goalPos;

const char START = 'S';
const char WALL = '#';
const char EMPTY = '.';
const char GOAL = 'X';
const char ROAD = '*';
const char CHECKROAD = '^';

 char curPrintedChar;

WORD originalAttrs;
WORD curAttrs;

HANDLE hConsole;
//유틸//
void SetCursorVisible(bool bvisible)
{
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO info;
    GetConsoleCursorInfo(h, &info);
    info.bVisible = bvisible ?  TRUE : FALSE;
    SetConsoleCursorInfo(h, &info);
}

void FreezeCursorPos()
{
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info;
    //GetConsoleScreenBufferInfo(h, &info);
    COORD pos = { (SHORT)0, (SHORT)0};
    SetConsoleCursorPosition(h, pos);
}


struct Node
{
    int r, c;
    float g, h;

    float f() const { return g + h; }

    bool operator > (const Node& other) const { return f() > other.f(); }
};

struct PairHash //unordered_map의 해쉬함수를 위해 필요한 함수객체(Functor)
{
    size_t operator () (const pair<int, int>& p) const noexcept { return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1); }
};

vector<vector<char>> grid(ROWS, vector<char>(COLS, EMPTY));

void SetPrintColor(char c)
{
    if (c == curPrintedChar)
        return;

    if (c == EMPTY)
    {
        SetConsoleTextAttribute(hConsole, originalAttrs);
        curPrintedChar = c;
    }
    else if (c == ROAD)
    {
        SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
        curPrintedChar = c;
    }
    else if (c == WALL)
    {
        SetConsoleTextAttribute(hConsole, FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        curPrintedChar = c;
    }
    else if (c == START)
    {
        SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_INTENSITY);
        curPrintedChar = c;

    }
    else if (c == GOAL)
    {
        SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN);
        curPrintedChar = c;
    }
    else if (c == CHECKROAD)
    {
        SetConsoleTextAttribute(hConsole, FOREGROUND_RED);
        curPrintedChar = c;
    }
    else
    {
        SetConsoleTextAttribute(hConsole, originalAttrs);
        curPrintedChar = c;
    }

}
void printGrid(bool bFreeze = false)
{
    if (bFreeze)
        FreezeCursorPos();
    cout << "  ";
    for (int i = 0; i < COLS; ++i)
    {
        if (i < 10)
            cout << "  " << i << " ";
        else
            cout << "  " << i;
    }
    cout << endl;
    for (int r = 0; r < ROWS; ++r)
    {
        cout << r << "   ";
        for (int c = 0; c < COLS; ++c)
        {
            SetPrintColor(grid[r][c]);
            cout << grid[r][c] << "   ";
        }
        cout << endl << endl;
    }
    SetPrintColor(';');
}


inline int sign(int v) {
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
}

void printDirections(const vector<pair<int, int>>& _paths, const int waittime = 50)
{

    pair<int, int> prev = { startPos.first, startPos.second };
    for (auto& path : _paths)
    {
        if (waittime > 0)
            this_thread::sleep_for(chrono::milliseconds(waittime));
        if (path.first != goalPos.first || path.second != goalPos.second)
        {
            grid[path.first][path.second] = ROAD;
            
            int dx = sign(path.first - prev.first); int dy = sign(path.second - prev.second);
            int nx = prev.first; int ny = prev.second;
            while (true)
            {
                nx += dx; ny += dy;
                if (nx == path.first && ny == path.second)
                    break;
                
                grid[nx][ny] = ROAD;
            }
                
            
            prev = path;
        }


        printGrid(true);


    }


}
bool isValid(int x, int y) {
    return x >= 0 && y >= 0 && x < ROWS && y < COLS && grid[x][y] == EMPTY;
}

float heuristic(const pair<int, int>& p1, const pair<int, int>& p2)
{
    return sqrt((p1.first - p2.first)*(p1.first - p2.first) + (p1.second - p2.second)*(p1.second - p2.second));
}

vector<pair<int, int>> reconstruct_path(unordered_map<pair<int, int>, pair<int, int>, PairHash>& came_from, pair<int, int> current)
{
    vector<pair<int, int>> path;

    while (came_from.find(current) != came_from.end())
    {
        path.push_back(current);
        current = came_from[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<pair<int, int>> astar(pair<int, int> start, pair<int, int> goal)
{
    priority_queue<Node, vector<Node>, greater<Node>> open;
    unordered_map<pair<int, int>, float, PairHash> g_score;
    unordered_map<pair<int, int>, pair<int, int>, PairHash> came_from;

    g_score[start] = 0;
    open.push({ start.first, start.second, 0.0f, heuristic(start, goal) });

    vector<pair<int, int>> directions;
    directions.push_back(pair<int, int>(1, 0));
    directions.push_back(pair<int, int>(0, 1));
    directions.push_back(pair<int, int>(-1, 0));
    directions.push_back(pair<int, int>(0, -1));


    vector<pair<int, int>> uqlidDirecs;
    uqlidDirecs.push_back(pair<int, int>(1, 1));
    uqlidDirecs.push_back(pair<int, int>(1, -1));
    uqlidDirecs.push_back(pair<int, int>(-1, -1));
    uqlidDirecs.push_back(pair<int, int>(-1, 1));
    while (!open.empty())
    {
        Node current = open.top();
        open.pop();
        
        grid[current.r][current.c] = CHECKROAD;
        printGrid(true);

        pair<int, int> curPos(current.r, current.c);

        if (curPos == goal)
            return reconstruct_path(came_from, curPos);

        for (const auto& dir : directions)
        {
            int dr = dir.first;
            int dc = dir.second;
            int nr = current.r + dr;
            int nc = current.c + dc;
            pair<int, int> neighbor = { nr, nc };

            if (nr < 0 || nc < 0 || nr >= ROWS || nc >= COLS)
                continue;
            if (grid[nr][nc] == WALL) // 해당 노드가 벽이다
                continue;
            float tentative_g = g_score[curPos] + 1.0f;
            if (!g_score.count(neighbor) || g_score[neighbor] > tentative_g)
            {
                came_from[neighbor] = curPos;
                g_score[neighbor] = tentative_g;
                open.push({ neighbor.first, neighbor.second, tentative_g, heuristic(neighbor, goal) });
            }
        }

        if (buqlid)
        {
            for (const auto& dir : uqlidDirecs)
            {
                int dr = dir.first;
                int dc = dir.second;
                int nr = current.r + dr;
                int nc = current.c + dc;
                pair<int, int> neighbor = { nr, nc };

                if (nr < 0 || nc < 0 || nr >= ROWS || nc >= COLS)
                    continue;
                if (grid[nr][nc] == WALL) // 해당 노드가 벽이다
                    continue;
                float tentative_g = g_score[curPos] + 1.4f;
                if (!g_score.count(neighbor) || g_score[neighbor] > tentative_g)
                {
                    came_from[neighbor] = curPos;
                    g_score[neighbor] = tentative_g;
                    open.push({ neighbor.first, neighbor.second, tentative_g, heuristic(neighbor, goal) });
                }
            }
        }
    }



    return {};
}

namespace JPS
{

    pair<int, int> jump(int x, int y, int dx, int dy, pair<int, int> goal) {
        int nx = x + dx, ny = y + dy;
        if (!isValid(nx, ny)) return { -1,-1 };
        if (nx == goal.first && ny == goal.second) return { nx, ny };

        // forced neighbor check
        if (dx != 0 && dy != 0) {
            if ((isValid(nx - dx, ny + dy) && !isValid(nx - dx, ny)) ||
                (isValid(nx + dx, ny - dy) && !isValid(nx, ny - dy)))
                return { nx, ny };
        }
        else {
            if (dx != 0) {
                if ((isValid(nx + dx, ny + 1) && !isValid(nx, ny + 1)) ||
                    (isValid(nx + dx, ny - 1) && !isValid(nx, ny - 1)))
                    return { nx, ny };
            }
            else {
                if ((isValid(nx + 1, ny + dy) && !isValid(nx + 1, ny)) ||
                    (isValid(nx - 1, ny + dy) && !isValid(nx - 1, ny)))
                    return { nx, ny };
            }
        }

        // recursive jump
        if (dx != 0 && dy != 0) {
            auto j1 = jump(nx, ny, dx, 0, goal);
            auto j2 = jump(nx, ny, 0, dy, goal);
            if (j1.first != -1 || j2.first != -1) return { nx, ny };
        }

        return jump(nx, ny, dx, dy, goal);
    }

    std::vector<std::pair<int, int>> findNeighbors(int x, int y, std::pair<int, int> parent) {
        std::vector<std::pair<int, int>> neighbors;

        // 부모가 유효하지 않으면 (start node) — 모든 유효한 이웃 반환
   

        // 부모 방향 정규화: dx,dy ∈ {-1,0,1}
        int pdx = x - parent.first;
        int pdy = y - parent.second;
        int dx = sign(pdx);
        int dy = sign(pdy);

        // CASE 1: 대각선 이동 (dx != 0 && dy != 0)
        if (dx != 0 && dy != 0) {
            // natural neighbors (항상 고려)
            // - 전진 대각선
            if (isValid(x + dx, y + dy)) neighbors.emplace_back(dx, dy);
            // - 수평과 수직의 자연 확장(대각선 이동시 허용되는 두 축)
            if (isValid(x + dx, y)) neighbors.emplace_back(dx,0);
            if (isValid(x, y + dy)) neighbors.emplace_back(0, dy);

            // forced neighbours: 주변 블록 상태로 인해 강제로 확인해야 하는 칸
            // 규칙: 만약 해당 축의 옆칸이 막혀있고, 그 옆칸을 우회하는 대각선 칸은 통행 가능하면 forced
            if (!isValid(x - dx, y) && isValid(x - dx, y + dy))
                neighbors.emplace_back(-dx, dy);
            if (!isValid(x, y - dy) && isValid(x + dx, y - dy))
                neighbors.emplace_back( dx, -dy);

            return neighbors;
        }

        // CASE 2: 수직 이동 (dx != 0, dy == 0)
        if (dx != 0 && dy == 0) {
            // natural neighbor: 바로 전진
            if (isValid(x + dx, y)) neighbors.emplace_back(dx, 0);

            // forced neighbors: 좌우가 막혀있고 그 대각선이 뚫렸을 때
            if (!isValid(x, y + 1) && isValid(x + dx, y + 1))
                neighbors.emplace_back(dx, 1);
            if (!isValid(x, y - 1) && isValid(x + dx, y - 1))
                neighbors.emplace_back(dx, -1);

            return neighbors;
        }

        // CASE 3: 수평 이동 (dx == 0, dy != 0)
        if (dx == 0 && dy != 0) {
            // natural neighbor: 바로 전진 (수직)
            if (isValid(x, y + dy)) neighbors.emplace_back(0, dy);

            // forced neighbors: 위아래 좌우 블록 상테에 따른 대각선 후보
            if (!isValid(x + 1, y) && isValid(x + 1, y + dy))
                neighbors.emplace_back(1, dy);
            if (!isValid(x - 1, y) && isValid(x - 1, y + dy))
                neighbors.emplace_back(-1, dy);

            return neighbors;
        }

        // 기본적으로는 빈 벡터 반환
        return neighbors;
    }

    vector<pair<int, int>> JPS(pair<int, int> start, pair<int, int> goal) {
        priority_queue<Node, vector<Node>, greater<Node>> open;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> came_from;
        unordered_map<pair<int, int>, float, PairHash> g_score;

        vector<pair<int, int>> dx;
        dx.push_back(pair<int, int>(1, 0));
        dx.push_back(pair<int, int>(0, 1));
        dx.push_back(pair<int, int>(-1, 0));
        dx.push_back(pair<int, int>(0, -1));
        dx.push_back(pair<int, int>(1, 1));
        dx.push_back(pair<int, int>(1, -1));
        dx.push_back(pair<int, int>(-1, -1));
        dx.push_back(pair<int, int>(-1, 1));


        open.push({ start.first, start.second, 0, heuristic(start, goal) });
        g_score[start] = 0;

        while (!open.empty()) {
            Node cur = open.top(); open.pop();
            pair<int, int> pos = { cur.r, cur.c };

            grid[cur.r][cur.c] = CHECKROAD;
            printGrid(true);

            if (pos == goal) {
          
                return reconstruct_path(came_from, pos);
            }
            
            vector<pair<int, int>> dirs;
            if (came_from.count(pos))
                dirs = findNeighbors(cur.r, cur.c, came_from[pos]);
            else
                for (auto& d : dx)
                    dirs.push_back({ d.first, d.second });

            for (auto [dx_, dy_] : dirs) {
                auto jp = jump(cur.r, cur.c, dx_, dy_, goal);
                if (jp.first == -1) continue;

                float ng = g_score[pos] + heuristic(pos, jp);
                if (!g_score.count(jp) || ng < g_score[jp]) {
                    g_score[jp] = ng;
                    float h = heuristic(jp, goal);
                    open.push({ jp.first, jp.second, ng, h });
                    came_from[jp] = pos;
                }
            }
        }
        return {};
    }

}

void HandleDismatchInput()
{
    cout << "올바른 입력이 아닙니다. 다시 입력해주세요." << endl;
/*    this_thread::sleep_for(chrono::seconds(1));

    std::cout << "\33[2K\r";
    std::cout << "\33[2K\r"*/;
    
}
//void enableANSI() {
//
//    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
//    DWORD dwMode = 0;
//    GetConsoleMode(hOut, &dwMode);
//    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
//    SetConsoleMode(hOut, dwMode);
//}
void MakeObstacle()
{
   

    int num = 0;
    int start_R = 0;
    int start_C = 0;
    int howlong = 0;
    int start = 0;
    int end = 0;

    srand((unsigned int)time(NULL));

    for (int i = 0; i < numofObstac; i++)
    {
        //cout << "MakeObstacle" << endl;
        

        num = rand() % 2;
        start_C = rand() % COLS;
        start_R = rand() % ROWS;

        if (num == 1) // Row 세로 장애물
        {
            howlong = rand() % ROWS;
            start = start_R - (howlong / 4);
            end = start_R + (howlong / 4);
            if (start < 0)
                start = 0;
            if (end >= ROWS)
                end = ROWS - 1;
            for (int i = start; i <= end; i++)
                grid[i][start_C] = WALL;
        }
        else // Cols 가로 장애물
        {

            howlong = rand() % COLS;
            start = start_C - (howlong / 4);
            end = start_C + (howlong / 4);
            if (start < 0)
                start = 0;
            if (end >= COLS)
                end = COLS - 1;
            for (int i = start; i <= end; i++)
                grid[start_R][i] = WALL;
        }
    }
}
int main()
{

    system("cls");
    SetCursorVisible(false);

    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO consoleInfo;
    GetConsoleScreenBufferInfo(hConsole, &consoleInfo);
    originalAttrs = consoleInfo.wAttributes;

    while (true)
    {
       
        cout << "A* 알고리즘 기반 길찾기 프로그램 (C++11 이상)" << endl <<
            "--------------------------------------------" << endl <<
            "  표시 기호 :" << endl <<
            "- '" << START<<"' : 시작 위치" << endl <<
            "- '" << WALL << "' : 장애물(통과 불가)" << endl <<
            "- '" << EMPTY << "' : 빈 공간" << endl <<
            "- '" << ROAD << "' : 이동 경로" << endl <<
            "- '" << GOAL << "' : 목표 지점" << endl <<
            "- '" << CHECKROAD << "' : 지나간곳" << endl <<
            "--------------------------------------------" << endl <<
            "사용 방법 :" << endl <<
            "1. 프로그램 실행 시 초기 맵이 출력됨" << endl <<
            "2. 목표 위치(row, col)을 입력하면" << endl <<
            "A * 알고리즘이 최단 경로를 찾아 표시함" << endl << endl;

        grid = vector<vector<char>>(ROWS, vector<char>(COLS, EMPTY)); // 맵 초기화
        int astarorjps;
        while (true)
        {
            cout << "알고리즘 선택 | A* : 1 / JPS(Jump Point Search) : 2 -> ";
            cin >> astarorjps;

            if (astarorjps != 1 && astarorjps != 2)
            {
                HandleDismatchInput();
                continue;
            }
            else if (astarorjps == 1) bastar = true;
            else bastar = false;
            break;
        }
        int staordy = 0;
        while (true)
        {
            cout << "고정 장애물 또는 랜덤 생성 장애물인지 선택해주세요 | 고정 : 1 / 랜덤 : 2 -> ";
            cin >> staordy;

            if (staordy != 1 && staordy != 2)
            {
                HandleDismatchInput();
                continue;
            }
            else if (staordy == 1) bstaticObstac = true;
            else bstaticObstac = false;
            break;
        }
        if (bstaticObstac)
        {
            for (int i = 5; i < 20; ++i) grid[7][i] = WALL;   // 가로 벽
            for (int i = 0; i < 10; ++i) grid[i][10] = WALL;  // 세로 벽
            for (int i = 15; i < 35; ++i) grid[13][i] = WALL;   // 가로 벽
            for (int i = 14; i < ROWS; ++i) grid[i][20] = WALL;  // 세로 벽
        }
        else
        {
            while (true)
            {
                cout << "설정 할 장애물 개수를 정해주세요 : ";
                cin >> numofObstac;
                if (numofObstac < 0)
                {
                    HandleDismatchInput();
                    continue;
                }
                else
                {
                    MakeObstacle();
                    break;
                }
            }
        }
       
        while (true)
        {
            cout << "휴리스틱 계산에 필요한 기준을 정해주세요. 맨헤튼 : 1 입력 / 유클리드 : 2 입력" << endl;
            int select;
            cin >> select;
            if (select != 1 && select != 2)
            {
                HandleDismatchInput();
                continue;
            }
            else if (select == 2)
            {
                buqlid = true;
                break;
            }
            else
            {
                buqlid = false;
                break;
            }
        }
        // 

        cout << "===== 초기 맵 =====" << endl;
        printGrid();

        while (true)
        {
            cout << "시작 위치 입력 (row col): ";
            if (!(cin >> startPos.first >> startPos.second))
            {
                HandleDismatchInput();
                continue;
            }
            if (startPos.first < 0 || startPos.first >= ROWS ||
                startPos.second < 0 || startPos.second >= COLS) {
                cout << "시작 좌표가 범위를 벗어났습니다.\n";
                HandleDismatchInput();
                continue;
            }
            if (grid[startPos.first][startPos.second] == WALL) {
                cout << "해당 좌표는 벽입니다.\n";
                HandleDismatchInput();
                continue;
            }
            else
            {
                grid[startPos.first][startPos.second] = START;
                break;
            }
        }
        while (true)
        {
            cout << "목표 위치 입력 (row col): ";
            if (!(cin >> goalPos.first >> goalPos.second))
            {
                HandleDismatchInput();
                continue;
            }
            if (goalPos.first < 0 || goalPos.first >= ROWS ||
                goalPos.second < 0 || goalPos.second >= COLS) {
                cout << "목표 좌표가 범위를 벗어났습니다.\n";
                HandleDismatchInput();
                continue;
            }
            if (grid[goalPos.first][goalPos.second] == WALL) {
                cout << "목표가 벽입니다.\n";
                HandleDismatchInput();
                continue;
            }
            else break;
        }

        system("cls");

        vector<pair<int, int>> path;

        if(bastar)
            path = astar(startPos, goalPos);
        else 
            path = JPS::JPS(startPos, goalPos);
        if (path.empty()) {
            cout << "경로를 찾을 수 없습니다.\n";
            goto RETRY;
        }
        grid[startPos.first][startPos.second] = START;
       grid[goalPos.first][goalPos.second] = GOAL;
  
       system("cls");
        cout << "===== 경로 표시 =====\n";

        //printGrid();

        printDirections(path);

RETRY:
        cout << endl;
        char re;
        while (true)
        {
            cout << "다시 하시겠습니까? | Yes : Y / No : N ";
            cin >> re;
            if (re != 'Y' && re != 'N')
            {
                HandleDismatchInput();
                continue;
            }
            else if (re == 'Y')
                break;
            else
            {
                cout << endl << "프로그램이 종료되었습니다." << endl;
                return 0;
            }
        }
    }
}