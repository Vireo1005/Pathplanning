#pragma once
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

// 点结构体
struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

// A*算法节点
struct AStarNode {
    Point pos;
    int g;  // 从起点到当前节点的代价
    int h;  // 从当前节点到终点的估计代价
    int f;  // f = g + h
    Point parent;
    AStarNode() : g(0), h(0), f(0) {}
    AStarNode(Point p, int g_, int h_, Point par) : pos(p), g(g_), h(h_), parent(par) {
        f = g + h;
    }
};

// 优先队列比较器（f值小的优先）
struct CompareNode {
    bool operator()(const AStarNode& a, const AStarNode& b) {
        return a.f > b.f;
    }
};

class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner();

    // 设置地图大小
    void SetMapSize(int rows, int cols);

    // 设置障碍物
    void SetObstacle(int x, int y, bool isObstacle);

    // 清除所有障碍物
    void ClearObstacles();

    // 设置起点
    void SetStart(int x, int y);

    // 设置终点
    void SetEnd(int x, int y);

    // BFS广度优先搜索
    bool BFS(vector<Point>& path);

    // A*算法搜索
    bool AStar(vector<Point>& path);

    bool Dijkstra(vector<Point>& path);
    bool DFS(vector<Point>& path);

    // 获取地图
    const vector<vector<bool>>& GetMap() const { return m_map; }
    Point GetStart() const { return m_start; }
    Point GetEnd() const { return m_end; }
    int GetRows() const { return m_rows; }
    int GetCols() const { return m_cols; }

private:
    int m_rows, m_cols;
    vector<vector<bool>> m_map;  // true表示障碍物
    Point m_start, m_end;

    // 曼哈顿距离启发函数
    int ManhattanDistance(Point a, Point b);

    // 检查点是否有效
    bool IsValid(int x, int y);
};
