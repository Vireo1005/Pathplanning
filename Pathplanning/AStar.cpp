#include "pch.h"
#include "AStar.h"
#include<functional>
#include<climits>
#include<utility>

PathPlanner::PathPlanner() : m_rows(10), m_cols(10), m_start(0, 0), m_end(9, 9) {
    SetMapSize(10, 10);
}

PathPlanner::~PathPlanner() {
}

void PathPlanner::SetMapSize(int rows, int cols) {
    m_rows = rows;
    m_cols = cols;
    m_map.assign(rows, vector<bool>(cols, false));
}

void PathPlanner::SetObstacle(int x, int y, bool isObstacle) {
    if (IsValid(x, y)) {
        m_map[x][y] = isObstacle;
    }
}

void PathPlanner::ClearObstacles() {
    for (int i = 0; i < m_rows; i++) {
        for (int j = 0; j < m_cols; j++) {
            m_map[i][j] = false;
        }
    }
}

void PathPlanner::SetStart(int x, int y) {
    if (IsValid(x, y)) {
        m_start = Point(x, y);
    }
}

void PathPlanner::SetEnd(int x, int y) {
    if (IsValid(x, y)) {
        m_end = Point(x, y);
    }
}

bool PathPlanner::IsValid(int x, int y) {
    return x >= 0 && x < m_rows && y >= 0 && y < m_cols;
}

int PathPlanner::ManhattanDistance(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// BFS广度优先搜索
bool PathPlanner::BFS(vector<Point>& path) {
    path.clear();
    if (m_start == m_end) {
        path.push_back(m_start);
        return true;
    }

    vector<vector<bool>> visited(m_rows, vector<bool>(m_cols, false));
    vector<vector<Point>> parent(m_rows, vector<Point>(m_cols, Point(-1, -1)));
    queue<Point> q;

    q.push(m_start);
    visited[m_start.x][m_start.y] = true;

    // 四个方向：上下左右
    int dx[] = { -1, 1, 0, 0 };
    int dy[] = { 0, 0, -1, 1 };

    while (!q.empty()) {
        Point curr = q.front();
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];

            if (IsValid(nx, ny) && !visited[nx][ny] && !m_map[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = curr;

                if (nx == m_end.x && ny == m_end.y) {
                    // 回溯路径
                    Point p = m_end;
                    while (!(p == Point(-1, -1))) {
                        path.push_back(p);
                        p = parent[p.x][p.y];
                    }
                    reverse(path.begin(), path.end());
                    return true;
                }
                q.push(Point(nx, ny));
            }
        }
    }
    return false;
}

// A*算法
bool PathPlanner::AStar(vector<Point>& path) {
    path.clear();
    if (m_start == m_end) {
        path.push_back(m_start);
        return true;
    }

    vector<vector<bool>> closed(m_rows, vector<bool>(m_cols, false));
    vector<vector<Point>> parent(m_rows, vector<Point>(m_cols, Point(-1, -1)));
    vector<vector<int>> gValue(m_rows, vector<int>(m_cols, INT_MAX));

    priority_queue<AStarNode, vector<AStarNode>, CompareNode> openList;

    gValue[m_start.x][m_start.y] = 0;
    openList.push(AStarNode(m_start, 0, ManhattanDistance(m_start, m_end), Point(-1, -1)));

    // 四个方向
    int dx[] = { -1, 1, 0, 0 };
    int dy[] = { 0, 0, -1, 1 };

    while (!openList.empty()) {
        AStarNode curr = openList.top();
        openList.pop();

        if (closed[curr.pos.x][curr.pos.y]) continue;
        closed[curr.pos.x][curr.pos.y] = true;
        parent[curr.pos.x][curr.pos.y] = curr.parent;

        if (curr.pos == m_end) {
            // 回溯路径
            Point p = m_end;
            while (!(p == Point(-1, -1))) {
                path.push_back(p);
                p = parent[p.x][p.y];
            }
            reverse(path.begin(), path.end());
            return true;
        }

        for (int i = 0; i < 4; i++) {
            int nx = curr.pos.x + dx[i];
            int ny = curr.pos.y + dy[i];

            if (IsValid(nx, ny) && !closed[nx][ny] && !m_map[nx][ny]) {
                int newG = curr.g + 1;
                if (newG < gValue[nx][ny]) {
                    gValue[nx][ny] = newG;
                    int h = ManhattanDistance(Point(nx, ny), m_end);
                    openList.push(AStarNode(Point(nx, ny), newG, h, curr.pos));
                }
            }
        }
    }
    return false;
}


// Dijkstra 算法
bool PathPlanner::Dijkstra(vector<Point>& path) {
    path.clear();
    if (m_start == m_end) {
        path.push_back(m_start);
        return true;
    }

    vector<vector<int>> dist(m_rows, vector<int>(m_cols, INT_MAX));
    vector<vector<Point>> parent(m_rows, vector<Point>(m_cols, Point(-1, -1)));

    // 小顶堆：first = 距离, second = 点坐标
    priority_queue<pair<int, Point>, vector<pair<int, Point>>, greater<pair<int, Point>>> pq;

    dist[m_start.x][m_start.y] = 0;
    pq.push({ 0, m_start });

    int dx[] = { -1, 1, 0, 0 };
    int dy[] = { 0, 0, -1, 1 };

    while (!pq.empty()) {
        pair<int, Point> top = pq.top();
        int d = top.first;
        Point curr = top.second;
        pq.pop();

        if (curr == m_end) break;
        if (d > dist[curr.x][curr.y]) continue;

        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];

            if (!IsValid(nx, ny) || m_map[nx][ny]) continue;

            int newDist = d + 1;
            if (newDist < dist[nx][ny]) {
                dist[nx][ny] = newDist;
                parent[nx][ny] = curr;
                pq.push({ newDist, Point(nx, ny) });
            }
        }
    }

    if (dist[m_end.x][m_end.y] == INT_MAX) return false;

    // 回溯路径
    Point p = m_end;
    while (!(p == Point(-1, -1))) {
        path.push_back(p);
        p = parent[p.x][p.y];
    }
    reverse(path.begin(), path.end());
    return true;
}


// DFS 深度优先搜索
bool PathPlanner::DFS(vector<Point>& path) {
    path.clear();
    if (m_start == m_end) {
        path.push_back(m_start);
        return true;
    }

    vector<vector<bool>> visited(m_rows, vector<bool>(m_cols, false));
    vector<Point> currentPath;
    bool found = false;

    // 递归 lambda
    function<void(int, int)> dfs = [&](int x, int y) {
        if (found) return;
        if (!IsValid(x, y) || visited[x][y] || m_map[x][y]) return;

        visited[x][y] = true;
        currentPath.push_back(Point(x, y));

        if (x == m_end.x && y == m_end.y) {
            path = currentPath;
            found = true;
            return;
        }

        // 按上下左右顺序探索
        dfs(x - 1, y);
        dfs(x + 1, y);
        dfs(x, y - 1);
        dfs(x, y + 1);

        currentPath.pop_back();
        };

    dfs(m_start.x, m_start.y);
    return found;
}