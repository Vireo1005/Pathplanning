#pragma once
#include "AStar.h"
#include "afxwin.h"
#include <vector>
#include <set>
#include <utility>

// 编辑模式枚举
enum EditMode {
    MODE_NONE = 0,
    MODE_OBSTACLE,
    MODE_GOAL,
    MODE_ERASE,
    MODE_START,
    MODE_ELEVATOR
};

// 动态障碍类型
enum DynObstacleType {
    DYN_RANDOM_WALK = 0,  // 随机游走型：遍历节点、本层随机走动
    DYN_DFS_TO_ELEVATOR   // DFS型：从随机出生点前往电梯（模拟前往第一层起点）
};

// 带楼层信息的路径节点
struct PathNode {
    int floor;
    int x;
    int y;
};

// 终点信息
struct GoalPoint {
    int floor;
    int x;
    int y;
};

// 动态障碍
struct DynObstacle {
    int              floor;     // 所在楼层
    Point            pos;       // 当前位置
    DynObstacleType  type;      // 障碍类型
    vector<Point>    path;      // DFS路径
    int              idx;       // DFS路径进度
    Point            last;      // 随机游走时上一格（不回头）
    bool             active;    // 是否活跃

    DynObstacle() : floor(0), pos(0, 0), type(DYN_RANDOM_WALK), idx(0), last(0, 0), active(true) {}
};

// CPathPlanningDlg 对话框
class CPathPlanningDlg : public CDialogEx
{
public:
    CPathPlanningDlg(CWnd* pParent = nullptr);

#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_PATHPLANNING_DIALOG };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()

public:
    // 控件变量
    int m_nRows;
    int m_nCols;
    int m_nFloors;
    int m_nCurFloor;
    int m_nGoalCount;
    CComboBox m_cmbAlgorithm;

    // 地图数据
    vector<PathPlanner> m_floors;
    Point m_start;
    int   m_startFloor;
    Point m_elevator;
    vector<GoalPoint> m_goals;
    vector<PathNode>  m_fullPath;
    bool  m_bHasPath;

    // 编辑模式
    EditMode m_editMode;

    // 机器人相关
    int  m_nRobotIndex;
    bool m_bRobotMoving;
    UINT m_nTimerID;
    int  m_nSpeed;
    bool m_bAccelerated;
    int  m_nNormalSpeed;

    // 实时路径
    vector<PathNode> m_traveledPath;
    vector<PathNode> m_aheadPath;

    // 动态障碍（每层4个：2随机游走 + 2DFS）
    static const int DYN_PER_FLOOR = 4;
    bool             m_bDynEnabled;
    vector<DynObstacle> m_dynObs;

    // 机器人反应式移动
    PathNode         m_robotPos;
    vector<PathNode> m_waypoints;
    int              m_wpIndex;
    bool             m_bWaiting;
    vector<int>      m_legOf;

    // 绘图
    CRect m_mapRect;
    int   m_cellSize;

    // === 函数声明 ===
    void CalcMapLayout();
    CPoint ScreenToGrid(CPoint point);
    CPoint GridToScreen(int row, int col);
    Point GetElevator();

    void DrawMap(CDC* pDC);
    void DrawGrid(CDC* pDC);
    void DrawObstacles(CDC* pDC);
    void DrawStartEnd(CDC* pDC);
    void DrawPath(CDC* pDC);
    void DrawRobot(CDC* pDC);
    void DrawElevator(CDC* pDC);
    void DrawDynamic(CDC* pDC);
    void DrawArrowHead(CDC* pDC, CPoint from, CPoint to);
    COLORREF LegColor(int leg);

    // 动态障碍
    void InitDynObstacles();
    void SpawnOneDynamic(DynObstacle& ob);
    void RandomWalkStep(DynObstacle& ob, const set<pair<int, int>>& occupied);
    void StepDynamic();

    // 路径规划
    bool FindPathOnFloor(int floor, Point s, Point e, vector<Point>& path, int algo);
    bool GetSegmentPath(const PathNode& a, const PathNode& b, vector<PathNode>& seg, int algo);
    bool PlanRoute(int algo);
    bool PlanAheadFull(int floor, Point s, Point e,
        const set<pair<int, int>>& blocked, vector<Point>& fullPath);

    // 机器人移动
    void StepRobot();
    int  PathDistance(int floor, Point a, Point b, const set<pair<int, int>>& blocked);

    // 编辑操作
    void ToggleGoal(int floor, int r, int c);
    void SetStartAt(int floor, int r, int c);
    void SetElevatorAt(int r, int c);

    // 按钮响应
    afx_msg void OnBnClickedBtnCreate();
    afx_msg void OnBnClickedBtnObstacle();
    afx_msg void OnBnClickedBtnErase();
    afx_msg void OnBnClickedBtnClear();
    afx_msg void OnBnClickedBtnEnd();
    afx_msg void OnBnClickedBtnSearch();
    afx_msg void OnBnClickedBtnMove();
    afx_msg void OnBnClickedBtnStop();
    afx_msg BOOL OnEraseBkgnd(CDC* pDC);
    afx_msg void OnBnClickedBtnRandom();
    afx_msg void OnBnClickedBtnPrevFloor();
    afx_msg void OnBnClickedBtnNextFloor();
    afx_msg void OnBnClickedBtnStart();
    afx_msg void OnBnClickedBtnElevator();
    afx_msg void OnBnClickedBtnAccelerate();
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnStnClickedStaticCols();

private:
    HICON m_hIcon;
    // 获取某层所有活跃动态障碍的位置集合
    void GetOccupiedSet(int floor, set<pair<int, int>>& occupied, const DynObstacle* skip = nullptr);
};