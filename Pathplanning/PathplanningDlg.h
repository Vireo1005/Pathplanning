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

// 带楼层信息的路径节点
struct PathNode {
    int floor;  
    int x;    
    int y;      
};

// 终点信息（每个终点可以在任意一层的任意位置）
struct GoalPoint {
    int floor;
    int x;
    int y;
};

// 动态障碍（每层一个）
struct DynObstacle {
    int   floor;          // 所在楼层(固定不变)
    Point pos;            // 当前位置
    bool  useDFS;         // true=奇数层(DFS朝起点), false=偶数层(随机游走)
    vector<Point> path;   // 奇数层的DFS路径
    int   idx;            // 奇数层路径进度
    Point last;           // 偶数层随机游走时的上一格(尽量不回头)

    bool      active;         // true = 障碍存在，false = 消失冷却中
    ULONGLONG vanishTime;     // 消失时的系统时间（仅在 !active 时有效）

    DynObstacle() : floor(0), pos(0, 0), useDFS(false), idx(0), last(0, 0) {}
};

// CPathPlanningDlg 对话框
class CPathPlanningDlg : public CDialogEx
{
public:
    CPathPlanningDlg(CWnd* pParent = nullptr);

    // 对话框数据
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_PATHPLANNING_DIALOG };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);

protected:
    HICON m_hIcon;

    // 生成的消息映射函数
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
    int m_nGoalCount;;

    CComboBox m_cmbAlgorithm;

    //每一层有自己的障碍物地图
    vector<PathPlanner>m_floors;

    //这里设置起点默认为第一层左上角位置
    Point m_start;
	int m_startFloor;
    Point m_elevator;

    //所有终点位置集合
	vector<GoalPoint> m_goals; 

	//完整路径集合（跨层）
    vector<PathNode>m_fullPath;
	bool m_bHasPath;

    // 编辑模式
    EditMode m_editMode;

    // 机器人相关
    int m_nRobotIndex;   // 当前机器人在路径中的位置
    bool m_bRobotMoving;
    UINT m_nTimerID;
    int m_nSpeed;        // 移动速度（毫秒）

    bool m_bAccelerated;    // 是否处于加速状态
    int  m_nNormalSpeed;    // 正常速度（加速前的速度）

    // ===== 实时路径（机器人始终在路径上） =====
    vector<PathNode> m_traveledPath;  // 已走过的轨迹
    vector<PathNode> m_aheadPath;     // 当前位置到下一关键点的实时前方路径

    // 规划一条完整的避障路径（返回整条路径，不止一步）
    bool PlanAheadFull(int floor, Point s, Point e,
        const std::set<std::pair<int, int>>& blocked, vector<Point>& fullPath);

    // ===== 动态障碍 =====
    bool             m_bDynEnabled;   // 是否启用动态障碍（默认开）
    vector<DynObstacle> m_dynObs;   // 每层一个动态障碍

    // ===== 机器人反应式移动 =====
    PathNode         m_robotPos;      // 机器人当前实际位置（会绕路，不再照抄m_fullPath）
    vector<PathNode> m_waypoints;     // 必经关键点：起点->各终点->起点
    int              m_wpIndex;       // 当前目标关键点下标
    bool             m_bWaiting;      // 是否被堵、原地等待

    vector<int> m_legOf;      // m_fullPath 每个节点属于第几段（用于分色画路径）
    int         m_nStepPerTick; // 机器人每帧前进的格子数（自适应加速）

    // 绘图区域
    CRect m_mapRect;
    int m_cellSize;

    // 计算单元格大小和地图区域
    void CalcMapLayout();

    // 屏幕坐标转网格坐标
    CPoint ScreenToGrid(CPoint point);

    // 网格坐标转屏幕坐标（左上角）
    CPoint GridToScreen(int row, int col);

    //电梯位置
    Point GetElevator();

    // 绘制地图
    void DrawMap(CDC* pDC);

    // 绘制网格
    void DrawGrid(CDC* pDC);

    // 绘制障碍物
    void DrawObstacles(CDC* pDC);

    // 绘制起点和终点
    void DrawStartEnd(CDC* pDC);

    // 绘制路径
    void DrawPath(CDC* pDC);

    // 绘制机器人
    void DrawRobot(CDC* pDC);

    //绘制电梯
	void DrawElevator(CDC* pDC);

    //绘制路线
    void     DrawArrowHead(CDC* pDC, CPoint from, CPoint to); 
    COLORREF LegColor(int leg); 

    void InitDynObstacles();              // 按层数创建每层一个动态障碍
    void SpawnOneDynamic(DynObstacle& ob);// (重新)随机生成一个障碍
    void RandomWalkStep(DynObstacle& ob); // 偶数层随机游走一步
    void DrawDynamic(CDC* pDC);                 // 画动态障碍
    void StepDynamic();                         // 动态障碍走一步
    void StepRobot();                           // 机器人反应式走一步(核心)
    bool PlanAvoidStep(int floor, Point s, Point e,
        const std::set<std::pair<int, int>>& blocked, Point& nextCell);

    bool FindPathOnFloor(int floor, Point s, Point e, vector<Point>& path,int algo); 
    bool GetSegmentPath(const PathNode& a, const PathNode& b, vector<PathNode>& seg,int algo);
    bool PlanRoute(int algo);  
    void ToggleGoal(int floor, int r, int c); 
    void SetStartAt(int floor, int r, int c);
    void SetElevatorAt(int r, int c);

    // 按钮响应函数
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
    // 鼠标事件
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);

    // 定时器
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnStnClickedStaticCols();
};
