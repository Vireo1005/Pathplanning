#pragma once
#include "AStar.h"
#include "afxwin.h"

// 编辑模式枚举
enum EditMode {
    MODE_NONE = 0,
    MODE_OBSTACLE,   // 绘制障碍物
    MODE_START,      // 设置起点
    MODE_END,        // 设置终点
    MODE_ERASE       // 擦除障碍物
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
    CComboBox m_cmbAlgorithm;

    // 路径规划器
    PathPlanner m_planner;

    // 当前路径
    vector<Point> m_path;
    bool m_bHasPath;

    // 编辑模式
    EditMode m_editMode;

    // 机器人相关
    int m_nRobotIndex;   // 当前机器人在路径中的位置
    bool m_bRobotMoving;
    UINT m_nTimerID;
    int m_nSpeed;        // 移动速度（毫秒）

    // 绘图区域
    CRect m_mapRect;
    int m_cellSize;

    // 计算单元格大小和地图区域
    void CalcMapLayout();

    // 屏幕坐标转网格坐标
    CPoint ScreenToGrid(CPoint point);

    // 网格坐标转屏幕坐标（左上角）
    CPoint GridToScreen(int row, int col);

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

    // 按钮响应函数
    afx_msg void OnBnClickedBtnCreate();
    afx_msg void OnBnClickedBtnObstacle();
    afx_msg void OnBnClickedBtnErase();
    afx_msg void OnBnClickedBtnClear();
    afx_msg void OnBnClickedBtnStart();
    afx_msg void OnBnClickedBtnEnd();
    afx_msg void OnBnClickedBtnSearch();
    afx_msg void OnBnClickedBtnMove();
    afx_msg void OnBnClickedBtnStop();

    // 鼠标事件
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
    afx_msg void OnMouseMove(UINT nFlags, CPoint point);

    // 定时器
    afx_msg void OnTimer(UINT_PTR nIDEvent);
};
