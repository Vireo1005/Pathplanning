#include "pch.h"
#include "framework.h"
#include "PathPlanning.h"
#include "PathPlanningDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CPathPlanningDlg 对话框

CPathPlanningDlg::CPathPlanningDlg(CWnd* pParent /*=nullptr*/)
    : CDialogEx(IDD_PATHPLANNING_DIALOG, pParent)
    , m_nRows(10)
    , m_nCols(10)
    , m_bHasPath(false)
    , m_editMode(MODE_NONE)
    , m_nRobotIndex(0)
    , m_bRobotMoving(false)
    , m_nTimerID(0)
    , m_nSpeed(200)
    , m_cellSize(30)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPathPlanningDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Text(pDX, IDC_EDIT_ROWS, m_nRows);
    DDX_Text(pDX, IDC_EDIT_COLS, m_nCols);
    DDV_MinMaxInt(pDX, m_nRows, 3, 30);
    DDV_MinMaxInt(pDX, m_nCols, 3, 30);
    DDX_Control(pDX, IDC_CMB_ALGORITHM, m_cmbAlgorithm);
}

BEGIN_MESSAGE_MAP(CPathPlanningDlg, CDialogEx)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_BN_CLICKED(IDC_BTN_CREATE, &CPathPlanningDlg::OnBnClickedBtnCreate)
    ON_BN_CLICKED(IDC_BTN_OBSTACLE, &CPathPlanningDlg::OnBnClickedBtnObstacle)
    ON_BN_CLICKED(IDC_BTN_ERASE, &CPathPlanningDlg::OnBnClickedBtnErase)
    ON_BN_CLICKED(IDC_BTN_CLEAR, &CPathPlanningDlg::OnBnClickedBtnClear)
    ON_BN_CLICKED(IDC_BTN_START, &CPathPlanningDlg::OnBnClickedBtnStart)
    ON_BN_CLICKED(IDC_BTN_END, &CPathPlanningDlg::OnBnClickedBtnEnd)
    ON_BN_CLICKED(IDC_BTN_SEARCH, &CPathPlanningDlg::OnBnClickedBtnSearch)
    ON_BN_CLICKED(IDC_BTN_MOVE, &CPathPlanningDlg::OnBnClickedBtnMove)
    ON_BN_CLICKED(IDC_BTN_STOP, &CPathPlanningDlg::OnBnClickedBtnStop)
    ON_WM_LBUTTONDOWN()
    ON_WM_MOUSEMOVE()
    ON_WM_TIMER()
END_MESSAGE_MAP()

// CPathPlanningDlg 初始化

BOOL CPathPlanningDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // 设置图标
    SetIcon(m_hIcon, TRUE);
    SetIcon(m_hIcon, FALSE);

    // 初始化算法下拉框
    m_cmbAlgorithm.AddString(_T("BFS 广度优先"));
    m_cmbAlgorithm.AddString(_T("A* 算法"));
    m_cmbAlgorithm.SetCurSel(0);

    // 初始化地图
    m_planner.SetMapSize(m_nRows, m_nCols);
    m_planner.SetStart(0, 0);
    m_planner.SetEnd(m_nRows - 1, m_nCols - 1);

    // 计算地图布局
    CalcMapLayout();

    return TRUE;
}

void CPathPlanningDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    CDialogEx::OnSysCommand(nID, lParam);
}

// 计算地图布局
void CPathPlanningDlg::CalcMapLayout()
{
    CRect clientRect;
    GetClientRect(&clientRect);

    // 地图区域：左边留出控制面板宽度
    int panelWidth = 200;
    int margin = 20;

    int mapWidth = clientRect.Width() - panelWidth - margin * 2;
    int mapHeight = clientRect.Height() - margin * 2;

    // 计算单元格大小
    int cellW = mapWidth / m_nCols;
    int cellH = mapHeight / m_nRows;
    m_cellSize = min(cellW, cellH);
    if (m_cellSize < 10) m_cellSize = 10;

    // 地图实际大小
    int actualW = m_cellSize * m_nCols;
    int actualH = m_cellSize * m_nRows;

    // 居中显示
    int left = panelWidth + margin + (mapWidth - actualW) / 2;
    int top = margin + (mapHeight - actualH) / 2;

    m_mapRect.SetRect(left, top, left + actualW, top + actualH);
}

// 屏幕坐标转网格坐标
CPoint CPathPlanningDlg::ScreenToGrid(CPoint point)
{
    if (!m_mapRect.PtInRect(point)) {
        return CPoint(-1, -1);
    }
    int row = (point.y - m_mapRect.top) / m_cellSize;
    int col = (point.x - m_mapRect.left) / m_cellSize;
    return CPoint(row, col);
}

// 网格坐标转屏幕坐标
CPoint CPathPlanningDlg::GridToScreen(int row, int col)
{
    return CPoint(
        m_mapRect.left + col * m_cellSize,
        m_mapRect.top + row * m_cellSize
    );
}

// 绘制地图
void CPathPlanningDlg::DrawMap(CDC* pDC)
{
    DrawGrid(pDC);
    DrawObstacles(pDC);
    DrawPath(pDC);
    DrawStartEnd(pDC);
    DrawRobot(pDC);
}

// 绘制网格
void CPathPlanningDlg::DrawGrid(CDC* pDC)
{
    CPen pen(PS_SOLID, 1, RGB(200, 200, 200));
    CPen* oldPen = pDC->SelectObject(&pen);

    // 绘制横线
    for (int i = 0; i <= m_nRows; i++) {
        int y = m_mapRect.top + i * m_cellSize;
        pDC->MoveTo(m_mapRect.left, y);
        pDC->LineTo(m_mapRect.right, y);
    }

    // 绘制竖线
    for (int j = 0; j <= m_nCols; j++) {
        int x = m_mapRect.left + j * m_cellSize;
        pDC->MoveTo(x, m_mapRect.top);
        pDC->LineTo(x, m_mapRect.bottom);
    }

    pDC->SelectObject(oldPen);
}

// 绘制障碍物
void CPathPlanningDlg::DrawObstacles(CDC* pDC)
{
    CBrush brush(RGB(100, 100, 100));
    CBrush* oldBrush = pDC->SelectObject(&brush);

    const auto& map = m_planner.GetMap();
    for (int i = 0; i < m_nRows; i++) {
        for (int j = 0; j < m_nCols; j++) {
            if (map[i][j]) {
                CPoint pos = GridToScreen(i, j);
                pDC->Rectangle(pos.x, pos.y, pos.x + m_cellSize, pos.y + m_cellSize);
            }
        }
    }

    pDC->SelectObject(oldBrush);
}

// 绘制起点和终点
void CPathPlanningDlg::DrawStartEnd(CDC* pDC)
{
    Point start = m_planner.GetStart();
    Point end = m_planner.GetEnd();

    // 起点 - 绿色
    CBrush startBrush(RGB(0, 200, 0));
    CBrush* oldBrush = pDC->SelectObject(&startBrush);
    CPoint startPos = GridToScreen(start.x, start.y);
    pDC->Rectangle(startPos.x, startPos.y, startPos.x + m_cellSize, startPos.y + m_cellSize);

    // 终点 - 红色
    CBrush endBrush(RGB(255, 80, 80));
    pDC->SelectObject(&endBrush);
    CPoint endPos = GridToScreen(end.x, end.y);
    pDC->Rectangle(endPos.x, endPos.y, endPos.x + m_cellSize, endPos.y + m_cellSize);

    pDC->SelectObject(oldBrush);

    // 标注文字
    pDC->SetBkMode(TRANSPARENT);
    pDC->SetTextColor(RGB(255, 255, 255));

    CFont font;
    font.CreatePointFont(80, _T("Arial"));
    CFont* oldFont = pDC->SelectObject(&font);

    pDC->TextOutW(startPos.x + m_cellSize / 4, startPos.y + m_cellSize / 4, _T("S"));
    pDC->TextOutW(endPos.x + m_cellSize / 4, endPos.y + m_cellSize / 4, _T("E"));

    pDC->SelectObject(oldFont);
}

// 绘制路径
void CPathPlanningDlg::DrawPath(CDC* pDC)
{
    if (!m_bHasPath || m_path.empty()) return;

    CPen pen(PS_SOLID, 3, RGB(255, 200, 0));
    CPen* oldPen = pDC->SelectObject(&pen);

    // 绘制路径连线
    for (size_t i = 0; i < m_path.size() - 1; i++) {
        CPoint p1 = GridToScreen(m_path[i].x, m_path[i].y);
        CPoint p2 = GridToScreen(m_path[i + 1].x, m_path[i + 1].y);

        pDC->MoveTo(p1.x + m_cellSize / 2, p1.y + m_cellSize / 2);
        pDC->LineTo(p2.x + m_cellSize / 2, p2.y + m_cellSize / 2);
    }

    pDC->SelectObject(oldPen);
}

// 绘制机器人
void CPathPlanningDlg::DrawRobot(CDC* pDC)
{
    if (!m_bHasPath || m_path.empty()) return;

    int idx = m_bRobotMoving ? m_nRobotIndex : 0;
    if (idx >= (int)m_path.size()) idx = (int)m_path.size() - 1;

    Point pos = m_path[idx];
    CPoint screenPos = GridToScreen(pos.x, pos.y);

    // 绘制圆形机器人 - 蓝色
    CBrush brush(RGB(0, 120, 255));
    CBrush* oldBrush = pDC->SelectObject(&brush);

    int radius = m_cellSize / 3;
    int cx = screenPos.x + m_cellSize / 2;
    int cy = screenPos.y + m_cellSize / 2;

    pDC->Ellipse(cx - radius, cy - radius, cx + radius, cy + radius);

    pDC->SelectObject(oldBrush);
}

// 绘制消息
void CPathPlanningDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CPaintDC dc(this);
        CalcMapLayout();
        DrawMap(&dc);
        CDialogEx::OnPaint();
    }
}

HCURSOR CPathPlanningDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

// 创建地图按钮
void CPathPlanningDlg::OnBnClickedBtnCreate()
{
    UpdateData(TRUE);

    if (m_nRows < 3 || m_nRows > 30 || m_nCols < 3 || m_nCols > 30) {
        MessageBox(_T("行列数必须在3-30之间！"), _T("提示"), MB_ICONWARNING);
        return;
    }

    // 停止机器人
    if (m_bRobotMoving) {
        KillTimer(m_nTimerID);
        m_bRobotMoving = false;
    }

    m_planner.SetMapSize(m_nRows, m_nCols);
    m_planner.SetStart(0, 0);
    m_planner.SetEnd(m_nRows - 1, m_nCols - 1);
    m_bHasPath = false;
    m_path.clear();
    m_editMode = MODE_NONE;

    Invalidate();
}

// 绘制障碍物按钮
void CPathPlanningDlg::OnBnClickedBtnObstacle()
{
    m_editMode = MODE_OBSTACLE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：绘制障碍物"));
}

// 擦除障碍物按钮
void CPathPlanningDlg::OnBnClickedBtnErase()
{
    m_editMode = MODE_ERASE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：擦除障碍物"));
}

// 清除所有障碍物
void CPathPlanningDlg::OnBnClickedBtnClear()
{
    m_planner.ClearObstacles();
    m_bHasPath = false;
    m_path.clear();
    m_editMode = MODE_NONE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：无"));
    Invalidate();
}

// 设置起点按钮
void CPathPlanningDlg::OnBnClickedBtnStart()
{
    m_editMode = MODE_START;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：设置起点"));
}

// 设置终点按钮
void CPathPlanningDlg::OnBnClickedBtnEnd()
{
    m_editMode = MODE_END;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：设置终点"));
}

// 搜索路径按钮
void CPathPlanningDlg::OnBnClickedBtnSearch()
{
    // 停止机器人
    if (m_bRobotMoving) {
        KillTimer(m_nTimerID);
        m_bRobotMoving = false;
    }

    int algo = m_cmbAlgorithm.GetCurSel();
    bool success = false;

    if (algo == 0) {
        success = m_planner.BFS(m_path);
    }
    else {
        success = m_planner.AStar(m_path);
    }

    if (success) {
        m_bHasPath = true;
        m_nRobotIndex = 0;
        CString msg;
        msg.Format(_T("找到路径！共%d步"), (int)m_path.size() - 1);
        SetDlgItemText(IDC_STATIC_STATUS, msg);
    }
    else {
        m_bHasPath = false;
        m_path.clear();
        SetDlgItemText(IDC_STATIC_STATUS, _T("无法到达终点！"));
        MessageBox(_T("起点到终点之间没有可行路径！"), _T("提示"), MB_ICONWARNING);
    }

    Invalidate();
}

// 开始移动按钮
void CPathPlanningDlg::OnBnClickedBtnMove()
{
    if (!m_bHasPath || m_path.empty()) {
        MessageBox(_T("请先搜索路径！"), _T("提示"), MB_ICONWARNING);
        return;
    }

    if (m_bRobotMoving) return;

    m_nRobotIndex = 0;
    m_bRobotMoving = true;
    m_nTimerID = SetTimer(1, m_nSpeed, nullptr);
    SetDlgItemText(IDC_STATIC_STATUS, _T("机器人移动中..."));
}

// 停止移动按钮
void CPathPlanningDlg::OnBnClickedBtnStop()
{
    if (m_bRobotMoving) {
        KillTimer(m_nTimerID);
        m_bRobotMoving = false;
        SetDlgItemText(IDC_STATIC_STATUS, _T("已停止"));
        Invalidate();
    }
}

// 鼠标左键按下
void CPathPlanningDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
    if (m_editMode == MODE_NONE) return;

    CPoint grid = ScreenToGrid(point);
    if (grid.x < 0 || grid.y < 0) return;

    switch (m_editMode) {
    case MODE_OBSTACLE:
        // 不能在起点终点上画障碍
        if (!(Point(grid.x, grid.y) == m_planner.GetStart()) &&
            !(Point(grid.x, grid.y) == m_planner.GetEnd())) {
            m_planner.SetObstacle(grid.x, grid.y, true);
            m_bHasPath = false;
            m_path.clear();
        }
        break;

    case MODE_ERASE:
        m_planner.SetObstacle(grid.x, grid.y, false);
        m_bHasPath = false;
        m_path.clear();
        break;

    case MODE_START:
        if (!m_planner.GetMap()[grid.x][grid.y]) {
            m_planner.SetStart(grid.x, grid.y);
            m_bHasPath = false;
            m_path.clear();
        }
        break;

    case MODE_END:
        if (!m_planner.GetMap()[grid.x][grid.y]) {
            m_planner.SetEnd(grid.x, grid.y);
            m_bHasPath = false;
            m_path.clear();
        }
        break;
    }

    Invalidate();
    CDialogEx::OnLButtonDown(nFlags, point);
}

// 鼠标移动（拖拽绘制）
void CPathPlanningDlg::OnMouseMove(UINT nFlags, CPoint point)
{
    if (!(nFlags & MK_LBUTTON)) return;
    if (m_editMode != MODE_OBSTACLE && m_editMode != MODE_ERASE) return;

    CPoint grid = ScreenToGrid(point);
    if (grid.x < 0 || grid.y < 0) return;

    if (m_editMode == MODE_OBSTACLE) {
        if (!(Point(grid.x, grid.y) == m_planner.GetStart()) &&
            !(Point(grid.x, grid.y) == m_planner.GetEnd())) {
            m_planner.SetObstacle(grid.x, grid.y, true);
            m_bHasPath = false;
            m_path.clear();
        }
    }
    else if (m_editMode == MODE_ERASE) {
        m_planner.SetObstacle(grid.x, grid.y, false);
        m_bHasPath = false;
        m_path.clear();
    }

    Invalidate();
    CDialogEx::OnMouseMove(nFlags, point);
}

// 定时器 - 机器人移动
void CPathPlanningDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == 1 && m_bRobotMoving) {
        m_nRobotIndex++;
        if (m_nRobotIndex >= (int)m_path.size()) {
            KillTimer(m_nTimerID);
            m_bRobotMoving = false;
            SetDlgItemText(IDC_STATIC_STATUS, _T("到达终点！"));
        }
        Invalidate();
    }

    CDialogEx::OnTimer(nIDEvent);
}
