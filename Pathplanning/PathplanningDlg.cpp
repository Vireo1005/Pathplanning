#include "pch.h"
#include "framework.h"
#include "PathPlanning.h"
#include "PathPlanningDlg.h"
#include "afxdialogex.h"
#include <set>
#include <queue>
#include <stack>
#include <functional>
#include <algorithm>
#include <climits>
#include <cmath>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CPathPlanningDlg::CPathPlanningDlg(CWnd* pParent /*=nullptr*/)
    : CDialogEx(IDD_PATHPLANNING_DIALOG, pParent)
    , m_nRows(10), m_nCols(10), m_nFloors(3), m_nCurFloor(0), m_nGoalCount(3)
    , m_bHasPath(false), m_editMode(MODE_NONE)
    , m_nRobotIndex(0), m_bRobotMoving(false), m_nTimerID(0), m_nSpeed(150)
    , m_cellSize(30), m_startFloor(0)
    , m_bDynEnabled(true), m_wpIndex(0), m_bWaiting(false)
    , m_bAccelerated(false), m_nNormalSpeed(150)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    m_start = Point(0, 0);
    m_elevator = Point(m_nRows - 1, m_nCols - 1);
    m_robotPos.floor = 0; m_robotPos.x = 0; m_robotPos.y = 0;
}

void CPathPlanningDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Text(pDX, IDC_EDIT_ROWS, m_nRows);
    DDX_Text(pDX, IDC_EDIT_COLS, m_nCols);
    DDV_MinMaxInt(pDX, m_nRows, 3, 50);
    DDV_MinMaxInt(pDX, m_nCols, 3, 50);
    DDX_Text(pDX, IDC_EDIT_FLOORS, m_nFloors);
    DDV_MinMaxInt(pDX, m_nFloors, 1, 10);
    DDX_Text(pDX, IDC_EDIT_GOALCOUNT, m_nGoalCount);
    DDV_MinMaxInt(pDX, m_nGoalCount, 1, 5);
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
    ON_BN_CLICKED(IDC_BTN_END, &CPathPlanningDlg::OnBnClickedBtnEnd)
    ON_BN_CLICKED(IDC_BTN_SEARCH, &CPathPlanningDlg::OnBnClickedBtnSearch)
    ON_BN_CLICKED(IDC_BTN_MOVE, &CPathPlanningDlg::OnBnClickedBtnMove)
    ON_BN_CLICKED(IDC_BTN_STOP, &CPathPlanningDlg::OnBnClickedBtnStop)
    ON_BN_CLICKED(IDC_BTN_RANDOM, &CPathPlanningDlg::OnBnClickedBtnRandom)
    ON_BN_CLICKED(IDC_BTN_PREVFLOOR, &CPathPlanningDlg::OnBnClickedBtnPrevFloor)
    ON_BN_CLICKED(IDC_BTN_NEXTFLOOR, &CPathPlanningDlg::OnBnClickedBtnNextFloor)
    ON_BN_CLICKED(IDC_BTN_START, &CPathPlanningDlg::OnBnClickedBtnStart)
    ON_BN_CLICKED(IDC_BTN_ELEVATOR, &CPathPlanningDlg::OnBnClickedBtnElevator)
    ON_BN_CLICKED(IDC_BTN_ACCELERATE, &CPathPlanningDlg::OnBnClickedBtnAccelerate)
    ON_WM_LBUTTONDOWN()
    ON_WM_MOUSEMOVE()
    ON_WM_TIMER()
    ON_WM_ERASEBKGND()
    ON_STN_CLICKED(IDC_STATIC_COLS, &CPathPlanningDlg::OnStnClickedStaticCols)
END_MESSAGE_MAP()

BOOL CPathPlanningDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();
    ModifyStyle(0, WS_CLIPCHILDREN);
    SetIcon(m_hIcon, TRUE);
    SetIcon(m_hIcon, FALSE);

    m_cmbAlgorithm.AddString(_T("BFS 广度优先"));
    m_cmbAlgorithm.AddString(_T("A* 算法"));
    m_cmbAlgorithm.AddString(_T("Dijkstra 算法"));
    m_cmbAlgorithm.AddString(_T("DFS 深度优先"));
    m_cmbAlgorithm.SetCurSel(0);

    m_floors.clear();
    m_floors.resize(m_nFloors);
    for (int f = 0; f < m_nFloors; f++)
        m_floors[f].SetMapSize(m_nRows, m_nCols);

    m_start = Point(0, 0);
    m_startFloor = 0;
    m_elevator = Point(m_nRows - 1, m_nCols - 1);
    m_goals.clear();
    CalcMapLayout();
    SetDlgItemText(IDC_STATIC_STATUS, _T("已初始化，当前第 1 层"));
    return TRUE;
}

void CPathPlanningDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    CDialogEx::OnSysCommand(nID, lParam);
}

Point CPathPlanningDlg::GetElevator() { return m_elevator; }

void CPathPlanningDlg::CalcMapLayout()
{
    CRect clientRect;
    GetClientRect(&clientRect);
    int panelWidth = 200, margin = 20;
    int mapWidth = clientRect.Width() - panelWidth - margin * 2;
    int mapHeight = clientRect.Height() - margin * 2;
    m_cellSize = min(mapWidth / m_nCols, mapHeight / m_nRows);
    if (m_cellSize < 10) m_cellSize = 10;
    int actualW = m_cellSize * m_nCols, actualH = m_cellSize * m_nRows;
    int left = panelWidth + margin + (mapWidth - actualW) / 2;
    int top = margin + (mapHeight - actualH) / 2;
    m_mapRect.SetRect(left, top, left + actualW, top + actualH);
}

CPoint CPathPlanningDlg::ScreenToGrid(CPoint point)
{
    if (!m_mapRect.PtInRect(point)) return CPoint(-1, -1);
    return CPoint((point.y - m_mapRect.top) / m_cellSize, (point.x - m_mapRect.left) / m_cellSize);
}

CPoint CPathPlanningDlg::GridToScreen(int row, int col)
{
    return CPoint(m_mapRect.left + col * m_cellSize, m_mapRect.top + row * m_cellSize);
}

void CPathPlanningDlg::DrawMap(CDC* pDC)
{
    DrawGrid(pDC);
    DrawObstacles(pDC);
    DrawElevator(pDC);
    DrawPath(pDC);
    DrawStartEnd(pDC);
    DrawRobot(pDC);
    DrawDynamic(pDC);
}

void CPathPlanningDlg::DrawGrid(CDC* pDC)
{
    CPen pen(PS_SOLID, 1, RGB(200, 200, 200));
    CPen* oldPen = pDC->SelectObject(&pen);
    for (int i = 0; i <= m_nRows; i++) {
        int y = m_mapRect.top + i * m_cellSize;
        pDC->MoveTo(m_mapRect.left, y);
        pDC->LineTo(m_mapRect.right, y);
    }
    for (int j = 0; j <= m_nCols; j++) {
        int x = m_mapRect.left + j * m_cellSize;
        pDC->MoveTo(x, m_mapRect.top);
        pDC->LineTo(x, m_mapRect.bottom);
    }
    pDC->SelectObject(oldPen);
}

void CPathPlanningDlg::DrawObstacles(CDC* pDC)
{
    CBrush brush(RGB(100, 100, 100));
    CBrush* oldBrush = pDC->SelectObject(&brush);
    const auto& map = m_floors[m_nCurFloor].GetMap();
    for (int i = 0; i < m_nRows; i++)
        for (int j = 0; j < m_nCols; j++)
            if (map[i][j]) {
                CPoint pos = GridToScreen(i, j);
                pDC->Rectangle(pos.x, pos.y, pos.x + m_cellSize, pos.y + m_cellSize);
            }
    pDC->SelectObject(oldBrush);
}

void CPathPlanningDlg::DrawStartEnd(CDC* pDC)
{
    pDC->SetBkMode(TRANSPARENT);
    CFont font;
    font.CreatePointFont(90, _T("Arial"));
    CFont* oldFont = pDC->SelectObject(&font);

    if (m_nCurFloor == m_startFloor) {
        CBrush b(RGB(0, 200, 0));
        CBrush* ob = pDC->SelectObject(&b);
        CPoint p = GridToScreen(m_start.x, m_start.y);
        pDC->Rectangle(p.x, p.y, p.x + m_cellSize, p.y + m_cellSize);
        pDC->SelectObject(ob);
        pDC->SetTextColor(RGB(255, 255, 255));
        pDC->TextOutW(p.x + m_cellSize / 4, p.y + m_cellSize / 4, _T("S"));
    }

    CBrush eb(RGB(255, 80, 80));
    CBrush* ob = pDC->SelectObject(&eb);
    for (size_t i = 0; i < m_goals.size(); i++) {
        if (m_goals[i].floor != m_nCurFloor) continue;
        CPoint p = GridToScreen(m_goals[i].x, m_goals[i].y);
        pDC->Rectangle(p.x, p.y, p.x + m_cellSize, p.y + m_cellSize);
        pDC->SetTextColor(RGB(255, 255, 255));
        CString t; t.Format(_T("%d"), (int)i + 1);
        pDC->TextOutW(p.x + m_cellSize / 4, p.y + m_cellSize / 4, t);
    }
    pDC->SelectObject(ob);
    pDC->SelectObject(oldFont);
}

void CPathPlanningDlg::DrawElevator(CDC* pDC)
{
    Point elev = GetElevator();
    CBrush b(RGB(150, 80, 220));
    CBrush* ob = pDC->SelectObject(&b);
    CPoint p = GridToScreen(elev.x, elev.y);
    pDC->Rectangle(p.x, p.y, p.x + m_cellSize, p.y + m_cellSize);
    pDC->SelectObject(ob);
    pDC->SetBkMode(TRANSPARENT);
    pDC->SetTextColor(RGB(255, 255, 255));
    CFont font; font.CreatePointFont(80, _T("Arial"));
    CFont* of = pDC->SelectObject(&font);
    pDC->TextOutW(p.x + m_cellSize / 5, p.y + m_cellSize / 4, _T("电"));
    pDC->SelectObject(of);
}

COLORREF CPathPlanningDlg::LegColor(int leg)
{
    static const COLORREF pal[6] = {
        RGB(255, 140, 0), RGB(0, 170, 0), RGB(0, 120, 255),
        RGB(200, 0, 200), RGB(210, 170, 0), RGB(0, 170, 170)
    };
    return pal[(leg % 6 + 6) % 6];
}

void CPathPlanningDlg::DrawArrowHead(CDC* pDC, CPoint from, CPoint to)
{
    const double PI = 3.14159265358979;
    double ang = atan2((double)(to.y - from.y), (double)(to.x - from.x));
    int len = max(m_cellSize / 3, 4);
    double a1 = ang + PI - PI / 6.0, a2 = ang + PI + PI / 6.0;
    CPoint w1(to.x + (int)(len * cos(a1)), to.y + (int)(len * sin(a1)));
    CPoint w2(to.x + (int)(len * cos(a2)), to.y + (int)(len * sin(a2)));
    pDC->MoveTo(to); pDC->LineTo(w1);
    pDC->MoveTo(to); pDC->LineTo(w2);
}

void CPathPlanningDlg::DrawPath(CDC* pDC)
{
    // 已走路径（蓝色实线）
    if (m_traveledPath.size() >= 2) {
        CPen pen(PS_SOLID, 3, RGB(0, 120, 255));
        CPen* oldPen = pDC->SelectObject(&pen);
        for (size_t i = 0; i + 1 < m_traveledPath.size(); i++) {
            const PathNode& a = m_traveledPath[i], & b = m_traveledPath[i + 1];
            if (a.floor != m_nCurFloor || b.floor != m_nCurFloor) continue;
            CPoint c1 = GridToScreen(a.x, a.y), c2 = GridToScreen(b.x, b.y);
            c1.Offset(m_cellSize / 2, m_cellSize / 2);
            c2.Offset(m_cellSize / 2, m_cellSize / 2);
            pDC->MoveTo(c1); pDC->LineTo(c2);
        }
        pDC->SelectObject(oldPen);
    }
    // 前方实时路径（橙色虚线）
    if (m_aheadPath.size() >= 2) {
        CPen pen(PS_DASH, 2, RGB(255, 140, 0));
        CPen* oldPen = pDC->SelectObject(&pen);
        for (size_t i = 0; i + 1 < m_aheadPath.size(); i++) {
            const PathNode& a = m_aheadPath[i], & b = m_aheadPath[i + 1];
            if (a.floor != m_nCurFloor || b.floor != m_nCurFloor) continue;
            CPoint c1 = GridToScreen(a.x, a.y), c2 = GridToScreen(b.x, b.y);
            c1.Offset(m_cellSize / 2, m_cellSize / 2);
            c2.Offset(m_cellSize / 2, m_cellSize / 2);
            pDC->MoveTo(c1); pDC->LineTo(c2);
        }
        pDC->SelectObject(oldPen);
    }
}

void CPathPlanningDlg::DrawRobot(CDC* pDC)
{
    if (!m_bHasPath || m_robotPos.floor != m_nCurFloor) return;
    CBrush brush(m_bWaiting ? RGB(150, 150, 150) : RGB(0, 120, 255));
    CBrush* oldBrush = pDC->SelectObject(&brush);
    CPoint sp = GridToScreen(m_robotPos.x, m_robotPos.y);
    int r = m_cellSize / 3, cx = sp.x + m_cellSize / 2, cy = sp.y + m_cellSize / 2;
    pDC->Ellipse(cx - r, cy - r, cx + r, cy + r);
    pDC->SelectObject(oldBrush);
}

void CPathPlanningDlg::DrawDynamic(CDC* pDC)
{
    if (!m_bDynEnabled) return;
    for (auto& ob : m_dynObs) {
        if (!ob.active || ob.floor != m_nCurFloor) continue;
        CPoint sp = GridToScreen(ob.pos.x, ob.pos.y);
        int cx = sp.x + m_cellSize / 2, cy = sp.y + m_cellSize / 2;
        int r = m_cellSize / 3;

        CBrush brush(RGB(220, 30, 30));
        CBrush* obp = pDC->SelectObject(&brush);
        pDC->Ellipse(cx - r, cy - r, cx + r, cy + r);
        pDC->SelectObject(obp);

        CPen pen(PS_SOLID, 2, RGB(255, 255, 255));
        CPen* op = pDC->SelectObject(&pen);
        int d = r / 2;
        pDC->MoveTo(cx - d, cy - d); pDC->LineTo(cx + d, cy + d);
        pDC->MoveTo(cx + d, cy - d); pDC->LineTo(cx - d, cy + d);
        pDC->SelectObject(op);
    }
}

void CPathPlanningDlg::OnPaint()
{
    if (IsIconic()) {
        CPaintDC dc(this);
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
        int cx = GetSystemMetrics(SM_CXICON), cy = GetSystemMetrics(SM_CYICON);
        CRect rect; GetClientRect(&rect);
        dc.DrawIcon((rect.Width() - cx + 1) / 2, (rect.Height() - cy + 1) / 2, m_hIcon);
    }
    else {
        CPaintDC dc(this);
        CalcMapLayout();
        CDC memDC; CBitmap memBmp;
        CRect clientRect; GetClientRect(&clientRect);
        memDC.CreateCompatibleDC(&dc);
        memBmp.CreateCompatibleBitmap(&dc, clientRect.Width(), clientRect.Height());
        CBitmap* pOldBmp = memDC.SelectObject(&memBmp);
        memDC.FillSolidRect(&clientRect, GetSysColor(COLOR_3DFACE));
        DrawMap(&memDC);
        dc.BitBlt(0, 0, clientRect.Width(), clientRect.Height(), &memDC, 0, 0, SRCCOPY);
        memDC.SelectObject(pOldBmp);
        memBmp.DeleteObject();
        memDC.DeleteDC();
        CDialogEx::OnPaint();
    }
}

HCURSOR CPathPlanningDlg::OnQueryDragIcon() { return static_cast<HCURSOR>(m_hIcon); }

bool CPathPlanningDlg::FindPathOnFloor(int floor, Point s, Point e, vector<Point>& path, int algo)
{
    path.clear();
    if (floor < 0 || floor >= m_nFloors) return false;
    const auto& map = m_floors[floor].GetMap();
    int R = m_nRows, C = m_nCols;
    if (s.x < 0 || s.x >= R || s.y < 0 || s.y >= C) return false;
    if (e.x < 0 || e.x >= R || e.y < 0 || e.y >= C) return false;
    if (map[s.x][s.y] || map[e.x][e.y]) return false;

    vector<vector<bool>>  visited(R, vector<bool>(C, false));
    vector<vector<Point>> parent(R, vector<Point>(C, Point(-1, -1)));
    int dx[4] = { -1, 1, 0, 0 }, dy[4] = { 0, 0, -1, 1 };
    bool found = false;

    if (algo == 0) { // BFS
        queue<Point> q; q.push(s); visited[s.x][s.y] = true;
        while (!q.empty()) {
            Point cur = q.front(); q.pop();
            if (cur == e) { found = true; break; }
            for (int d = 0; d < 4; d++) {
                int nx = cur.x + dx[d], ny = cur.y + dy[d];
                if (nx < 0 || nx >= R || ny < 0 || ny >= C) continue;
                if (visited[nx][ny] || map[nx][ny]) continue;
                visited[nx][ny] = true; parent[nx][ny] = cur;
                q.push(Point(nx, ny));
            }
        }
    }
    else if (algo == 3) { // DFS
        stack<Point> st; st.push(s); visited[s.x][s.y] = true;
        while (!st.empty()) {
            Point cur = st.top(); st.pop();
            if (cur == e) { found = true; break; }
            for (int d = 0; d < 4; d++) {
                int nx = cur.x + dx[d], ny = cur.y + dy[d];
                if (nx < 0 || nx >= R || ny < 0 || ny >= C) continue;
                if (visited[nx][ny] || map[nx][ny]) continue;
                visited[nx][ny] = true; parent[nx][ny] = cur;
                st.push(Point(nx, ny));
            }
        }
    }
    else { // A*(1) / Dijkstra(2)
        vector<vector<int>> g(R, vector<int>(C, INT_MAX));
        typedef pair<int, int> PQItem;
        priority_queue<PQItem, vector<PQItem>, greater<PQItem>> pq;
        g[s.x][s.y] = 0;
        int h0 = (algo == 1) ? (abs(s.x - e.x) + abs(s.y - e.y)) : 0;
        pq.push(make_pair(h0, s.x * C + s.y));
        while (!pq.empty()) {
            int code = pq.top().second; pq.pop();
            Point cur(code / C, code % C);
            if (visited[cur.x][cur.y]) continue;
            visited[cur.x][cur.y] = true;
            if (cur == e) { found = true; break; }
            for (int d = 0; d < 4; d++) {
                int nx = cur.x + dx[d], ny = cur.y + dy[d];
                if (nx < 0 || nx >= R || ny < 0 || ny >= C) continue;
                if (map[nx][ny] || visited[nx][ny]) continue;
                int ng = g[cur.x][cur.y] + 1;
                if (ng < g[nx][ny]) {
                    g[nx][ny] = ng; parent[nx][ny] = cur;
                    int h = (algo == 1) ? (abs(nx - e.x) + abs(ny - e.y)) : 0;
                    pq.push(make_pair(ng + h, nx * C + ny));
                }
            }
        }
    }

    if (!found) return false;
    vector<Point> rev; Point cur = e;
    while (!(cur == s)) { rev.push_back(cur); cur = parent[cur.x][cur.y]; }
    rev.push_back(s);
    for (int i = (int)rev.size() - 1; i >= 0; i--) path.push_back(rev[i]);
    return true;
}

bool CPathPlanningDlg::GetSegmentPath(const PathNode& a, const PathNode& b, vector<PathNode>& seg, int algo)
{
    seg.clear();
    Point pa(a.x, a.y), pb(b.x, b.y);
    if (a.floor == b.floor) {
        vector<Point> p;
        if (!FindPathOnFloor(a.floor, pa, pb, p, algo)) return false;
        for (auto& q : p) { PathNode n = { a.floor, q.x, q.y }; seg.push_back(n); }
        return true;
    }
    else {
        Point elev = GetElevator();
        vector<Point> p1, p2;
        if (!FindPathOnFloor(a.floor, pa, elev, p1, algo)) return false;
        if (!FindPathOnFloor(b.floor, elev, pb, p2, algo)) return false;
        for (auto& q : p1) { PathNode n = { a.floor, q.x, q.y }; seg.push_back(n); }
        for (auto& q : p2) { PathNode n = { b.floor, q.x, q.y }; seg.push_back(n); }
        return true;
    }
}

bool CPathPlanningDlg::PlanRoute(int algo)
{
    m_fullPath.clear();
    int N = (int)m_goals.size();
    if (N == 0) return false;

    vector<PathNode> pts;
    PathNode sp = { m_startFloor, m_start.x, m_start.y };
    pts.push_back(sp);
    for (auto& g : m_goals) { PathNode n = { g.floor, g.x, g.y }; pts.push_back(n); }

    int total = N + 1;
    vector<vector<int>> dist(total, vector<int>(total, -1));
    for (int i = 0; i < total; i++)
        for (int j = 0; j < total; j++) {
            if (i == j) { dist[i][j] = 0; continue; }
            vector<PathNode> seg;
            dist[i][j] = GetSegmentPath(pts[i], pts[j], seg, algo) ? (int)seg.size() - 1 : -1;
        }

    vector<int> order(N);
    for (int i = 0; i < N; i++) order[i] = i + 1;
    vector<int> bestOrder; int bestCost = INT_MAX;
    do {
        int cost = 0, prev = 0; bool ok = true;
        for (int k = 0; k < N; k++) {
            int cur = order[k];
            if (dist[prev][cur] < 0) { ok = false; break; }
            cost += dist[prev][cur]; prev = cur;
        }
        if (ok && dist[prev][0] >= 0) {
            cost += dist[prev][0];
            if (cost < bestCost) { bestCost = cost; bestOrder = order; }
        }
    } while (next_permutation(order.begin(), order.end()));

    if (bestOrder.empty()) return false;

    m_waypoints.clear();
    m_waypoints.push_back(sp);
    for (int v : bestOrder) m_waypoints.push_back(pts[v]);
    m_waypoints.push_back(sp);

    vector<int> tour; tour.push_back(0);
    for (int v : bestOrder) tour.push_back(v);
    tour.push_back(0);

    m_legOf.clear();
    for (size_t k = 0; k + 1 < tour.size(); k++) {
        vector<PathNode> seg;
        GetSegmentPath(pts[tour[k]], pts[tour[k + 1]], seg, algo);
        size_t startIdx = (k == 0) ? 0 : 1;
        for (size_t s2 = startIdx; s2 < seg.size(); s2++) {
            m_fullPath.push_back(seg[s2]);
            m_legOf.push_back((int)k);
        }
    }
    return true;
}

// 编辑操作
void CPathPlanningDlg::ToggleGoal(int floor, int r, int c)
{
    Point elev = GetElevator();
    if (floor == m_startFloor && r == m_start.x && c == m_start.y) return;
    if (r == elev.x && c == elev.y) return;
    if (m_floors[floor].GetMap()[r][c]) return;

    for (size_t i = 0; i < m_goals.size(); i++)
        if (m_goals[i].floor == floor && m_goals[i].x == r && m_goals[i].y == c) {
            m_goals.erase(m_goals.begin() + i);
            m_bHasPath = false; m_fullPath.clear();
            return;
        }
    if ((int)m_goals.size() >= m_nGoalCount) {
        MessageBox(_T("终点数量已达上限，请先移除或增大终点数量！"), _T("提示"), MB_ICONWARNING);
        return;
    }
    GoalPoint g = { floor, r, c };
    m_goals.push_back(g);
    m_bHasPath = false; m_fullPath.clear();
}

void CPathPlanningDlg::SetStartAt(int floor, int r, int c)
{
    Point elev = GetElevator();
    if (r == elev.x && c == elev.y) return;
    if (m_floors[floor].GetMap()[r][c]) return;
    for (auto& g : m_goals) if (g.floor == floor && g.x == r && g.y == c) return;
    m_startFloor = floor; m_start = Point(r, c);
    m_bHasPath = false; m_fullPath.clear();
}

void CPathPlanningDlg::SetElevatorAt(int r, int c)
{
    if (r == m_start.x && c == m_start.y) return;
    for (auto& g : m_goals) if (g.x == r && g.y == c) return;
    for (int f = 0; f < m_nFloors; f++) m_floors[f].SetObstacle(r, c, false);
    m_elevator = Point(r, c);
    m_bHasPath = false; m_fullPath.clear();
}

void CPathPlanningDlg::OnBnClickedBtnCreate()
{
    UpdateData(TRUE);
    if (m_nRows < 3 || m_nRows > 50 || m_nCols < 3 || m_nCols > 50) {
        MessageBox(_T("行列数必须在3-50之间！"), _T("提示"), MB_ICONWARNING); return;
    }
    if (m_nFloors < 1 || m_nFloors > 10) {
        MessageBox(_T("楼层数必须在1-10之间！"), _T("提示"), MB_ICONWARNING); return;
    }
    if (m_nGoalCount < 1 || m_nGoalCount > 5) {
        MessageBox(_T("终点数量必须在1-5之间！"), _T("提示"), MB_ICONWARNING); return;
    }
    if (m_bRobotMoving) { KillTimer(m_nTimerID); m_bRobotMoving = false; }

    m_bAccelerated = false; m_nSpeed = 150; m_nNormalSpeed = 150;
    m_floors.clear(); m_floors.resize(m_nFloors);
    for (int f = 0; f < m_nFloors; f++) m_floors[f].SetMapSize(m_nRows, m_nCols);

    m_nCurFloor = 0; m_start = Point(0, 0); m_startFloor = 0;
    m_elevator = Point(m_nRows - 1, m_nCols - 1);
    m_goals.clear(); m_fullPath.clear(); m_waypoints.clear();
    m_wpIndex = 0; m_bWaiting = false; m_bHasPath = false;
    m_editMode = MODE_NONE; m_dynObs.clear();
    srand((unsigned)time(nullptr));
    CalcMapLayout();

    CString s; s.Format(_T("已创建 %d 层地图，当前第 1 层"), m_nFloors);
    SetDlgItemText(IDC_STATIC_STATUS, s);
    Invalidate();
}

void CPathPlanningDlg::OnBnClickedBtnObstacle()
{
    m_editMode = MODE_OBSTACLE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：绘制障碍物"));
}

void CPathPlanningDlg::OnBnClickedBtnErase()
{
    m_editMode = MODE_ERASE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：擦除障碍物"));
}

void CPathPlanningDlg::OnBnClickedBtnClear()
{
    m_floors[m_nCurFloor].ClearObstacles();
    m_bHasPath = false; m_fullPath.clear(); m_editMode = MODE_NONE;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前层障碍物已清除"));
    Invalidate();
}

void CPathPlanningDlg::OnBnClickedBtnStart()
{
    m_editMode = MODE_START;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：设置起点（在当前层点格子放置）"));
}

void CPathPlanningDlg::OnBnClickedBtnEnd()
{
    m_nGoalCount = GetDlgItemInt(IDC_EDIT_GOALCOUNT);
    if (m_nGoalCount < 1) m_nGoalCount = 1;
    if (m_nGoalCount > 5) m_nGoalCount = 5;
    m_editMode = MODE_GOAL;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：设置终点（点格子添加/再点移除）"));
}

void CPathPlanningDlg::OnBnClickedBtnElevator()
{
    m_editMode = MODE_ELEVATOR;
    SetDlgItemText(IDC_STATIC_STATUS, _T("当前模式：设置电梯"));
}

void CPathPlanningDlg::OnBnClickedBtnSearch()
{
    if (m_bRobotMoving) { KillTimer(m_nTimerID); m_bRobotMoving = false; }
    if (m_goals.empty()) {
        MessageBox(_T("请先设置至少一个终点！"), _T("提示"), MB_ICONWARNING); return;
    }
    int algo = m_cmbAlgorithm.GetCurSel();
    if (algo < 0) algo = 0;

    bool ok = PlanRoute(algo);
    if (ok) {
        m_bHasPath = true;
        m_robotPos.floor = m_startFloor;
        m_robotPos.x = m_start.x; m_robotPos.y = m_start.y;
        m_wpIndex = 1; m_bWaiting = false;
        m_traveledPath.clear(); m_aheadPath.clear();

        int steps = (int)m_fullPath.size() - 1;
        CString algoName[4] = { _T("BFS"), _T("A*"), _T("Dijkstra"), _T("DFS") };
        CString msg;
        msg.Format(_T("【%s】规划成功！访问 %d 个终点并返回起点，总步数：%d 步"),
            (LPCTSTR)algoName[algo], (int)m_goals.size(), steps);
        SetDlgItemText(IDC_STATIC_STATUS, msg);
    }
    else {
        m_bHasPath = false; m_fullPath.clear(); m_waypoints.clear();
        SetDlgItemText(IDC_STATIC_STATUS, _T("存在无法到达的终点！"));
        MessageBox(_T("无法在起点与所有终点之间规划出完整路径！"), _T("提示"), MB_ICONWARNING);
    }
    Invalidate();
}

void CPathPlanningDlg::OnBnClickedBtnMove()
{
    if (!m_bHasPath || m_waypoints.empty()) {
        MessageBox(_T("请先搜索路径！"), _T("提示"), MB_ICONWARNING); return;
    }
    if (m_bRobotMoving) return;
    if (m_wpIndex < 1 || m_wpIndex >= (int)m_waypoints.size()) {
        m_robotPos.floor = m_startFloor;
        m_robotPos.x = m_start.x; m_robotPos.y = m_start.y;
        m_wpIndex = 1;
    }
    m_bWaiting = false;
    m_nCurFloor = m_robotPos.floor;
    srand((unsigned)time(nullptr));

    // 初始化动态障碍（每层4个）
    if (m_bDynEnabled) {
        if ((int)m_dynObs.size() != m_nFloors * DYN_PER_FLOOR) {
            m_dynObs.clear();
            InitDynObstacles();
        }
    }

    m_nSpeed = 150; m_nNormalSpeed = 150; m_bAccelerated = false;
    m_traveledPath.clear(); m_aheadPath.clear();
    m_bRobotMoving = true;
    m_nTimerID = (UINT)SetTimer(1, m_nSpeed, nullptr);
    SetDlgItemText(IDC_STATIC_STATUS, _T("机器人移动中..."));
    Invalidate();
}

void CPathPlanningDlg::OnBnClickedBtnStop()
{
    if (m_bRobotMoving) {
        KillTimer(m_nTimerID); m_bRobotMoving = false;
        SetDlgItemText(IDC_STATIC_STATUS, _T("已停止"));
        Invalidate();
    }
}

void CPathPlanningDlg::OnBnClickedBtnRandom()
{
    if (m_bRobotMoving) { KillTimer(m_nTimerID); m_bRobotMoving = false; }
    m_floors[m_nCurFloor].ClearObstacles();
    m_bHasPath = false; m_fullPath.clear();

    Point elev = GetElevator();
    vector<Point> targets;
    if (m_nCurFloor == m_startFloor) targets.push_back(m_start);
    for (auto& g : m_goals) if (g.floor == m_nCurFloor) targets.push_back(Point(g.x, g.y));

    // BFS保护路径格子
    set<pair<int, int>> protectedSet;
    protectedSet.insert({ elev.x, elev.y });
    int R = m_nRows, C = m_nCols;
    vector<vector<bool>> visited(R, vector<bool>(C, false));
    vector<vector<Point>> parent(R, vector<Point>(C, Point(-1, -1)));
    queue<Point> q; q.push(elev); visited[elev.x][elev.y] = true;

    set<pair<int, int>> targetSet;
    for (auto& t : targets) targetSet.insert({ t.x, t.y });
    int foundCount = 0;
    int dx[4] = { -1, 1, 0, 0 }, dy[4] = { 0, 0, -1, 1 };

    while (!q.empty() && foundCount < (int)targets.size()) {
        Point cur = q.front(); q.pop();
        if (targetSet.count({ cur.x, cur.y })) {
            foundCount++;
            Point p = cur;
            while (!(p == elev)) { protectedSet.insert({ p.x, p.y }); p = parent[p.x][p.y]; }
        }
        for (int d = 0; d < 4; d++) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx >= 0 && nx < R && ny >= 0 && ny < C && !visited[nx][ny]) {
                visited[nx][ny] = true; parent[nx][ny] = cur;
                q.push(Point(nx, ny));
            }
        }
    }
    for (auto& t : targets) protectedSet.insert({ t.x, t.y });

    // 随机放置障碍
    srand((unsigned)time(nullptr));
    const double density = 0.25;
    for (int r = 0; r < m_nRows; ++r)
        for (int c = 0; c < m_nCols; ++c) {
            if (protectedSet.count({ r, c }) > 0) continue;
            if (m_nCurFloor == m_startFloor && r == m_start.x && c == m_start.y) continue;
            if (r == elev.x && c == elev.y) continue;
            bool isGoal = false;
            for (auto& g : m_goals)
                if (g.floor == m_nCurFloor && g.x == r && g.y == c) { isGoal = true; break; }
            if (isGoal) continue;
            if ((double)rand() / RAND_MAX < density)
                m_floors[m_nCurFloor].SetObstacle(r, c, true);
        }
    SetDlgItemText(IDC_STATIC_STATUS, _T("已随机生成障碍（保证连通）"));
    Invalidate();
}

void CPathPlanningDlg::OnBnClickedBtnPrevFloor()
{
    if (m_nCurFloor > 0) {
        m_nCurFloor--;
        CString s; s.Format(_T("当前第 %d 层"), m_nCurFloor + 1);
        SetDlgItemText(IDC_STATIC_STATUS, s);
        Invalidate();
    }
}

void CPathPlanningDlg::OnBnClickedBtnNextFloor()
{
    if (m_nCurFloor < m_nFloors - 1) {
        m_nCurFloor++;
        CString s; s.Format(_T("当前第 %d 层"), m_nCurFloor + 1);
        SetDlgItemText(IDC_STATIC_STATUS, s);
        Invalidate();
    }
}

void CPathPlanningDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
    if (m_editMode == MODE_NONE) { CDialogEx::OnLButtonDown(nFlags, point); return; }
    CPoint grid = ScreenToGrid(point);
    if (grid.x < 0 || grid.y < 0) { CDialogEx::OnLButtonDown(nFlags, point); return; }

    int r = grid.x, c = grid.y;
    Point elev = GetElevator();
    switch (m_editMode) {
    case MODE_OBSTACLE:
        if (!(m_nCurFloor == m_startFloor && r == m_start.x && c == m_start.y) && !(r == elev.x && c == elev.y)) {
            m_floors[m_nCurFloor].SetObstacle(r, c, true);
            m_bHasPath = false; m_fullPath.clear();
        }
        break;
    case MODE_ERASE:
        m_floors[m_nCurFloor].SetObstacle(r, c, false);
        m_bHasPath = false; m_fullPath.clear();
        break;
    case MODE_GOAL: ToggleGoal(m_nCurFloor, r, c); break;
    case MODE_START: SetStartAt(m_nCurFloor, r, c); break;
    case MODE_ELEVATOR: SetElevatorAt(r, c); break;
    }
    Invalidate();
    CDialogEx::OnLButtonDown(nFlags, point);
}

void CPathPlanningDlg::OnMouseMove(UINT nFlags, CPoint point)
{
    if (!(nFlags & MK_LBUTTON)) { CDialogEx::OnMouseMove(nFlags, point); return; }
    if (m_editMode != MODE_OBSTACLE && m_editMode != MODE_ERASE) {
        CDialogEx::OnMouseMove(nFlags, point); return;
    }
    CPoint grid = ScreenToGrid(point);
    if (grid.x < 0 || grid.y < 0) { CDialogEx::OnMouseMove(nFlags, point); return; }

    int r = grid.x, c = grid.y;
    Point elev = GetElevator();
    if (m_editMode == MODE_OBSTACLE) {
        if (!(m_nCurFloor == m_startFloor && r == m_start.x && c == m_start.y) && !(r == elev.x && c == elev.y)) {
            m_floors[m_nCurFloor].SetObstacle(r, c, true);
            m_bHasPath = false; m_fullPath.clear();
        }
    }
    else {
        m_floors[m_nCurFloor].SetObstacle(r, c, false);
        m_bHasPath = false; m_fullPath.clear();
    }
    Invalidate();
    CDialogEx::OnMouseMove(nFlags, point);
}

void CPathPlanningDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == 1 && m_bRobotMoving) {
        StepDynamic();
        StepRobot();
        if (m_wpIndex >= (int)m_waypoints.size()) {
            KillTimer(m_nTimerID); m_bRobotMoving = false;
            SetDlgItemText(IDC_STATIC_STATUS, _T("已完成所有终点并返回起点！"));
        }
        else {
            m_nCurFloor = m_robotPos.floor;
            SetDlgItemText(IDC_STATIC_STATUS, m_bWaiting ? _T("障碍挡路，等待中...") : _T("机器人移动中..."));
        }
        Invalidate();
    }
    CDialogEx::OnTimer(nIDEvent);
}

BOOL CPathPlanningDlg::OnEraseBkgnd(CDC* pDC) { return TRUE; }
void CPathPlanningDlg::OnStnClickedStaticCols() {}

void CPathPlanningDlg::GetOccupiedSet(int floor, set<pair<int, int>>& occupied, const DynObstacle* skip)
{
    occupied.clear();
    for (auto& ob : m_dynObs) {
        if (!ob.active || ob.floor != floor) continue;
        if (&ob == skip) continue;
        occupied.insert({ ob.pos.x, ob.pos.y });
    }
}

// 初始化：每层4个动态障碍（前2个随机游走，后2个DFS）
void CPathPlanningDlg::InitDynObstacles()
{
    m_dynObs.clear();
    for (int f = 0; f < m_nFloors; f++) {
        for (int i = 0; i < DYN_PER_FLOOR; i++) {
            DynObstacle ob;
            ob.floor = f;
            ob.type = (i < 2) ? DYN_RANDOM_WALK : DYN_DFS_TO_ELEVATOR;
            ob.idx = 0;
            SpawnOneDynamic(ob);
            m_dynObs.push_back(ob);
        }
    }
}

// 在本层随机空格生成一个障碍
void CPathPlanningDlg::SpawnOneDynamic(DynObstacle& ob)
{
    ob.active = true;
    ob.path.clear();
    ob.idx = 0;
    int f = ob.floor;
    Point elev = GetElevator();

    // 找一个随机空格出生
    Point spawn(-1, -1);
    set<pair<int, int>> occupied;
    GetOccupiedSet(f, occupied, &ob);

    for (int t = 0; t < 200; t++) {
        int r = rand() % m_nRows;
        int c = rand() % m_nCols;
        if (m_floors[f].GetMap()[r][c]) continue;
        if (r == elev.x && c == elev.y) continue;
        if (f == m_startFloor && r == m_start.x && c == m_start.y) continue;
        bool isGoal = false;
        for (auto& g : m_goals)
            if (g.floor == f && g.x == r && g.y == c) { isGoal = true; break; }
        if (isGoal) continue;
        if (occupied.count({ r, c })) continue;  // 避开已有障碍
        spawn = Point(r, c);
        break;
    }
    if (spawn.x < 0) { ob.pos = elev; ob.last = elev; return; }
    ob.pos = spawn;
    ob.last = spawn;

    // DFS型：规划一条到电梯的DFS路径
    if (ob.type == DYN_DFS_TO_ELEVATOR) {
        vector<Point> p;
        if (FindPathOnFloor(f, spawn, elev, p, 3) && (int)p.size() >= 2) {
            ob.path = p;
            ob.idx = 0;
        }
    }
}

// 随机游走一步：尽量不回头，尽量遍历新格子，尽量避开其他障碍
void CPathPlanningDlg::RandomWalkStep(DynObstacle& ob, const set<pair<int, int>>& occupied)
{
    int f = ob.floor;
    const auto& map = m_floors[f].GetMap();
    int dx[4] = { -1, 1, 0, 0 }, dy[4] = { 0, 0, -1, 1 };

    vector<Point> cand;       // 所有可走方向
    vector<Point> noBack;     // 不回头的方向
    vector<Point> noCollision;// 不与其他障碍碰撞的方向

    for (int d = 0; d < 4; d++) {
        int nx = ob.pos.x + dx[d], ny = ob.pos.y + dy[d];
        if (nx < 0 || nx >= m_nRows || ny < 0 || ny >= m_nCols) continue;
        if (map[nx][ny]) continue;
        Point np(nx, ny);
        cand.push_back(np);
        if (!(np == ob.last)) noBack.push_back(np);
        if (!occupied.count({ nx, ny })) noCollision.push_back(np);
    }

    Point next;
    // 优先级：不碰撞+不回头 > 不碰撞 > 不回头 > 任意可走
    vector<Point> pref;
    for (auto& p : noCollision)
        if (!(p == ob.last)) pref.push_back(p);
    if (!pref.empty())
        next = pref[rand() % pref.size()];
    else if (!noCollision.empty())
        next = noCollision[rand() % noCollision.size()];
    else if (!noBack.empty())
        next = noBack[rand() % noBack.size()];
    else if (!cand.empty())
        next = cand[rand() % cand.size()];
    else
        return;

    ob.last = ob.pos;
    ob.pos = next;
}

// 所有动态障碍各走一步 + 即时重生
void CPathPlanningDlg::StepDynamic()
{
    if (!m_bDynEnabled) return;

    // 按层处理：先收集占用，再逐个移动
    for (int f = 0; f < m_nFloors; f++) {
        set<pair<int, int>> occupied;
        GetOccupiedSet(f, occupied);

        for (auto& ob : m_dynObs) {
            if (ob.floor != f || !ob.active) continue;

            occupied.erase({ ob.pos.x, ob.pos.y });

            if (ob.type == DYN_DFS_TO_ELEVATOR && (int)ob.path.size() >= 2) {
                if (ob.idx + 1 < (int)ob.path.size()) {
                    Point nextP = ob.path[ob.idx + 1];
                    if (!occupied.count({ nextP.x, nextP.y })) {
                        ob.idx++;
                        ob.pos = nextP;
                    }
                    else {
                        ob.idx++;
                        ob.pos = nextP;
                    }
                }
                else {
                    ob.active = false;
                }
            }
            else {
                RandomWalkStep(ob, occupied);
            }

            if (ob.active)
                occupied.insert({ ob.pos.x, ob.pos.y });
        }
    }

    for (int f = 0; f < m_nFloors; f++) {
        vector<DynObstacle*> inactiveSlots;
        int activeCount = 0;
        for (auto& ob : m_dynObs) {
            if (ob.floor != f) continue;
            if (ob.active) activeCount++;
            else inactiveSlots.push_back(&ob);
        }
        for (auto slot : inactiveSlots) {
            if (activeCount >= DYN_PER_FLOOR) break;
            SpawnOneDynamic(*slot);
            if (slot->active) activeCount++;
        }
    }
}

// 计算两点间路径距离（BFS求最短路径长度），返回-1表示不可达
int CPathPlanningDlg::PathDistance(int floor, Point a, Point b, const set<pair<int, int>>& blocked)
{
    if (a == b) return 0;
    const auto& map = m_floors[floor].GetMap();
    int R = m_nRows, C = m_nCols;
    if (map[b.x][b.y] || blocked.count({ b.x, b.y })) return -1;

    vector<vector<int>> dist(R, vector<int>(C, -1));
    queue<Point> q;
    q.push(a);
    dist[a.x][a.y] = 0;
    int dx[4] = { -1, 1, 0, 0 }, dy[4] = { 0, 0, -1, 1 };

    while (!q.empty()) {
        Point cur = q.front(); q.pop();
        if (cur == b) return dist[cur.x][cur.y];
        for (int d = 0; d < 4; d++) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx < 0 || nx >= R || ny < 0 || ny >= C) continue;
            if (map[nx][ny] || dist[nx][ny] >= 0) continue;
            if (blocked.count({ nx, ny })) continue;
            dist[nx][ny] = dist[cur.x][cur.y] + 1;
            q.push(Point(nx, ny));
        }
    }
    return -1;
}

// 规划从s到e的完整避障路径
bool CPathPlanningDlg::PlanAheadFull(int floor, Point s, Point e,
    const set<pair<int, int>>& blocked, vector<Point>& fullPath)
{
    fullPath.clear();
    if (s == e) { fullPath.push_back(s); return true; }
    const auto& map = m_floors[floor].GetMap();
    int R = m_nRows, C = m_nCols;
    if (map[e.x][e.y] || blocked.count({ e.x, e.y })) return false;

    vector<vector<bool>>  visited(R, vector<bool>(C, false));
    vector<vector<Point>> parent(R, vector<Point>(C, Point(-1, -1)));
    queue<Point> q;
    q.push(s); visited[s.x][s.y] = true;
    int dx[4] = { -1, 1, 0, 0 }, dy[4] = { 0, 0, -1, 1 };
    bool found = false;

    while (!q.empty()) {
        Point cur = q.front(); q.pop();
        if (cur == e) { found = true; break; }
        for (int d = 0; d < 4; d++) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx < 0 || nx >= R || ny < 0 || ny >= C) continue;
            if (visited[nx][ny] || map[nx][ny]) continue;
            if (blocked.count({ nx, ny })) continue;
            visited[nx][ny] = true; parent[nx][ny] = cur;
            q.push(Point(nx, ny));
        }
    }
    if (!found) return false;

    vector<Point> rev; Point cur = e;
    while (!(cur == s)) { rev.push_back(cur); cur = parent[cur.x][cur.y]; if (cur.x < 0) return false; }
    rev.push_back(s);
    for (int i = (int)rev.size() - 1; i >= 0; i--) fullPath.push_back(rev[i]);
    return true;
}

// 机器人反应式走一步
void CPathPlanningDlg::StepRobot()
{
    if (m_waypoints.empty() || m_wpIndex >= (int)m_waypoints.size()) return;
    PathNode tgt = m_waypoints[m_wpIndex];

    // 到达关键点，切下一个
    if (m_robotPos.floor == tgt.floor && m_robotPos.x == tgt.x && m_robotPos.y == tgt.y) {
        m_wpIndex++;
        if (m_wpIndex >= (int)m_waypoints.size()) { m_aheadPath.clear(); return; }
        tgt = m_waypoints[m_wpIndex];
    }

    Point elev = GetElevator();
    Point subTarget;
    bool switchedFloor = false;

    if (m_robotPos.floor == tgt.floor) {
        subTarget = Point(tgt.x, tgt.y);
    }
    else {
        if (m_robotPos.x == elev.x && m_robotPos.y == elev.y) {
            // 到达电梯 → 切换楼层
            PathNode node = { m_robotPos.floor, m_robotPos.x, m_robotPos.y };
            if (m_traveledPath.empty() || !(m_traveledPath.back().floor == node.floor &&
                m_traveledPath.back().x == node.x && m_traveledPath.back().y == node.y))
                m_traveledPath.push_back(node);
            m_robotPos.floor = tgt.floor;
            m_bWaiting = false;
            switchedFloor = true;
        }
        else {
            subTarget = elev;
        }
    }

    if (!switchedFloor) {
        // 收集动态障碍位置
        set<pair<int, int>> blocked;
        if (m_bDynEnabled) {
            for (auto& ob : m_dynObs) {
                if (!ob.active || ob.floor != m_robotPos.floor) continue;
                blocked.insert({ ob.pos.x, ob.pos.y });
            }
        }

        Point cur(m_robotPos.x, m_robotPos.y);
        vector<Point> fp;
        bool shouldWait = false;

        // 第一步：尝试绕开所有动态障碍
        bool canAvoid = PlanAheadFull(m_robotPos.floor, cur, subTarget, blocked, fp)
            && (int)fp.size() >= 2;

        if (canAvoid) {
            // 能绕开 → 直接走绕开的路径
            m_bWaiting = false;
        }
        else if (m_bDynEnabled && !blocked.empty()) {
            // 无法避开 → 计算到最近障碍的路径距离，决定是否静止
            set<pair<int, int>> emptySet;
            vector<Point> idealPath;
            if (PlanAheadFull(m_robotPos.floor, cur, subTarget, emptySet, idealPath)
                && (int)idealPath.size() >= 2) {
                // 在理想路径（不考虑动态障碍）上找第一个动态障碍
                int distToOb = -1;
                for (size_t i = 1; i < idealPath.size(); i++) {
                    if (blocked.count({ idealPath[i].x, idealPath[i].y })) {
                        distToOb = (int)i; // 路径距离（格数）
                        break;
                    }
                }
                if (distToOb >= 0 && distToOb <= 3) {
                    // 距离 ≤ 3 格 → 静止，让动态障碍通过
                    shouldWait = true;
                    fp = idealPath;
                }
                else {
                    // 距离 > 3 格 → 继续沿理想路径前进
                    fp = idealPath;
                    shouldWait = false;
                }
            }
            else {
                shouldWait = true; // 连静态障碍都过不去
            }
        }
        else {
            shouldWait = true; // 完全无路
        }

        if (!shouldWait && (int)fp.size() >= 2) {
            // 前进一格
            PathNode node = { m_robotPos.floor, m_robotPos.x, m_robotPos.y };
            if (m_traveledPath.empty() || !(m_traveledPath.back().floor == node.floor &&
                m_traveledPath.back().x == node.x && m_traveledPath.back().y == node.y))
                m_traveledPath.push_back(node);
            m_robotPos.x = fp[1].x;
            m_robotPos.y = fp[1].y;
            m_bWaiting = false;
        }
        else {
            m_bWaiting = true;
        }
    }

    // 更新前方显示路径
    m_aheadPath.clear();
    if (m_wpIndex < (int)m_waypoints.size()) {
        PathNode curTgt = m_waypoints[m_wpIndex];
        Point sub = (m_robotPos.floor == curTgt.floor) ? Point(curTgt.x, curTgt.y) : elev;

        set<pair<int, int>> blockedDisp;
        if (m_bDynEnabled) {
            for (auto& ob : m_dynObs) {
                if (!ob.active || ob.floor != m_robotPos.floor) continue;
                blockedDisp.insert({ ob.pos.x, ob.pos.y });
            }
        }
        vector<Point> fp;
        Point curP(m_robotPos.x, m_robotPos.y);
        if (PlanAheadFull(m_robotPos.floor, curP, sub, blockedDisp, fp)) {
            for (auto& p : fp) {
                PathNode n = { m_robotPos.floor, p.x, p.y };
                m_aheadPath.push_back(n);
            }
        }
    }
}

// 加速按钮
void CPathPlanningDlg::OnBnClickedBtnAccelerate()
{
    const int MIN_SPEED = 30;
    if (!m_bAccelerated) {
        m_nNormalSpeed = m_nSpeed;
        int newSpeed = m_nNormalSpeed / 3;
        if (newSpeed < MIN_SPEED) newSpeed = MIN_SPEED;
        m_nSpeed = newSpeed;
        m_bAccelerated = true;
    }
    else {
        m_nSpeed = m_nNormalSpeed;
        m_bAccelerated = false;
    }
    if (m_bRobotMoving && m_nTimerID != 0) {
        KillTimer(m_nTimerID);
        m_nTimerID = (UINT)SetTimer(1, m_nSpeed, nullptr);
    }
    CString msg;
    if (m_bAccelerated)
        msg.Format(_T("已加速 (间隔 %d ms) —— 再次点击恢复原速"), m_nSpeed);
    else
        msg.Format(_T("已恢复原速 (间隔 %d ms)"), m_nSpeed);
    SetDlgItemText(IDC_STATIC_STATUS, msg);
}