#include "pch.h"
#include "framework.h"
#include "PathPlanning.h"
#include "PathPlanningDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

BEGIN_MESSAGE_MAP(CPathPlanningApp, CWinApp)
    ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()

CPathPlanningApp::CPathPlanningApp() noexcept
{
}

CPathPlanningApp theApp;

BOOL CPathPlanningApp::InitInstance()
{
    CWinApp::InitInstance();

    CPathPlanningDlg dlg;
    m_pMainWnd = &dlg;
    INT_PTR nResponse = dlg.DoModal();
    if (nResponse == IDOK)
    {
    }
    else if (nResponse == IDCANCEL)
    {
    }

    return FALSE;
}
