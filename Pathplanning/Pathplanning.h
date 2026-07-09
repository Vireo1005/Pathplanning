#pragma once
#include "resource.h"

class CPathPlanningApp : public CWinApp
{
public:
    CPathPlanningApp() noexcept;

public:
    virtual BOOL InitInstance();

    DECLARE_MESSAGE_MAP()
};

extern CPathPlanningApp theApp;
