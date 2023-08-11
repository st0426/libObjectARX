// GridCtrlUtility.cpp: implementation of the CGridCtrlUtility class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "GridCtrlUtil.h"
#include <assert.h>
#include <algorithm>
#include <functional>
#include "CellTypes\GridCellCheck.h"
#include "CellTypes\GridCellCombo.h"
#include "CellTypes\GridCellButton.h"

using namespace std;

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CGridCtrlUtil::CGridCtrlUtil()
{
}

CGridCtrlUtil::~CGridCtrlUtil()
{

}

// ���õ�Ԫ��Ϊ��ѡ������
void CGridCtrlUtil::SetCellCheckType( bool bIsCheck, CGridCtrl &gridCtrl, int rowIndex, 
										int columnIndex)
{
	gridCtrl.SetCellType(rowIndex, columnIndex, RUNTIME_CLASS(CGridCellCheck));
	CGridCellCheck* pCheckCell = (CGridCellCheck*)gridCtrl.GetCell(rowIndex, columnIndex);
	if (pCheckCell != NULL)
	{
		pCheckCell->SetCheck(bIsCheck);
	}
}

// ��ø�ѡ��Ԫ���û��Ƿ�ѡ��
bool CGridCtrlUtil::GetCellChecked(CGridCtrl &gridCtrl, int rowIndex, int columnIndex)
{
	if (gridCtrl.GetCell(rowIndex, columnIndex)->IsKindOf(RUNTIME_CLASS(CGridCellCheck)))
	{
		CGridCellCheck* pCheckCell = (CGridCellCheck*)gridCtrl.GetCell(rowIndex, columnIndex);
		if (pCheckCell != NULL)
		{
			return pCheckCell->GetCheck() == TRUE;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

// ��ø�ѡ��Ԫ���ѡ��״̬
void CGridCtrlUtil::SetCellChecked( CGridCtrl &grid, int rowIndex, int colIndex, bool bChecked )
{	
	if (grid.GetCell(rowIndex, colIndex)->IsKindOf(RUNTIME_CLASS(CGridCellCheck)))
	{
		((CGridCellCheck*)grid.GetCell(rowIndex, colIndex))->SetCheck(bChecked);
	}
}

// ���õ�Ԫ��Ϊ��Ͽ�����
void CGridCtrlUtil::SetCellComboType( CGridCtrl &gridCtrl, int row, int col )
{
	gridCtrl.SetCellType(row, col, RUNTIME_CLASS(CGridCellCombo));
}

// ������Ͽ�Ԫ��Ŀ�ѡ��
void CGridCtrlUtil::SetCellComboOptions( CGridCtrl &gridCtrl, int row, int col, const CStringArray &ar )
{
	CGridCellCombo *pCell = (CGridCellCombo*) gridCtrl.GetCell(row, col);
	pCell->SetOptions(ar);
}

// ������Ͽ�Ԫ��Ŀ�ѡ��
void CGridCtrlUtil::SetCellComboOptions( CGridCtrl &gridCtrl, int row, int col, const std::vector<CString> &items )
{
	CStringArray ar;
	for (int i = 0; i < items.size(); i++)
	{
		ar.Add(items[i]);
	}
	SetCellComboOptions(gridCtrl, row, col, ar);
}

// ������Ͽ�Ԫ�����ʽ
void CGridCtrlUtil::SetCellComboDwStyle( CGridCtrl &gridCtrl, int row, int col, DWORD dwStyle )
{
	CGridCellCombo *pCell = (CGridCellCombo*) gridCtrl.GetCell(row, col);
	pCell->SetStyle(dwStyle);
}

// ���õ�Ԫ��Ϊ��ͨ�ı༭������
void CGridCtrlUtil::SetCellEditType( CGridCtrl &gridCtrl, int row, int col )
{
	gridCtrl.SetCellType(row, col, RUNTIME_CLASS(CGridCell));
}

// ���õ�Ԫ��Ϊ��ť����
void CGridCtrlUtil::SetCellButtonType( CGridCtrl &gridCtrl, int row, int col )
{
	gridCtrl.SetCellType(row, col, RUNTIME_CLASS(CGridCellButton));
}

// ĳ����Ԫ���Ƿ��Ǹ�ѡ������
bool CGridCtrlUtil::CellIsCheckType( CGridCtrl &grid, int row, int col )
{
	return (grid.GetCell(row, col)->IsKindOf(RUNTIME_CLASS(CGridCellCheck)) == TRUE);
}
