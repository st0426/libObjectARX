// GridCtrlUtility.h: interface for the CGridCtrlUtility class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GRIDCTRLUTIL_H__1DA720E2_B555_46F7_9B99_304A8B57B54F__INCLUDED_)
#define AFX_GRIDCTRLUTIL_H__1DA720E2_B555_46F7_9B99_304A8B57B54F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "GridCtrl.h"
#include <vector>

class CGridCtrlUtil  
{
public:
	CGridCtrlUtil();
	virtual ~CGridCtrlUtil();

	//////////////////////////////////////////////////////////////////////////
	// ��Ԫ������Ϊ��������
	//////////////////////////////////////////////////////////////////////////
	// ���õ�Ԫ��Ϊ��ѡ����ʽ
	static void SetCellCheckType( bool bIsCheck, CGridCtrl &gridCtrl, int rowIndex, int columnIndex);
	// ��Ԫ���Ƿ��Ǹ�ѡ��
	static bool CellIsCheckType(CGridCtrl &grid, int row, int col);
	// ���ø�ѡ��Ԫ���ѡ��״̬
	static void SetCellChecked(CGridCtrl &grid, int rowIndex, int colIndex, bool bChecked);
	// �жϸ�ѡ��ĵ�Ԫ���Ƿ�ѡ��
	static bool GetCellChecked(CGridCtrl &gridCtrl, int rowIndex, int columnIndex);

	// ���õ�Ԫ��Ϊ��Ͽ�����
	static void SetCellComboType(CGridCtrl &gridCtrl, int row, int col);
	// ������Ͽ��ѡ���
	static void SetCellComboOptions(CGridCtrl &gridCtrl, int row, int col, const CStringArray &ar);
	static void SetCellComboOptions(CGridCtrl &gridCtrl, int row, int col, const std::vector<CString> &items);
	// ������Ͽ����ʽ
	static void SetCellComboDwStyle(CGridCtrl &gridCtrl, int row, int col, DWORD dwStyle);

	// ������Ͽ�Ϊ��ť����
	static void SetCellButtonType(CGridCtrl &gridCtrl, int row, int col);

	// ���õ�Ԫ��Ϊ��ͨ����
	static void SetCellEditType(CGridCtrl &gridCtrl, int row, int col);
};

#endif // !defined(AFX_GRIDCTRLUTIL_H__1DA720E2_B555_46F7_9B99_304A8B57B54F__INCLUDED_)
