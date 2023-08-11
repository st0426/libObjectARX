#include "excel.h"
#include <vector>
// ExcelUtil.h: interface for the CExcelUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_EXCELUTIL_H__EA0F9C4C_5901_43F0_ABA7_7FEDB6E681C0__INCLUDED_)
#define AFX_EXCELUTIL_H__EA0F9C4C_5901_43F0_ABA7_7FEDB6E681C0__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "..\UI\MfcGridCtrl\GridCtrl.h"
#include "..\ObjectARX\DwgTable\ZhifanDwgTable.h"

#if defined (_ZFGKCOMMONLIB_)
#define ZFGK_DLLIMPEXP __declspec(dllexport)
#else
#define ZFGK_DLLIMPEXP 
#endif

class ZFGK_DLLIMPEXP CExcelUtil  
{
public:
	CExcelUtil();
	virtual ~CExcelUtil();

	// ����Excel����
	// �����ڵ��ó����InitApplication�����г�ʼ��COM��̬���ӿ⣺::CoInitialize(NULL);
	// ͬʱ��UnloadApplication�����йر�COM���֧�֣�::CoUninitialize();
	static bool RunExcelApp(_Application &excelApp, bool bVisible = true, bool bUserControl = true);

	// �½�һ��workbook
	// templateFileName��ģ���ļ�λ�ã���չ��Ϊxlt��
	static bool NewWorkBook(_Application excelApp, const TCHAR* templateFileName, _Workbook &excelBook);
	static void NewWorkBook(_Application excelApp, _Workbook &excelBook);

	// ��excel�ļ������workbook
	static bool OpenWorkBook(_Application excelApp, const TCHAR* fileName, _Workbook &excelBook, bool bIsReadOnly = true);

	// ����ָ����_Workbook���������WorkSheet������
	static void GetWorkSheets(_Workbook &excelBook, std::vector<CString> &vecSheetNames);
	static void GetWorkSheets(_Workbook excelBook, Worksheets &excelSheets);

	// ��û��worksheet������
	static int GetWorkSheetCount(Worksheets excelSheets);

	// ���ָ��sheet���Ƶ�workSheet
	static bool GetWorkSheet( Worksheets excelSheets, const TCHAR* szWorkSheetName, _Worksheet &excelSheet );	
	static bool GetWorkSheet( _Workbook excelBook, const TCHAR* szWorkSheetName, _Worksheet &excelSheet );

	// ���ָ��λ�õ�workSheet
	// i���ӡ�1����ʼ
	static bool GetWorkSheet( Worksheets excelSheets, int i, _Worksheet &excelSheet);
	static bool GetWorkSheet( _Workbook excelBook, int i, _Worksheet &excelSheet);

	// ��õ�ǰ���workSheet
	static void GetActiveWorkSheet(_Workbook excelBook, _Worksheet &excelSheet);
	
	// ȡ��ָ��excelSheet����Ч��������
	static void GetUsedRange(_Worksheet excelSheet, Range &excelRange);

	// ȡ������
	static long GetRowCount(Range excelRange);
	static long GetRowCount(_Worksheet excelSheet);

	// �и�
	static double GetRowHeight(_Worksheet sht, int row);
	static void SetRowHeight(_Worksheet sht, int row, double rowHeight);
	
	// �������
	static long GetColumnCount(Range excelRange);
	static long GetColumnCount(_Worksheet excelSheet);

	// �п�
	static double GetColumnWidth(_Worksheet sht, int col);
	static void SetColumnWidth(_Worksheet sht, int col, double width);

	// ȡ��ָ����ָ���еĵ�Ԫ������
	// �кš��кŴӡ�1����ʼ����
	static bool GetCellValue(Range &excelRange, int rowIndex, int columnIndex, CString &strValue);
	static bool GetCellValue(_Worksheet excelSheet, int rowIndex, int columnIndex, CString &strValue);

	// ��excel��Ԫ����д������
	static bool SetCellValue( Range &excelRange, int rowIndex, int columnIndex, const TCHAR* szValue );
	static bool SetCellValue( _Worksheet excelSheet, int rowIndex, int columnIndex, const TCHAR* szValue );
	static bool SetCellValue( _Worksheet excelSheet, int rowIndex, int columnIndex, const VARIANT &val );

	// �ϲ���Ԫ��
	static void MergeCells(Range &excelRange, int row, int col, int mergeRows, int mergeCols, bool bHorCenter = true);
	static void MergeCells(_Worksheet excelSheet, int row, int col, int mergeRows, int mergeCols, bool bHorCenter = true);

	// ���õ�Ԫ�������
	static void SetCellFontInfo(_Worksheet sht, int row, int col, int size, bool bBold);

	// ���ñ߿�
	static void SetFrame(Range &excelRange, bool bInnerFrame, bool bExteralFrame);

	// ����ˮƽ���뷽ʽΪ����
	static void SetHorizontalCenter(Range &excelRange);
	static void SetHorizontalCenter(_Worksheet sht, int row, int col);

	// ���õ�Ԫ����Զ�����
	static void SetAutoWrap(Range &excelRange, bool bAutoWrap);

	// Excel��ת��//////////////////////////////////////////////////////////////////////////
	// ��ĸת�������֣�ֻ֧��2����ĸ��
	static int GetColumnNumberIndex(const TCHAR * column);
	static CString GetColumnName(int col);

	// ��GridCtrl�����ݵ���ΪExcel
	static void GridToExcel(CGridCtrl &gridCtrl, const TCHAR* templateFile, const TCHAR* fileName, int startRowIndex = 1);
	static void GridToExcel(CGridCtrl &gridCtrl, const TCHAR* fileName, int startRowIndex = 1);

	// ZhifanDwgTable���ݵ���ΪExcel
	static void DwgTableToExcel(ZhifanDwgTable *pTable, int startRowIndex = 1);
};

#endif // !defined(AFX_EXCELUTIL_H__EA0F9C4C_5901_43F0_ABA7_7FEDB6E681C0__INCLUDED_)
