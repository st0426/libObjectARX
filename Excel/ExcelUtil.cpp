// ExcelUtil.cpp: implementation of the CExcelUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ExcelUtil.h"
#include <io.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CExcelUtil::CExcelUtil()
{

}

CExcelUtil::~CExcelUtil()
{

}

bool CExcelUtil::RunExcelApp( _Application &excelApp, bool bVisible /*= true*/, bool bUserControl /*= true*/ )
{
	// ����Excel
	// �ȼ���Ƿ��Ѿ�����
	CLSID clsId;
	::CLSIDFromProgID(L"Excel.Application", &clsId); // from registry
	IUnknown* pUnk = NULL;
	if (::GetActiveObject(clsId, NULL, &pUnk) == S_OK) 
		
	{
		LPDISPATCH pDispatch;
		VERIFY(pUnk->QueryInterface(IID_IDispatch,(void**) &pDispatch) == S_OK);
		excelApp.AttachDispatch(pDispatch);
		pUnk->Release();
	} 
	else
		
	{
		if(!excelApp.CreateDispatch(TEXT("Excel.Application")))
		{
			AfxMessageBox(TEXT("��ǰ�����û�а�װExcel!"));
			return false;
		}
	}
	
	excelApp.SetVisible(bVisible);
	excelApp.SetUserControl(bUserControl);	// �Ƿ����������û�����Excel
	
	return true;
}

bool CExcelUtil::OpenWorkBook( _Application excelApp, const TCHAR* fileName, _Workbook &excelBook, bool bIsReadOnly /*= true*/ )
{
	try
	{
		COleVariant vOptional((long)DISP_E_PARAMNOTFOUND, VT_ERROR);  //��ʾһ��Ϊoptional��ֵ
		COleVariant vIsReadOnly((short)bIsReadOnly);	// ֻ��
		// COleVariant vUpdateLinks((short)0);
		// UpdateLinks      Variant ���ͣ���ѡ��ָ���ļ������ӵĸ��·�ʽ��
		// ���ʡ�Ա�����������ʾ�û�ѡ�����ӵĸ��·�ʽ�����򣬸ò�����ȡֵӦΪ�±��е�ĳ��ֵ��
		// ֵ ���� 
		// 0 �������κ����á� 
		// 1 �����ⲿ���ã���������Զ�����á� 
		// 2 ����Զ�����ã����������ⲿ���á� 
		// 3 ͬʱ����Զ�����ú��ⲿ���á� 			
		
		// ���Workbooks
		LPDISPATCH pDispatch = excelApp.GetWorkbooks();
		Workbooks excelBooks;
		excelBooks.AttachDispatch(pDispatch);
		
		//��Excel�ļ�
		pDispatch = excelBooks.Open(fileName, vOptional, vIsReadOnly, vOptional, vOptional, 
			vOptional, vOptional, vOptional, vOptional, vOptional, vOptional, 
			vOptional, vOptional, vOptional, vOptional); 
		
		excelBook.AttachDispatch(pDispatch);
	}
	catch (_com_error e)
	{
		assert(false);
		AfxMessageBox(e.ErrorMessage());
		return false;
	}
	
	return true;
}

// ���Դ���
// _Application excelApp;
// CExcelUtil::RunExcelApp(excelApp);
// _Workbook workBook;
// CExcelUtil::OpenWorkBook(excelApp, TEXT("E:\\�����ļ�.xls"), workBook, false);
// std::vector<CString> shts;
// CExcelUtil::GetWorkSheets(workBook, shts);
// for (int k = 0; k < shts.size(); k++)
// {
// 	acutPrintf(TEXT("\n%s"), shts[k]);
// }
// 	return;

void CExcelUtil::GetWorkSheets( _Workbook &excelBook, std::vector<CString> &vecSheetNames )
{
	vecSheetNames.clear();
	
	Worksheets excelSheets;
	GetWorkSheets(excelBook, excelSheets);
	
	int count = excelSheets.GetCount();
	for (int i = 0; i < count; i++)
	{
		CString sheetName = TEXT("");
		_Worksheet excelSheet;
		if (GetWorkSheet(excelBook, i + 1, excelSheet))
		{
			sheetName = excelSheet.GetName();
			vecSheetNames.push_back(sheetName);
		}		
	}
}

void CExcelUtil::GetWorkSheets( _Workbook excelBook, Worksheets &excelSheets )
{
	LPDISPATCH pDispatch = excelBook.GetWorksheets();
	excelSheets.AttachDispatch(pDispatch);
}

bool CExcelUtil::GetWorkSheet( Worksheets excelSheets, const TCHAR* szWorkSheetName, _Worksheet &excelSheet )
{
	bool bIsFind = false;
	
	for (int i = 1; i < excelSheets.GetCount() + 1; i++)
	{
		LPDISPATCH pDispatch = excelSheets.GetItem(COleVariant(long(i)));
		excelSheet.AttachDispatch(pDispatch);
		
		CString strSheetName = excelSheet.GetName();
		CString strSheetNameInput;
		
		strSheetNameInput.Format(TEXT("%s"), szWorkSheetName);
		if (strSheetName.CompareNoCase(strSheetNameInput) == 0)
		{
			bIsFind = true;
			break;
		}
	}
	
	return bIsFind;
}

bool CExcelUtil::GetWorkSheet( _Workbook excelBook, const TCHAR* szWorkSheetName, _Worksheet &excelSheet )
{
	LPDISPATCH pDispatch = excelBook.GetWorksheets();
	Worksheets excelSheets;
	excelSheets.AttachDispatch(pDispatch);
	
	return GetWorkSheet(excelSheets, szWorkSheetName, excelSheet);
}

bool CExcelUtil::GetWorkSheet( Worksheets excelSheets, int i, _Worksheet &excelSheet )
{
	if (i > 0 
		&& i <= excelSheets.GetCount())
	{
		LPDISPATCH pDispatch = excelSheets.GetItem(COleVariant(long(i)));
		excelSheet.AttachDispatch(pDispatch);
		
		return true;
	}
	else
	{
		return false;
	}
}

bool CExcelUtil::GetWorkSheet( _Workbook excelBook, int i, _Worksheet &excelSheet )
{
	LPDISPATCH pDispatch = excelBook.GetWorksheets();
	Worksheets excelSheets;
	excelSheets.AttachDispatch(pDispatch);
	
	return GetWorkSheet(excelSheets, i, excelSheet);
}

bool CExcelUtil::GetCellValue( Range &excelRange, int rowIndex, int columnIndex, CString &strValue )
{
	assert(rowIndex > 0);
	assert(columnIndex > 0);
	
	COleVariant vValue;
	vValue = excelRange.GetItem(COleVariant((long)rowIndex), COleVariant((long)columnIndex));
	strValue = (LPCTSTR)_bstr_t(vValue);
	
	return true;
}

bool CExcelUtil::GetCellValue( _Worksheet excelSheet, int rowIndex, int columnIndex, CString &strValue )
{
	Range excelRange;
	GetUsedRange(excelSheet, excelRange);
	
	return GetCellValue(excelRange, rowIndex, columnIndex, strValue);
}

void CExcelUtil::GetActiveWorkSheet( _Workbook excelBook, _Worksheet &excelSheet )
{
	// ��õ�ǰ���Worksheet
	LPDISPATCH pDispatch = excelBook.GetActiveSheet();
	excelSheet.AttachDispatch(pDispatch);
}

void CExcelUtil::GetUsedRange( _Worksheet excelSheet, Range &excelRange )
{
	excelRange = excelSheet.GetUsedRange();
}

long CExcelUtil::GetRowCount( Range excelRange )
{
	Range rowRange;
	rowRange.AttachDispatch(excelRange.GetRows());
	return rowRange.GetCount();
}

long CExcelUtil::GetRowCount( _Worksheet excelSheet )
{
	Range excelRange;
	GetUsedRange(excelSheet, excelRange);
	
	return GetRowCount(excelRange);
}

long CExcelUtil::GetColumnCount( Range excelRange )
{
	Range rowRange;
	rowRange.AttachDispatch(excelRange.GetColumns());
	return rowRange.GetCount();
}

long CExcelUtil::GetColumnCount( _Worksheet excelSheet )
{
	Range excelRange;
	GetUsedRange(excelSheet, excelRange);
	
	return GetColumnCount(excelRange);
}

bool CExcelUtil::SetCellValue( Range &excelRange, int rowIndex, int columnIndex, const TCHAR* szValue )
{
	if (rowIndex <= 0
		|| columnIndex <= 0)
	{
		return false;
	}
	
	excelRange.SetItem(COleVariant((long)rowIndex), COleVariant((long)columnIndex), COleVariant(szValue));
	
	return true;
}

bool CExcelUtil::SetCellValue( _Worksheet excelSheet, int rowIndex, int columnIndex, const TCHAR* szValue )
{
	Range excelRange;
	excelRange.AttachDispatch(excelSheet.GetCells());
	
	return SetCellValue(excelRange, rowIndex, columnIndex, szValue);
}

bool CExcelUtil::SetCellValue( _Worksheet excelSheet, int rowIndex, int columnIndex, const VARIANT &val )
{
	Range excelRange;
	excelRange.AttachDispatch(excelSheet.GetCells());
	
	excelRange.SetItem(COleVariant((long)rowIndex), COleVariant((long)columnIndex), val);

	return true;
}

int CExcelUtil::GetColumnNumberIndex( const TCHAR * column )
{
	int rtValue = -1;
	CString strColumn = column;
	strColumn.TrimLeft();
	strColumn.TrimRight();
	int len = strColumn.GetLength();
	strColumn.MakeUpper();
	if (len != 1 && len != 2)
	{
		rtValue = -1;
	}
	else if (len == 1)
	{
		TCHAR c = strColumn.GetAt(0);
		rtValue = (int)c - 64;
	}
	else
	{
		TCHAR cr = strColumn.GetAt(1);
		TCHAR cl = strColumn.GetAt(0);
		rtValue = (int)cr - 64 + ((int)cl - 64) * 26;
	}
	
	return rtValue;
}

CString CExcelUtil::GetColumnName( int col )
{
	CString rtVal = TEXT("");
	
	int alpha = col / 27;
	int rem = col - alpha * 26;
	if (alpha > 0)
	{	
		rtVal = (char)(alpha + 64);
	}
	if (rem > 0)
	{
		CString rtTmp = (char)(rem + 64);
		rtVal = rtVal + rtTmp;
	}
	
	return rtVal;
}

bool CExcelUtil::NewWorkBook( _Application excelApp, const TCHAR* templateFileName, _Workbook &excelBook )
{
	if (_taccess(templateFileName, 0) != -1)
	{
		// ���Workbooks
		LPDISPATCH pDispatch = excelApp.GetWorkbooks();
		Workbooks excelBooks;
		excelBooks.AttachDispatch(pDispatch);
		
		// ��õ�ǰ���Workbook
		pDispatch = excelBooks.Add(COleVariant(templateFileName));
		excelBook.AttachDispatch(pDispatch);
		
		return true;
	}
	else
	{
		AfxMessageBox(TEXT("ָ����Excelģ���ļ�������!"));
		return false;
	}
}

void CExcelUtil::NewWorkBook( _Application excelApp, _Workbook &excelBook )
{
	// ���Workbooks
	LPDISPATCH pDispatch = excelApp.GetWorkbooks();
	Workbooks excelBooks;
	excelBooks.AttachDispatch(pDispatch);
	
	// ��õ�ǰ���Workbook
	COleVariant vOptional((long)DISP_E_PARAMNOTFOUND, VT_ERROR);
	pDispatch = excelBooks.Add(vOptional);
	excelBook.AttachDispatch(pDispatch);
}

int CExcelUtil::GetWorkSheetCount( Worksheets excelSheets )
{
	return excelSheets.GetCount();
}

void CExcelUtil::GridToExcel( CGridCtrl &gridCtrl, const TCHAR* templateFile, const TCHAR* fileName, int startRowIndex )
{
	if (gridCtrl.GetRowCount() > 0)
	{
		// ������excel
		_Application excelApp;
		RunExcelApp(excelApp, false, false);	// ��������ʱ����ʾexcel���û�����ʱ�������������
		
		// ȡ��workBook
		_Workbook excelBook;
		if (templateFile != NULL)
		{
			if (_taccess(templateFile, 0) == -1 || !NewWorkBook(excelApp, templateFile, excelBook))
			{
				acutPrintf(TEXT("\nģ���ļ�\"%s\"������."), templateFile);
				return;
			}
		}
		else
		{
			NewWorkBook(excelApp, excelBook);
		}
		
		// ȡ��worksheet
		_Worksheet excelSheet;
		GetActiveWorkSheet(excelBook, excelSheet);
		
		// ��excel��д������
		LPDISPATCH pDispatch = excelSheet.GetCells();
		Range excelRange;
		excelRange.AttachDispatch(pDispatch);
		
		for (int i = 0; i < gridCtrl.GetRowCount(); i++)	// ��
		{
			for (int j = 0; j < gridCtrl.GetColumnCount(); j++)	// ��
			{
				int rowIndex = i + startRowIndex;
				int columnIndex = j + 1;
				CExcelUtil::SetCellValue(excelRange, rowIndex, columnIndex, gridCtrl.GetItemText(i, j));
				bool bMergeWithOthers = gridCtrl.GetCell(i, j)->IsMergeWithOthers();		// ���ϲ�
				bool bMerged = gridCtrl.GetCell(i, j)->IsMerged();
				if (bMerged)
				{
					long rows = gridCtrl.GetCell(i, j)->GetMergeRange().GetRowSpan();
					long cols = gridCtrl.GetCell(i, j)->GetMergeRange().GetColSpan();
					Range r;
					r.AttachDispatch(excelRange.GetItem(COleVariant((long)rowIndex), COleVariant((long)columnIndex)).pdispVal);
					r.AttachDispatch(r.GetResize(COleVariant(rows), COleVariant(cols)));
					r.Merge(COleVariant((long)0));
				}
			}
		}
		
		excelSheet.SaveAs(fileName, vtMissing, vtMissing, vtMissing, vtMissing,
			vtMissing, vtMissing, vtMissing, vtMissing, vtMissing);
		
		excelApp.SetVisible(true);
		
		excelBook.DetachDispatch();
		excelSheet.DetachDispatch();
		excelRange.DetachDispatch();
	}
}

void CExcelUtil::GridToExcel( CGridCtrl &gridCtrl, const TCHAR* fileName, int startRowIndex )
{
	GridToExcel(gridCtrl, NULL, fileName);
}

void CExcelUtil::MergeCells( Range &excelRange, int row, int col, int mergeRows, int mergeCols, bool bHorCenter )
{
	Range r;	
	r.AttachDispatch(excelRange.GetItem(COleVariant((long)row), COleVariant((long)col)).pdispVal);	
	r.AttachDispatch(r.GetResize(COleVariant((long)mergeRows), COleVariant((long)mergeCols)));	
	r.Merge(COleVariant((long)0));
	if (bHorCenter)
	{
		r.SetHorizontalAlignment(COleVariant((long)-4108));
	}
}

void CExcelUtil::MergeCells( _Worksheet excelSheet, int row, int col, int mergeRows, int mergeCols, bool bHorCenter )
{
	Range excelRange;
	excelRange.AttachDispatch(excelSheet.GetCells());
	MergeCells(excelRange, row, col, mergeRows, mergeCols, bHorCenter);
}

double CExcelUtil::GetRowHeight( _Worksheet sht, int row )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)row), COleVariant((long)1)).pdispVal);
	return cell.GetRowHeight().dblVal;
}

void CExcelUtil::SetRowHeight( _Worksheet sht, int row, double rowHeight )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)row), COleVariant((long)1)).pdispVal);
	cell.SetRowHeight(COleVariant(rowHeight));
}

double CExcelUtil::GetColumnWidth( _Worksheet sht, int col )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)1), COleVariant((long)col)).pdispVal);
	return cell.GetColumnWidth().dblVal;
}

void CExcelUtil::SetColumnWidth( _Worksheet sht, int col, double width )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)1), COleVariant((long)col)).pdispVal);
	cell.SetColumnWidth(COleVariant(width));
}

void CExcelUtil::SetCellFontInfo( _Worksheet sht, int row, int col, int size, bool bBold )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)row), COleVariant((long)col)).pdispVal);
	ExcelLib::Font font;
	font.AttachDispatch(cell.GetFont());
	font.SetBold(COleVariant((long)(bBold ? 1 : 0)));
	font.SetSize(COleVariant((long)size));
}

void CExcelUtil::SetFrame( Range &excelRange, bool bInnerFrame, bool bExteralFrame )
{
	Borders bs;
	bs.AttachDispatch(excelRange.GetBorders());

	if (bInnerFrame)
	{
		Border bLeft;
		bLeft.AttachDispatch(bs.GetItem(7));
		bLeft.SetLineStyle(COleVariant((long)1));
		bLeft.SetWeight(COleVariant((long)3));

		Border bTop;
		bTop.AttachDispatch(bs.GetItem(8));
		bTop.SetLineStyle(COleVariant((long)1));
		bTop.SetWeight(COleVariant((long)3));

		Border bBottom;
		bBottom.AttachDispatch(bs.GetItem(9));
		bBottom.SetLineStyle(COleVariant((long)1));
		bBottom.SetWeight(COleVariant((long)3));

		Border bRight;
		bRight.AttachDispatch(bs.GetItem(10));
		bRight.SetLineStyle(COleVariant((long)1));
		bRight.SetWeight(COleVariant((long)3));
	}

	if (bExteralFrame)
	{
		Border bVertical;
		bVertical.AttachDispatch(bs.GetItem(11));
		bVertical.SetLineStyle(COleVariant((long)1));
		bVertical.SetWeight(COleVariant((long)2));
		
		Border bHorizontal;
		bHorizontal.AttachDispatch(bs.GetItem(12));
		bHorizontal.SetLineStyle(COleVariant((long)1));
		bHorizontal.SetWeight(COleVariant((long)2));
	}
}

void CExcelUtil::SetHorizontalCenter( Range &excelRange )
{
	excelRange.SetHorizontalAlignment(COleVariant((long)-4108));
}

void CExcelUtil::SetHorizontalCenter( _Worksheet sht, int row, int col )
{
	Range excelRange;
	excelRange.AttachDispatch(sht.GetCells());
	Range cell;
	cell.AttachDispatch(excelRange.GetItem(COleVariant((long)row), COleVariant((long)col)).pdispVal);
	SetHorizontalCenter(cell);
}

void CExcelUtil::DwgTableToExcel( ZhifanDwgTable *pTable, int startRowIndex /*= 1*/ )
{
	if (pTable->GetRowCount() > 0)
	{
		// ������excel
		_Application excelApp;
		RunExcelApp(excelApp, false, false);	// ��������ʱ����ʾexcel���û�����ʱ�������������
		
		// ȡ��workBook
		_Workbook excelBook;
		NewWorkBook(excelApp, excelBook);
		
		// ȡ��worksheet
		_Worksheet excelSheet;
		GetActiveWorkSheet(excelBook, excelSheet);
		
		// ��excel��д������
		LPDISPATCH pDispatch = excelSheet.GetCells();
		Range excelRange;
		excelRange.AttachDispatch(pDispatch);
		
		for (int i = 0; i < pTable->GetRowCount(); i++)	// ��
		{
			for (int j = 0; j < pTable->GetColumnCount(); j++)	// ��
			{
				int rowIndex = i + startRowIndex;
				int columnIndex = j + 1;
				CExcelUtil::SetCellValue(excelRange, rowIndex, columnIndex, pTable->GetCellText(i, j));
				//bool bMergeWithOthers = pTable->GetCellInfo(i, j).GetMergeType() == CTableCell::eMerge;		// �ϲ���������Ԫ��
				bool bMerged = pTable->GetCellInfo(i, j).GetMergeType() == CTableCell::eMerge;		// �ϲ���������Ԫ��
				if (bMerged)
				{
					long rows = pTable->GetCellInfo(i, j).GetRowSpan();
					long cols = pTable->GetCellInfo(i, j).GetColumnSpan();
					Range r;
					r.AttachDispatch(excelRange.GetItem(COleVariant((long)rowIndex), COleVariant((long)columnIndex)).pdispVal);
					r.AttachDispatch(r.GetResize(COleVariant(rows), COleVariant(cols)));
					r.Merge(COleVariant((long)0));
				}
			}
		}
		
// 		excelSheet.SaveAs(fileName, vtMissing, vtMissing, vtMissing, vtMissing,
// 			vtMissing, vtMissing, vtMissing, vtMissing, vtMissing);
		
		excelApp.SetVisible(true);
		
		excelBook.DetachDispatch();
		excelSheet.DetachDispatch();
		excelRange.DetachDispatch();
	}
}

void CExcelUtil::SetAutoWrap( Range &excelRange, bool bAutoWrap )
{
	if (bAutoWrap)
	{
		excelRange.SetWrapText(COleVariant((long)1));
	}
	else
	{
		excelRange.SetWrapText(COleVariant((long)0));
	}
}
