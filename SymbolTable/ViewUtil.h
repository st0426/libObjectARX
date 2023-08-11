// ViewUtil.h: interface for the CViewUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VIEWUTIL_H__B066F34E_E9B5_4B50_9966_18A00967B85B__INCLUDED_)
#define AFX_VIEWUTIL_H__B066F34E_E9B5_4B50_9966_18A00967B85B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CViewUtil  
{
public:
	CViewUtil();
	virtual ~CViewUtil();

	// ��õ�ǰ����ͼ����
	static void GetCurrentView(AcDbViewTableRecord &view);

	// ������ʾ��Χ�������С�ǵ���������ű������޸���ͼ
	static void Set(const AcGePoint3d &ptMin, const AcGePoint3d &ptMax, double scale = 1.0);

	// ����ͼ�ƶ������������ĵ�
	static void SetCenter(const AcGePoint3d &center);

	// ��ǰͼ����ʾ����ģ�Ϳռ��ʵ��
	static void ZoomExtent();

	// ��̨������ͼ�����ݿ⣬ģ�Ϳռ�����ʵ�������ʾ
	static void DwgZoomExtent(AcDbDatabase *pDb);
};

#endif // !defined(AFX_VIEWUTIL_H__B066F34E_E9B5_4B50_9966_18A00967B85B__INCLUDED_)
