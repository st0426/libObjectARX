// GePointUtil.h: interface for the CGePointUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GEPOINTUTIL_H__4CCA822B_9551_4997_9CCC_EDCFF5F3A5FE__INCLUDED_)
#define AFX_GEPOINTUTIL_H__4CCA822B_9551_4997_9CCC_EDCFF5F3A5FE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include <acaplmgr.h>

class CGePointUtil  
{
public:
	CGePointUtil();
	virtual ~CGePointUtil();

	// ������������е�
	static AcGePoint3d GetMiddlePoint(const AcGePoint3d &startPoint, const AcGePoint3d &endPoint);
	static AcGePoint2d GetMiddlePoint(const AcGePoint2d &startPoint, const AcGePoint2d &endPoint);

	// ���ݼ�����ķ�ʽ����һ���������
	static AcGePoint3d PolarPoint(const AcGePoint3d &basePoint, double angle, double length);
	static AcGePoint2d PolarPoint(const AcGePoint2d &basePoint, double angle, double length);

	// ����ֱ����������һ���������
	static AcGePoint3d RelativePoint(const AcGePoint3d& pt, double x, double y);

	// �������Ƿ���ͬ
	static bool IsEqual(const AcGePoint3d &firstPoint, const AcGePoint3d &secondPoint, double tol = 1.0E-7);
	static bool IsEqual(const AcGePoint2d &firstPoint, const AcGePoint2d &secondPoint, double tol = 1.0E-7);

	// �������Ƿ����ĳ����
	// ����: ���������е��������������ֵС��0��ʾ�����в����������
	static int FindPoint(const AcGePoint3dArray &points, const AcGePoint3d &point, double tol = 1.0E-7);
	static int FindPoint(const AcGePoint2dArray &points, const AcGePoint2d &point, double tol = 1.0E-7);

	// �������й��˵��ظ���
	// points: ������
	// tol: �жϵ��غ�ʱ�ľ��ȣ�����֮��ľ���С��tol��Ϊ���������غϣ�
	static void FilterEqualPoints(AcGePoint3dArray &points, double tol = 1.0E-7);
	static void FilterEqualPoints(AcGePoint3dArray &points, const AcGePoint2d &pt, double tol = 1.0E-7);
};

#endif // !defined(AFX_GEPOINTUTIL_H__4CCA822B_9551_4997_9CCC_EDCFF5F3A5FE__INCLUDED_)
