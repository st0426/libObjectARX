// PolylineUtil.h: interface for the CPolylineUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_POLYLINEUTIL_H__EEB1ED65_B3FE_4181_AD86_5B54BF6533BA__INCLUDED_)
#define AFX_POLYLINEUTIL_H__EEB1ED65_B3FE_4181_AD86_5B54BF6533BA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <acaplmgr.h>
#include <dbpl.h>

class CPolylineUtil  
{
public:
	CPolylineUtil();
	virtual ~CPolylineUtil();

	// �����Ż������
	static AcDbObjectId Add(const AcGePoint2dArray &points, double width = 0);
	static AcDbObjectId Add(const AcGePoint2d &ptStart, const AcGePoint2d &ptEnd, 
		double width = 0);

	// ������ά�����
	static AcDbObjectId Add3dPolyline(const AcGePoint3dArray &points);

	// �����������
	static AcDbObjectId AddPolygon(const AcGePoint2d &ptCenter, int number, 
		double radius, double rotation, double width);

	// ��������
	static AcDbObjectId AddRectangle(const AcGePoint2d &pt1, const AcGePoint2d &pt2, 
		double width);

	// ����Բ�εĶ����
	static AcDbObjectId AddPolyCircle(const AcGePoint2d &ptCenter, double radius, double width);

	// ����һ��Բ���Ķ����
	static AcDbObjectId AddPolyArc(const AcGePoint2d &ptCenter, double radius, 
		double angleStart, double angleEnd, double width);

	// ��Ͷ���ߵ�λ�ù�ϵ
	// ����ֵ��-1��ʾ�ڶ�����ⲿ��0��ʾ�ڶ�����ϣ�1��ʾ�ڶ�����ڲ�
	static int PtRelationToPoly(AcDbPolyline *pPoly, const AcGePoint2d &pt, double tol = 1.0E-7);
};

#endif // !defined(AFX_POLYLINEUTIL_H__EEB1ED65_B3FE_4181_AD86_5B54BF6533BA__INCLUDED_)
