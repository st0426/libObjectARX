// LineUtil.cpp: implementation of the CLineUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "LineUtil.h"
#include "..\Document\DwgDatabaseUtil.h"
#include <dbents.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CLineUtil::CLineUtil()
{

}

CLineUtil::~CLineUtil()
{

}



AcDbObjectId CLineUtil::Add( const AcGePoint3d &startPoint, const AcGePoint3d &endPoint, AcDbDatabase *pDb )
{
	// ����ֱ��ʵ��
	AcDbLine *pLine = new AcDbLine(startPoint, endPoint);

	// ��ֱ��ʵ����ӵ�ģ�Ϳռ�
	return CDwgDatabaseUtil::PostToModelSpace(pLine, pDb);
}

bool CLineUtil::ThreePointIsCollinear( const AcGePoint2d &pt1, const AcGePoint2d &pt2, const AcGePoint2d &pt3 )
{
	double xy = pt1.x * pt1.x + pt1.y * pt1.y;
	double xyse = xy - pt3.x * pt3.x - pt3.y * pt3.y;
	double xysm = xy - pt2.x * pt2.x - pt2.y * pt2.y;
	xy = (pt1.x - pt2.x) * (pt1.y - pt3.y) - (pt1.x - pt3.x) * (pt1.y - pt2.y);	
	
	return (fabs(xy) < 1.0E-5);
}

int CLineUtil::PtInLeftOfLine( const AcGePoint3d &ptStart, const AcGePoint3d &ptEnd, const AcGePoint3d &pt, double tol /*= 1.0E-7*/ )
{
	return PtInLeftOfLine(ptStart.x, ptStart.y, ptEnd.x, ptEnd.y, pt.x, pt.y, tol);	
}

int CLineUtil::PtInLeftOfLine( const AcGePoint2d &ptStart, const AcGePoint2d &ptEnd, const AcGePoint2d &pt, double tol /*= 1.0E-7*/ )
{
	return PtInLeftOfLine(ptStart.x, ptStart.y, ptEnd.x, ptEnd.y, pt.x, pt.y, tol);	
}

int CLineUtil::PtInLeftOfLine( double x1, double y1, double x2, double y2, double x3, double y3, double tol /*= 1.0E-7*/ )
{
	// ����ʸ���Ĳ�˽����һ����ʸ��������ʽֵ��������ʸ��ȷ����ƽ���ı��ε����
	double a = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	if (fabs(a) < tol)
	{
		return 0;
	}
	else if (a > 0)
	{
		return 1;
	}
	else
	{
		return - 1;
	}
}
