// circleutil.cpp: implementation of the CCircleUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "circleutil.h"
#include <dbents.h>
#include "..\Document\DwgDatabaseUtil.h"
#include "..\Geometry\GePointUtil.h"
#include <complex>
#include <gearc2d.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCircleUtil::CCircleUtil()
{

}

CCircleUtil::~CCircleUtil()
{

}

AcDbObjectId CCircleUtil::Add( const AcGePoint3d &centerPoint, const AcGeVector3d &normal, double radius, AcDbDatabase *pDb )
{
	AcDbCircle *pCircle = new AcDbCircle(centerPoint, normal, radius);
	return CDwgDatabaseUtil::PostToModelSpace(pCircle, pDb);
}

AcDbObjectId CCircleUtil::Add( const AcGePoint3d &centerPoint, double radius, AcDbDatabase *pDb )
{
	AcGeVector3d vec(0, 0, 1);

	return Add(centerPoint, vec, radius, pDb);
}

AcDbObjectId CCircleUtil::Add( const AcGePoint2d &point1, const AcGePoint2d &point2 )
{
	// ����Բ�ĺͰ뾶
	AcGePoint2d center = CGePointUtil::GetMiddlePoint(point1, point2);
	AcGePoint3d center3d(center.x, center.y, 0);		// Բ��
	double radius = center.distanceTo(point1);
	
	// ����Բ
	return Add(center3d, radius);
}

AcDbObjectId CCircleUtil::Add( const AcGePoint2d &pt1, const AcGePoint2d &pt2, const AcGePoint2d &pt3 )
{
	// ʹ����ѧ����////////////////////////////////////////////////
	double xysm = 0, xyse = 0, xy = 0;
	AcGePoint3d ptCenter;
	double radius = 0;	
	
	xy = pow(pt1.x, 2) + pow(pt1.y, 2);
	xyse = xy - pow(pt3.x, 2) - pow(pt3.y, 2);
	xysm = xy - pow(pt2.x, 2) - pow(pt2.y, 2);
	xy = (pt1.x - pt2.x) * (pt1.y - pt3.y) - (pt1.x - pt3.x) * (pt1.y - pt2.y);
	
	// �жϲ�����Ч��
	if (fabs(xy) < 0.000001)
	{
		AfxMessageBox(TEXT("������Ĳ����޷�����Բ�Σ�"));
		return AcDbObjectId::kNull;
	}
	
	// ���Բ�ĺͰ뾶
	ptCenter.x = (xysm * (pt1.y - pt3.y) - xyse * (pt1.y - pt2.y)) / (2 * xy);
	ptCenter.y = (xyse * (pt1.x - pt2.x) - xysm * (pt1.x - pt3.x)) / (2 * xy);
	ptCenter.z = 0;
	radius = sqrt((pt1.x - ptCenter.x) * (pt1.x - ptCenter.x) + 
		(pt1.y - ptCenter.y) * (pt1.y - ptCenter.y));
	
	if (radius < 0.000001)
	{
		AfxMessageBox(TEXT("�뾶��С��"));
		return AcDbObjectId::kNull;
	}
	
	return Add(ptCenter, radius);

	// ʹ�ü�����///////////////////////////////////////////////////////
// 	AcGeCircArc2d geArc(pt1, pt2, pt3);
// 	AcGePoint3d ptCenter(geArc.center().x, geArc.center().y, 0);
// 	return Add(ptCenter, geArc.radius());
}
