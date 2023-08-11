// PolylineUtil.cpp: implementation of the CPolylineUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PolylineUtil.h"
#include <dbpl.h>
#include "..\Document\DwgDatabaseUtil.h"
#include <dbents.h>
#include "..\Geometry\MathUtil.h"
#include <complex>
#include "EntityUtil.h"
#include <geray2d.h>
#include "..\Geometry\GePointUtil.h"
#include "..\Others\ConvertUtil.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CPolylineUtil::CPolylineUtil()
{

}

CPolylineUtil::~CPolylineUtil()
{

}

AcDbObjectId CPolylineUtil::Add( const AcGePoint2dArray &points, double width )
{
	int numVertices = points.length();	
	AcDbPolyline *pPoly = new AcDbPolyline(numVertices);
	
	for (int i = 0; i < numVertices; i++)
	{
		pPoly->addVertexAt(i, points.at(i), 0, width, width);
	}
	
	AcDbObjectId polyId;
	polyId = CDwgDatabaseUtil::PostToModelSpace(pPoly);
	
	return polyId;
}

AcDbObjectId CPolylineUtil::Add( const AcGePoint2d &ptStart, const AcGePoint2d &ptEnd, double width )
{
	AcGePoint2dArray points;
	points.append(ptStart);
	points.append(ptEnd);
	
	return Add(points, width);
}

AcDbObjectId CPolylineUtil::Add3dPolyline( const AcGePoint3dArray &points )
{
	AcGePoint3dArray verts = points;
	AcDb3dPolyline *pPoly3d = new AcDb3dPolyline(AcDb::k3dSimplePoly, verts);
	
	return CDwgDatabaseUtil::PostToModelSpace(pPoly3d);
}

AcDbObjectId CPolylineUtil::AddPolygon( const AcGePoint2d &ptCenter, int number, double radius, 
									   double rotation, double width )
{	
	double angle = 2 * CMathUtil::PI() / (double)number;
	AcGePoint2dArray points;
	for (int i = 0; i < number; i++)
	{
		AcGePoint2d pt;
		pt.x = ptCenter.x + radius * cos(i * angle);
		pt.y = ptCenter.y + radius * sin(i * angle);
		
		points.append(pt);
	}
	
	AcDbObjectId polyId = Add(points, width);
	
	// ����պ�
	AcDbEntity *pEnt = NULL;
	if (acdbOpenAcDbEntity(pEnt, polyId, AcDb::kForWrite) == Acad::eOk)
	{
		AcDbPolyline *pPoly = AcDbPolyline::cast(pEnt);
		if (pPoly != NULL)
		{
			pPoly->setClosed(Adesk::kTrue);
		}
		pEnt->close();
	}
	
	CEntityUtil::Rotate(polyId, ptCenter, rotation);
	
	return polyId;
}

AcDbObjectId CPolylineUtil::AddRectangle( const AcGePoint2d &pt1, const AcGePoint2d &pt2, double width )
{
	// ��ȡ�����ǵ������ֵ
	double x1 = pt1.x, x2 = pt2.x;
	double y1 = pt1.y, y2 = pt2.y;
	
	// ������εĽǵ�
	AcGePoint2d ptLeftBottom(min(x1, x2), min(y1, y2));
	AcGePoint2d ptRightBottom(max(x1, x2), min(y1, y2));
	AcGePoint2d ptRightTop(max(x1, x2), max(y1, y2));
	AcGePoint2d ptLeftTop(min(x1, x2), max(y1, y2));
	
	// ������Ӧ�Ķ����
	AcDbPolyline *pPoly = new AcDbPolyline(4);
	pPoly->addVertexAt(0, ptLeftBottom, 0, width, width);
	pPoly->addVertexAt(1, ptRightBottom, 0, width, width);
	pPoly->addVertexAt(2, ptRightTop, 0, width, width);
	pPoly->addVertexAt(3, ptLeftTop, 0, width, width);
	pPoly->setClosed(Adesk::kTrue);
	
	// ���������ӵ�ģ�Ϳռ�
	AcDbObjectId polyId;
	polyId = CDwgDatabaseUtil::PostToModelSpace(pPoly);
	
	return polyId;
}

AcDbObjectId CPolylineUtil::AddPolyCircle( const AcGePoint2d &ptCenter, double radius, double width )
{
	// ���㶥���λ��
	AcGePoint2d pt1, pt2, pt3;
	pt1.x = ptCenter.x + radius;
	pt1.y = ptCenter.y;
	pt2.x = ptCenter.x - radius;
	pt2.y = ptCenter.y;
	pt3.x = ptCenter.x + radius;
	pt3.y = ptCenter.y;
	
	// ���������
	AcDbPolyline *pPoly = new AcDbPolyline(3);
	pPoly->addVertexAt(0, pt1, 1, width, width);
	pPoly->addVertexAt(1, pt2, 1, width, width);
	pPoly->addVertexAt(2, pt3, 1, width, width);
	pPoly->setClosed(Adesk::kTrue);
	
	// ���������ӵ�ģ�Ϳռ�
	AcDbObjectId polyId = CDwgDatabaseUtil::PostToModelSpace(pPoly);
	
	return polyId;
}

AcDbObjectId CPolylineUtil::AddPolyArc( const AcGePoint2d &ptCenter, double radius, 
									   double angleStart, double angleEnd, double width )
{
	// ���㶥���λ��
	AcGePoint2d pt1, pt2;
	pt1.x = ptCenter.x + radius * cos(angleStart);
	pt1.y = ptCenter.y + radius * sin(angleStart);
	pt2.x = ptCenter.x + radius * cos(angleEnd);
	pt2.y = ptCenter.y + radius * sin(angleEnd);
	
	// ���������
	AcDbPolyline *pPoly = new AcDbPolyline(3);
	pPoly->addVertexAt(0, pt1, tan((angleEnd - angleStart) / 4), width, width);
	pPoly->addVertexAt(1, pt2, 0, width, width);	
	
	// ���������ӵ�ģ�Ϳռ�
	AcDbObjectId polyId = CDwgDatabaseUtil::PostToModelSpace(pPoly);
	
	return polyId;
}

// ���������ߺͶ���߼��㽻��
static void IntersectWithGeRay( AcDbPolyline *pPoly, const AcGeRay2d &geRay, AcGePoint3dArray &intPoints, double tol /*= 1.0E-7*/ )
{
	intPoints.setLogicalLength(0);
	AcGePoint2dArray intPoints2d;
	
	// ����ߵ�ÿһ�ηֱ������߼��㽻��
	AcGeTol geTol;
	geTol.setEqualPoint(tol);
	for (int i = 0; i < pPoly->numVerts(); i++)
	{
		if (i < pPoly->numVerts() - 1 || pPoly->isClosed() == Adesk::kTrue)
		{
			double bulge = 0;
			pPoly->getBulgeAt(i, bulge);
			if (fabs(bulge) < 1.0E-7)
			{
				// ������������߶������㽻��
				AcGeLineSeg2d geLine;
				Acad::ErrorStatus es = pPoly->getLineSegAt(i, geLine);
				AcGePoint2d intPoint;
				if (geLine.intersectWith(geRay, intPoint, geTol) == Adesk::kTrue)
				{
					if (CGePointUtil::FindPoint(intPoints2d, intPoint, tol) < 0)
					{
						intPoints2d.append(intPoint);
					}
				}
			}
			else
			{
				// �����������Բ�������㽻��
				AcGeCircArc2d geArc;
				pPoly->getArcSegAt(i, geArc);
				AcGePoint2d pt1, pt2;
				int count = 0;
				if (geArc.intersectWith(geRay, count, pt1, pt2, geTol) == Adesk::kTrue)
				{
					if (CGePointUtil::FindPoint(intPoints2d, pt1, tol) < 0)
					{
						intPoints2d.append(pt1);
					}
					if (count > 1 && CGePointUtil::FindPoint(intPoints2d, pt2, tol) < 0)
					{
						intPoints2d.append(pt2);
					}
				}
			}
		}
	}
	
	double z = pPoly->elevation();
	for (i = 0; i < intPoints2d.length(); i++)
	{
		intPoints.append(AcGePoint3d(intPoints2d[i].x, intPoints2d[i].y, z));
	}
}

// ���Ƿ��Ƕ���ߵĶ���
static bool PointIsPolyVert( AcDbPolyline *pPoly, const AcGePoint2d &pt, double tol )
{
	for (int i = 0; i < (int)pPoly->numVerts(); i++)
	{
		AcGePoint3d vert;
		pPoly->getPointAt(i, vert);
		
		if (CGePointUtil::IsEqual(CConvertUtil::ToPoint2d(vert), pt, tol))
		{
			return true;
		}
	}
	
	return false;
}

int CPolylineUtil::PtRelationToPoly( AcDbPolyline *pPoly, const AcGePoint2d &pt, double tol /*= 1.0E-7*/ )
{
	assert (pPoly);

	// 1.����㵽����ߵ������͸����ĵ��غϣ���ʾ���ڶ������
	AcGePoint3d closestPoint;
	pPoly->getClosestPointTo(CConvertUtil::ToPoint3d(pt, pPoly->elevation()), closestPoint);		// ���������������������ĵ�	
	if (fabs(closestPoint.x - pt.x) < tol && fabs(closestPoint.y - pt.y) < tol)			// ���ڶ������
	{
		return 0;
	}

	// 2.��һ�����ߵķ����Ǵ�����㵽��ǰ�㣬����ǵ�ǰ��
	// ���ߵ������pt������Ϊ������㵽pt������������жϣ�����������pt̫����ʱ�������Ҳ�ᱻ��Ϊһ�����㣨������㲻̫���ױ��ų�����
	// ���⣬���������߷���������жϳ��㲻���ڲ������
	AcGeVector3d vec(-(closestPoint[X] - pt[X]), -(closestPoint[Y] - pt[Y]), 0);
	AcGeRay2d geRay(AcGePoint2d(pt.x, pt.y), AcGePoint2d(pt.x + vec.x, pt.y + vec.y));

	// 3.���������߼��㽻��
	AcGePoint3dArray intPoints;
	IntersectWithGeRay(pPoly, geRay, intPoints, 1.0E-4);
	// IntersectWith����������õ��ܽ��Ľ��㣬��Щ�������й���
	CGePointUtil::FilterEqualPoints(intPoints, 1.0E-4);	

	// 4.�жϵ�Ͷ���ߵ�λ�ù�ϵ
RETRY:
	// 4.1 ������ߺͶ����û�н��㣬��ʾ���ڶ���ߵ��ⲿ
	if (intPoints.length() == 0)
	{
			return -1;
	}
	else
	{
		// 3.1 ���˵��������߱������ӳ�������Ӱ��
		CGePointUtil::FilterEqualPoints(intPoints, CConvertUtil::ToPoint2d(closestPoint));		// 2008-0907�޶���¼����pt���������ȽϽ���ʱ������㾹Ȼ����Ϊһ�����㣡
		// 3.2 ���ĳ��������������ڸ������ͬһ����Ҫȥ������㣨��������Բ��ǽ��㣬��������intersectwith������Bug��	
		for (int i = intPoints.length() - 1; i >= 0; i--)
		{
			if ((intPoints[i][X] - pt[X]) * (closestPoint[X] - pt[X]) >= 0 && 
				(intPoints[i][Y] - pt[Y]) * (closestPoint[Y] - pt[Y]) >= 0)
			{
				intPoints.removeAt(i);
			}
		}

		int count = intPoints.length();
		for (i = 0; i < intPoints.length(); i++)
		{
			if (PointIsPolyVert(pPoly, CConvertUtil::ToPoint2d(intPoints[i]), 1.0E-4))		// ֻҪ�н����Ƕ���ߵĶ�������½����ж�
			{
				// ���������ܿ�������߶�������(����붥�����ܽ�������Ϊ������ڶ�����ϣ���Ϊ�������û��ʲô�õ��жϷ���)
				if (PointIsPolyVert(pPoly, AcGePoint2d(pt.x, pt.y), 1.0E-4))
				{
					return 0;
				}

				// ��������תһ����С�ĽǶ�(2��)�ٴ��жϣ��ٶ�����������ͨ���ϴ��жϵ��Ķ��㣩
				vec = vec.rotateBy(0.035, AcGeVector3d::kZAxis);				
				geRay.set(AcGePoint2d(pt.x, pt.y), AcGePoint2d(pt.x + vec.x, pt.y + vec.y));
				intPoints.setLogicalLength(0);
				IntersectWithGeRay(pPoly, geRay, intPoints, 1.0E-4);
				goto RETRY;		// �����жϽ��
			}
		}
		
		if (count % 2 == 0)
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}
}
