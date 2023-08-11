// ViewUtil.cpp: implementation of the CViewUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ViewUtil.h"
#include <dbsymtb.h>
#include "..\Others\ConvertUtil.h"
#include <math.h>
#include "..\Document\DwgDatabaseUtil.h"
#include "..\Geometry\GePointUtil.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CViewUtil::CViewUtil()
{

}

CViewUtil::~CViewUtil()
{

}

void CViewUtil::GetCurrentView( AcDbViewTableRecord &view )
{
	struct resbuf rb;
	struct resbuf wcs, ucs, dcs;	// ת������ʱʹ�õ�����ϵͳ���
	
	wcs.restype = RTSHORT;
	wcs.resval.rint = 0;
	ucs.restype = RTSHORT;
	ucs.resval.rint = 1;
	dcs.restype = RTSHORT;
	dcs.resval.rint = 2;
	
	// ��õ�ǰ�ӿڵ�"�鿴"ģʽ
	acedGetVar(TEXT("VIEWMODE"), &rb);
	view.setPerspectiveEnabled((rb.resval.rint & 1) != 0);
	view.setFrontClipEnabled((rb.resval.rint & 2) != 0);
	view.setBackClipEnabled((rb.resval.rint & 4) != 0);
	view.setFrontClipAtEye((rb.resval.rint & 16) == 0);
	
	// ��ǰ�ӿ�����ͼ�����ĵ㣨UCS���꣩
	acedGetVar(TEXT("VIEWCTR"), &rb);
	acedTrans(rb.resval.rpoint, &ucs, &dcs, 0, rb.resval.rpoint);
	view.setCenterPoint(AcGePoint2d(rb.resval.rpoint[X], 
		rb.resval.rpoint[Y]));	
	
	// ��ǰ�ӿ�͸��ͼ�еľ�ͷ���೤�ȣ���λΪ���ף�
	acedGetVar(TEXT("LENSLENGTH"), &rb);
	view.setLensLength(rb.resval.rreal);
	
	// ��ǰ�ӿ���Ŀ����λ�ã��� UCS �����ʾ��
	acedGetVar(TEXT("TARGET"), &rb);
	acedTrans(rb.resval.rpoint, &ucs, &wcs, 0, rb.resval.rpoint);
	view.setTarget(AcGePoint3d(rb.resval.rpoint[X], 
		rb.resval.rpoint[Y], rb.resval.rpoint[Z]));
	
	// ��ǰ�ӿڵĹ۲췽��UCS��
	acedGetVar(TEXT("VIEWDIR"), &rb);
	acedTrans(rb.resval.rpoint, &ucs, &wcs, 1, rb.resval.rpoint);
	view.setViewDirection(AcGeVector3d(rb.resval.rpoint[X], 
						  rb.resval.rpoint[Y], rb.resval.rpoint[Z]));
	
	// ��ǰ�ӿڵ���ͼ�߶ȣ�ͼ�ε�λ��
	acedGetVar(TEXT("VIEWSIZE"), &rb);
	view.setHeight(rb.resval.rreal);
	double height = rb.resval.rreal;
	
	// ������Ϊ��λ�ĵ�ǰ�ӿڵĴ�С��X �� Y ֵ��
	acedGetVar(TEXT("SCREENSIZE"), &rb);
	view.setWidth(rb.resval.rpoint[X] / rb.resval.rpoint[Y] * height);
	
	// ��ǰ�ӿڵ���ͼŤת��
	acedGetVar(TEXT("VIEWTWIST"), &rb);
	view.setViewTwist(rb.resval.rreal);
	
	// ��ģ��ѡ������һ������ѡ���Ϊ��ǰ
	acedGetVar(TEXT("TILEMODE"), &rb);
	int tileMode = rb.resval.rint;
	// ���õ�ǰ�ӿڵı�ʶ��
	acedGetVar(TEXT("CVPORT"), &rb);
	int cvport = rb.resval.rint;
	
	// �Ƿ���ģ�Ϳռ����ͼ
	bool paperspace = ((tileMode == 0) && (cvport == 1)) ? true : false;
	view.setIsPaperspaceView(paperspace);
	
	if (!paperspace)
	{
		// ��ǰ�ӿ���ǰ�����ƽ�浽Ŀ��ƽ���ƫ����
		acedGetVar(TEXT("FRONTZ"), &rb);
		view.setFrontClipDistance(rb.resval.rreal);
		
		// ��õ�ǰ�ӿں������ƽ�浽Ŀ��ƽ���ƫ��ֵ
		acedGetVar(TEXT("BACKZ"), &rb);
		view.setBackClipDistance(rb.resval.rreal);
	}
	else
	{
		view.setFrontClipDistance(0.0);
		view.setBackClipDistance(0.0);
	}
}

void CViewUtil::Set( const AcGePoint3d &ptMin, const AcGePoint3d &ptMax, double scale )
{
	AcDbViewTableRecord view;
	GetCurrentView(view);

	// �����������������������ϵת������ʾ����ϵ
	AcGePoint3d ptMinDcs = CConvertUtil::WcsToDcsPoint(ptMin);
	AcGePoint3d ptMaxDcs = CConvertUtil::WcsToDcsPoint(ptMax);
	
	// ������ͼ�����ĵ�
	view.setCenterPoint(AcGePoint2d((ptMinDcs.x + ptMaxDcs.x) / 2, (ptMinDcs.y + ptMaxDcs.y) / 2));
	
	// ������ͼ�ĸ߶ȺͿ��
	view.setHeight(fabs(ptMinDcs.y - ptMaxDcs.y) * scale);
	view.setWidth(fabs(ptMinDcs.x - ptMaxDcs.x) * scale);
	
	// ����ͼ��������Ϊ��ǰ��ͼ
	acedSetCurrentView(&view, NULL);
}

void CViewUtil::SetCenter( const AcGePoint3d &center )
{
	AcDbViewTableRecord view;
	GetCurrentView(view);
	
	// �������ĵ����������ϵת������ʾ����ϵ
	AcGePoint3d centerDcs = CConvertUtil::WcsToDcsPoint(center);
	
	// ������ͼ�����ĵ�
	view.setCenterPoint(CConvertUtil::ToPoint2d(centerDcs));
	
	// ����ͼ��������Ϊ��ǰ��ͼ
	acedSetCurrentView(&view, NULL);
}

void CViewUtil::ZoomExtent()
{
	// ��õ�ǰͼ��������ʵ�����С��Χ��
	AcDbBlockTable *pBlkTbl;
	AcDbBlockTableRecord *pBlkTblRcd;
	acdbHostApplicationServices()->workingDatabase()
		->getBlockTable(pBlkTbl, AcDb::kForRead);
	pBlkTbl->getAt(ACDB_MODEL_SPACE, pBlkTblRcd, AcDb::kForRead);
	pBlkTbl->close();
	
	AcDbExtents extent;		// ģ�Ϳռ�İ�Χ��
	extent.addBlockExt(pBlkTblRcd);
	pBlkTblRcd->close();
	
	// ��Χ���ǳ����壬�����屻�任��DCS��֮��ÿ�����㶼�п����������С�ǵ�
	AcGePoint3dArray verts;
	verts.append(extent.minPoint());
	verts.append(AcGePoint3d(extent.maxPoint().x, extent.minPoint().y, extent.minPoint().z));
	verts.append(AcGePoint3d(extent.maxPoint().x, extent.maxPoint().y, extent.minPoint().z));
	verts.append(AcGePoint3d(extent.minPoint().x, extent.maxPoint().y, extent.minPoint().z));
	verts.append(AcGePoint3d(extent.minPoint().x, extent.minPoint().y, extent.maxPoint().z));
	verts.append(AcGePoint3d(extent.maxPoint().x, extent.minPoint().y, extent.maxPoint().z));
	verts.append(extent.maxPoint());
	verts.append(AcGePoint3d(extent.minPoint().x, extent.maxPoint().y, extent.maxPoint().z));
	
	// ������������нǵ�ת�Ƶ�DCS��
	for (int i = 0; i < verts.length(); i++)
	{
		verts[i] = CConvertUtil::WcsToDcsPoint(verts[i]);
	}
	
	// ������нǵ���DCS����С�İ�Χ����
	double xmin = 1.0E30, ymin = 1.0E30;
	double xmax = -1.0E30, ymax = -1.0E30;
	for (int i = 0; i < verts.length(); i++)
	{
		xmin = min(xmin, verts[i].x);
		ymin = min(ymin, verts[i].y);
		xmax = max(xmax, verts[i].x);
		ymax = max(ymax, verts[i].y);
	}
	
	AcDbViewTableRecord view;
	GetCurrentView(view);
	
	// ������ͼ�����ĵ�
	view.setCenterPoint(AcGePoint2d((xmin + xmax) / 2, (ymin + ymax) / 2));
	
	// ������ͼ�ĸ߶ȺͿ��
	view.setHeight(fabs(ymax - ymin));
	view.setWidth(fabs(xmax - xmin));
	
	// ����ͼ��������Ϊ��ǰ��ͼ
	Acad::ErrorStatus es = acedSetCurrentView(&view, NULL);
}

void CViewUtil::DwgZoomExtent( AcDbDatabase *pDb )
{
	assert (pDb);
	
	// ���ģ�Ϳռ�����ʵ�����С��Χ��
	AcDbExtents ext = CDwgDatabaseUtil::GetModelSpaceExtent(pDb);
	
	AcDbViewportTable* pViewportTable = NULL;
	if (pDb->getViewportTable(pViewportTable, AcDb::kForWrite) == Acad::eOk)
	{
		AcDbViewportTableRecord *pRecord = NULL;
		if (pViewportTable->getAt(TEXT("*ACTIVE"), pRecord, AcDb::kForWrite) == Acad::eOk)
		{
			AcGePoint3d center = CGePointUtil::GetMiddlePoint(ext.minPoint(), ext.maxPoint());
			double height = ext.maxPoint().y - ext.minPoint().y;
			double width = ext.maxPoint().x - ext.minPoint().x;
			pRecord->setCenterPoint(CConvertUtil::ToPoint2d(center));
			pRecord->setHeight(height * 1.2);
			pRecord->setWidth(width * 1.2);
			pRecord->close();
		}				
		pViewportTable->close();
	}
}
