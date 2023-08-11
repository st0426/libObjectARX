// RegionUtil.cpp: implementation of the CRegionUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "RegionUtil.h"
#include <dbcurve.h>
#include <dbregion.h>
#include "..\Document\DwgDatabaseUtil.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRegionUtil::CRegionUtil()
{

}

CRegionUtil::~CRegionUtil()
{

}

AcDbObjectIdArray CRegionUtil::Add( const AcDbObjectIdArray& curveIds )
{
	AcDbObjectIdArray regionIds;	// ���ɵ������ID����
	AcDbVoidPtrArray curves;		// ָ����Ϊ����߽�����ߵ�ָ�������
	AcDbVoidPtrArray regions;		// ָ�򴴽�����������ָ�������
	AcDbEntity *pEnt = NULL;				// ��ʱָ�룬�����رձ߽�����
	AcDbRegion *pRegion = NULL;		// ��ʱ����������������ӵ�ģ�Ϳռ�
	
	// ��curveIds��ʼ��curves
	for (int i = 0; i < curveIds.length(); i++)
	{		
		acdbOpenAcDbEntity(pEnt, curveIds.at(i), AcDb::kForRead);
		if (pEnt->isKindOf(AcDbCurve::desc()))
		{
			curves.append(static_cast<void*>(pEnt));
		}
	}
	
	Acad::ErrorStatus es = AcDbRegion::createFromCurves(curves, regions);
	if (es == Acad::eOk)
	{
		// �����ɵ�������ӵ�ģ�Ϳռ�
		for (i = 0; i < regions.length(); i++)
		{
			// ����ָ�루��ָ���κ����ͣ�ת��Ϊָ�������ָ��
			pRegion = static_cast<AcDbRegion*>(regions[i]);
			pRegion->setDatabaseDefaults();
			AcDbObjectId regionId;
			regionId = CDwgDatabaseUtil::PostToModelSpace(pRegion);
			regionIds.append(regionId);
		}
	}
	else	// ����������ɹ���ҲҪɾ���Ѿ����ɵ�����
	{
		for (i = 0; i < regions.length(); i++)
		{
			delete (AcRxObject*)regions[i];
		}
	}
	
	// �ر���Ϊ�߽�Ķ���
	for (i = 0; i < curves.length(); i++)
	{
		pEnt = static_cast<AcDbEntity*>(curves[i]);
		pEnt->close();
	}
	
	return regionIds;
}
