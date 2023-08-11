// HatchUtil.cpp: implementation of the CHatchUtil class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "HatchUtil.h"
#include <dbhatch.h>
#include "..\Document\DwgDatabaseUtil.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHatchUtil::CHatchUtil()
{

}

CHatchUtil::~CHatchUtil()
{

}

AcDbObjectId CHatchUtil::Add( const AcDbObjectIdArray &loopIds, const TCHAR* patName, double patternScale )
{
	Acad::ErrorStatus es;
	AcDbHatch *pHatch = new AcDbHatch();
	
	// �������ƽ��
	AcGeVector3d normal(0, 0, 1);
	pHatch->setNormal(normal);
	pHatch->setElevation(0);
	
	// ���ù�����
	pHatch->setAssociative(true);
	
	// �������ͼ��
	pHatch->setPattern(AcDbHatch::kPreDefined, patName);
	
	// ������߽�
	es = pHatch->appendLoop(AcDbHatch::kExternal, loopIds);
	
	// ��ʾ������
	es = pHatch->evaluateHatch();	
	
	// ��ӵ�ģ�Ϳռ�
	AcDbObjectId hatchId = CDwgDatabaseUtil::PostToModelSpace(pHatch);
	
	// ����ǹ����Ե���䣬����������߽�󶨣��Ա�ʹ���ܻ�ñ߽�����޸ĵ�֪ͨ
	if (acdbOpenObject(pHatch, hatchId, AcDb::kForRead) == Acad::eOk)
	{
		AcDbObjectIdArray assocEntIds;
		pHatch->getAssocObjIds(assocEntIds);
		for (int i = 0; i < assocEntIds.length(); i++)
		{
			AcDbEntity *pEnt = NULL;
			if (acdbOpenObject(pEnt, assocEntIds[i], AcDb::kForWrite) == Acad::eOk)
			{
				// ���һ�����÷�Ӧ��
				pEnt->addPersistentReactor(hatchId);
				pEnt->close();
			}
		}
		
		pHatch->close();
	}
	
	return hatchId;
}
