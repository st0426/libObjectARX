// EntityUtil.h: interface for the CEntityUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ENTITYUTIL_H__848573C8_AD29_4B2A_BFFB_6DF0EAFCB34F__INCLUDED_)
#define AFX_ENTITYUTIL_H__848573C8_AD29_4B2A_BFFB_6DF0EAFCB34F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CEntityUtil  
{
public:
	CEntityUtil();
	virtual ~CEntityUtil();

	// �޸�ʵ�������/////////////////////////////////////////////////////
	// ����ʵ�����ɫ
	static void SetColor(AcDbObjectId entId, int colorIndex);

	// ����ʵ���ͼ��
	static void SetLayer(AcDbObjectId entId, const TCHAR* layerName);

	// ����ʵ�������
	static void SetLinetype(AcDbObjectId entId, const TCHAR* linetype);

	// ɾ��һ��ʵ��
	static void Erase(AcDbObjectId entId);

	// ���α任///////////////////////////////////////////////////////////
	// ��ά��ת
	static Acad::ErrorStatus Rotate(AcDbObjectId entId, 
		const AcGePoint2d &ptBase, double rotation);

	// �ƶ�
	static Acad::ErrorStatus Move(AcDbObjectId entId, const AcGePoint3d &ptBase, 
		const AcGePoint3d &ptDest);

	// ��������
	static Acad::ErrorStatus Scale(AcDbObjectId entId, 
		const AcGePoint3d &ptBase, double scaleFactor);
};

#endif // !defined(AFX_ENTITYUTIL_H__848573C8_AD29_4B2A_BFFB_6DF0EAFCB34F__INCLUDED_)
