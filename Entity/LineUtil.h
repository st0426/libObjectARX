// LineUtil.h: interface for the CLineUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LINEUTIL_H__7B01AAA2_B236_4E0C_A689_D549EE8A77D1__INCLUDED_)
#define AFX_LINEUTIL_H__7B01AAA2_B236_4E0C_A689_D549EE8A77D1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CLineUtil  
{
public:
	CLineUtil();
	virtual ~CLineUtil();

	// ���һ��ֱ��
	static AcDbObjectId Add(const AcGePoint3d &startPoint, const AcGePoint3d &endPoint, 
		AcDbDatabase *pDb = acdbHostApplicationServices()->workingDatabase());

	// �����Ƿ���
	static bool ThreePointIsCollinear(const AcGePoint2d &pt1, const AcGePoint2d &pt2, const AcGePoint2d &pt3);

	// �жϵ��Ƿ���ֱ�ߵ���ࣨ��ֱ�ߵ���㵽�յ�Ϊ�۲췽��
	// tol: ����ʸ��������ƽ���ı��ε���������ڷ���ֵΪ0��������������ԵرȽ�
	// ����ֵ��1��ʾ��ֱ�ߵ���࣬0��ʾ��ֱ���ϣ�-1��ʾ��ֱ�ߵ��Ҳ�
	static int PtInLeftOfLine(const AcGePoint3d &ptStart, const AcGePoint3d &ptEnd, const AcGePoint3d &pt, double tol = 1.0E-7);
	static int PtInLeftOfLine(const AcGePoint2d &ptStart, const AcGePoint2d &ptEnd, const AcGePoint2d &pt, double tol = 1.0E-7);
	static int PtInLeftOfLine(double x1, double y1, double x2, double y2, double x3, double y3, double tol = 1.0E-7);
};

#endif // !defined(AFX_LINEUTIL_H__7B01AAA2_B236_4E0C_A689_D549EE8A77D1__INCLUDED_)
