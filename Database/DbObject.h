// DbObject.h: interface for the CDbObject class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DBOBJECT_H__21ED5A84_0F8E_4672_946A_5C47BA6F575B__INCLUDED_)
#define AFX_DBOBJECT_H__21ED5A84_0F8E_4672_946A_5C47BA6F575B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#if defined (_ZFGKCOMMONLIB_)
#define ZFGK_DLLIMPEXP __declspec(dllexport)
#else
#define ZFGK_DLLIMPEXP 
#endif

class ZFGK_DLLIMPEXP CDbObject  
{
public:
	CDbObject();
	CDbObject(const COleVariant &variant);
	virtual ~CDbObject();

	// ʹ��һ��COleVariant��������ʼ��CDbObject����
	void Set(const COleVariant &variant);

	// ת��Ϊ�ַ���
	CString ToString();

	// ת��Ϊ����
	int ToInteger();

	// ת��Ϊshort
	short ToShort();
	
	// ת��Ϊ������
	long ToLong();

	// ת��Ϊ�ֽ�
	byte ToByte();

	// �Ƿ�Ϊ��
	bool IsNull();
	
	// ת��Ϊbool
	bool ToBool();
	
	// ת��Ϊdouble
	double ToDouble();

	// ת��ΪCTime
	CTime ToDateTime();

private:
	void SetNull();

private:
	COleVariant m_variant;
};

#endif // !defined(AFX_DBOBJECT_H__21ED5A84_0F8E_4672_946A_5C47BA6F575B__INCLUDED_)
