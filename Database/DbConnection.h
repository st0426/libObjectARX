// DbConnection.h: interface for the CDbConnection class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DBCONNECTION_H__CD115B30_432E_4257_9541_CAB651F7D3F8__INCLUDED_)
#define AFX_DBCONNECTION_H__CD115B30_432E_4257_9541_CAB651F7D3F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#if defined (_ZFGKCOMMONLIB_)
#define ZFGK_DLLIMPEXP __declspec(dllexport)
#else
#define ZFGK_DLLIMPEXP 
#endif

class ZFGK_DLLIMPEXP CDbConnection  
{
public:
	CDbConnection(const TCHAR* connectionString);
	virtual ~CDbConnection();

	// ������
	bool Open();

	// �ر�����
	void Close();

	// �Ƿ���Access����
	void SetIsAccessConnection(bool bAccessCon);

	// ��ʼ������
	void BeginTrans();

	// �ύ����
	void CommitTrans();
	
	// �ع�����
	void RollbackTrans();

	// �ͷ����ݿ�����
	static void Release();

	// �Ƿ����ĳ����
	bool TableExist(const TCHAR *tableName);

	// ĳ�������Ƿ���ڸ������ֶ�
	bool FieldExist(const TCHAR* tableName, const TCHAR* fieldName);

protected:
	CDbConnection();

private:
	// ������Ӷ����Comָ��
	_ConnectionPtr GetConnectionPtr();
	friend class CDbCommand;

	// �ж���������COM�Ƿ��Ѿ��رգ���ֹ�رն��
	bool m_bClosed;
	bool m_bAccessConnection;		// �Ƿ���Access���ݿ⣬����ǾͲ�����ִ��Close����ʱ�����ر����ݿ����ӣ����򽫹ر����ݿ�����

private:
	_ConnectionPtr m_pConnection;
	CString m_strConnectionString;
	
	static _ConnectionPtr m_pStConnection;
	static CString m_strStConString;
};

#endif // !defined(AFX_DBCONNECTION_H__CD115B30_432E_4257_9541_CAB651F7D3F8__INCLUDED_)
