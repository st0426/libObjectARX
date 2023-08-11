// DbCommand.h: interface for the CDbCommand class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DBCOMMAND_H__1A146D13_AE69_4F1E_A4BF_6847C0B61A61__INCLUDED_)
#define AFX_DBCOMMAND_H__1A146D13_AE69_4F1E_A4BF_6847C0B61A61__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "DbConnection.h"
#include "DbObject.h"
#include "DbDataReader.h"
#include <set>

#if defined (_ZFGKCOMMONLIB_)
#define ZFGK_DLLIMPEXP __declspec(dllexport)
#else
#define ZFGK_DLLIMPEXP 
#endif

class ZFGK_DLLIMPEXP CDbCommand  
{
public:	
	CDbCommand(const TCHAR* sql, CDbConnection* pCon);
	CDbCommand(CDbConnection* pCon);
	CDbCommand();
	virtual ~CDbCommand();
	
	//************************************
	// ����: ִ��û�з���ֵ�Ĳ�ѯ
	// ����ֵ:   int, ��Ӱ�������
	// ����: �ŷ� -- 2008-11-5 17:40
	//************************************
	int ExecuteNonQuery();

	// ����һ���������һ�е�һ�еĲ�ѯ
	CDbObject ExecuteScalar();

	// ����һ��Reader�Ĳ�ѯ(�����߲���Ҫ�ͷ�ָ��ָ����ڴ�)
	CDbDataReader* ExecuteReader();

	// ����SQL�ַ���
	void SetCommandText(const TCHAR* sql);
	
	// ɾ��һ�������������
	bool DeleteFromTable(const TCHAR* tableName);

	// ����һ�����еĶ������ֶ�(��ӦSqlServer���ݿ��varbinary(MAX)�ֶ�)
	// tableName: ����
	// pkName��������
	// fieldName���������ֶ���
	// pkId: Ҫ���µļ�¼������ID
	// fileName��Ҫ���µ��ļ���
	bool UpdateBinaryField(const TCHAR* tableName, const TCHAR* pkName, const TCHAR* fieldName, int pkId, const TCHAR* fileName);

	// ��ȡһ�����еĶ������ֶ�(��ӦSqlServer���ݿ��varbinary(MAX)�ֶ�)�����䱣��Ϊһ���ļ�
	// tableName: ����
	// pkName��������
	// fieldName���������ֶ���
	// pkId: Ҫ���µļ�¼������ID
	// fileName��Ҫ������ļ���
	bool GetBinaryField(const TCHAR* tableName, const TCHAR* pkName, const TCHAR* fieldName, int pkId, const TCHAR* fileName);

	//*****************************************************
	// ����: Ϊ��ǰ���������Ӳ���
	// ����: const char * name, ��������
	// ����: DataTypeEnum type, ��������(���������ı�����)
	// ����: long size, ������С
	// ����: long value, ����ֵ
	// ����ֵ: void, 
	// ����: ����ǿ��2008-12-16 18:27
	//*****************************************************
	//void AddParameter( const char* name, DataTypeEnum type, long size, long value );

	//*****************************************************
	// ����: Ϊ��ǰ���������Ӳ���
	// ����: const char * name, ��������
	// ����: DataTypeEnum type, ��������(���������ı�����)
	// ����: long size, ������С
	// ����: const char * value, ����ֵ
	// ����ֵ: void, ��
	// ����: ����ǿ��2008-12-16 16:00
	//*****************************************************
	//void AddParameter( const char* name, DataTypeEnum type, long size, const char* value );

	// ����ַ������͵��������
	void AddCharParameter(const TCHAR *name, const TCHAR *value, long size = 50);
	void AddVarcharParameter(const TCHAR *name, const TCHAR* value, long size = 50);

	// ����ֽ����͵��������
	void AddByteParameter(const TCHAR *name, byte value);

	// ������͵��������
	// bool bZeroAsNull, �������Ĳ�����0����ô��ӵ����ݿ�ʱ�Ƿ���ָ��ΪNULLֵ
	void AddIntParameter(const TCHAR *name, int value, bool bZeroAsNull = false);

	// ���ʵ�����͵��������
	void AddDoubleParameter(const TCHAR *name, double value);

	// ��ӳ����͵��������
	void AddLongParameter(const TCHAR *name, long value);

	// ��Ӳ������͵��������
	void AddBoolParameter(const TCHAR* name, bool value);

	// ɾ��ĳ������
	void DeleteParameter(const TCHAR *name);

private:
	//*****************************************************
	// ����: ��ʼ���������
	// ����ֵ: void, ��
	// ����: ����ǿ��2008-12-16 17:42
	//*****************************************************
	void Init();	

private:
	CString m_sql;
	CDbConnection* m_pCon;
	CDbDataReader m_dataReader;
	_CommandPtr m_cmd;
	std::set<CString> m_paramSet;

	//_RecordsetPtr m_pRecordset;
};

#endif // !defined(AFX_DBCOMMAND_H__1A146D13_AE69_4F1E_A4BF_6847C0B61A61__INCLUDED_)
