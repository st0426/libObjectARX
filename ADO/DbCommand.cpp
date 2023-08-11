// DbCommand.cpp: implementation of the CDbCommand class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "DbCommand.h"
#include <stdexcept>
#include <io.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CDbCommand::CDbCommand(const TCHAR* sql, CDbConnection* pCon)// : m_pRecordset(NULL)
{
	m_sql = sql;
	m_pCon = pCon;

	Init();
}

CDbCommand::CDbCommand(CDbConnection* pCon) : m_sql(TEXT(""))//, m_pRecordset(NULL)
{
	m_pCon = pCon;

	Init();
}

CDbCommand::CDbCommand() : m_pCon(NULL), m_sql(TEXT(""))//, m_pRecordset(NULL)
{
	Init();
}

CDbCommand::~CDbCommand()
{

}
//////////////////////////////////////////////////////////////////////////
int CDbCommand::ExecuteNonQuery()
{
	assert (m_pCon);
	assert (m_sql.GetLength() > 0);

	try
	{
		COleVariant vaRowsAffected;
		m_cmd->Execute(&vaRowsAffected, NULL, adCmdText);		

		return (int)vaRowsAffected.intVal;
	}
	catch (_com_error e)
	{		
		_bstr_t   bstrSource(e.Source()); 
		_bstr_t   bstrDescription(e.Description()); 
		CString str1, str2;
		str1.Format(TEXT("%s"), (LPCTSTR)bstrSource);
		str2.Format(TEXT("%s"), (LPCTSTR)bstrDescription);

		CString strErrMsg;
		strErrMsg.Format(TEXT("ִ�в�ѯʧ��!��ϸ��Ϣ: %s"), str2);
		AfxMessageBox(strErrMsg);
		assert(false);

		return 0;
	}
}

CDbObject CDbCommand::ExecuteScalar()
{
	assert (m_pCon);
	assert (m_sql.GetLength() > 0);

	CDbObject result;
	try
	{
		COleVariant vaRowsAffected;
		_RecordsetPtr pRecordset = m_cmd->Execute(&vaRowsAffected, NULL, adCmdText);	
		
		if (pRecordset->EndOfFile == VARIANT_FALSE)
		{
			pRecordset->MoveFirst();
			_variant_t vaIndex = (long)0;
			COleVariant vaHolder = pRecordset->GetCollect(vaIndex);
			result.Set(vaHolder);
		}
		pRecordset->Close();

		return result;
	}
	catch (_com_error e)
	{		
		_bstr_t   bstrSource(e.Source()); 
		_bstr_t   bstrDescription(e.Description()); 
		CString str1, str2;
		str1.Format(TEXT("%s"), (LPCTSTR)bstrSource);
		str2.Format(TEXT("%s"), (LPCTSTR)bstrDescription);

		CString strErrMsg;
		strErrMsg.Format(TEXT("ִ�в�ѯʧ��!��ϸ��Ϣ: %s"), str2);
		AfxMessageBox(strErrMsg);
		assert(false);

		return result;
	}
}

CDbDataReader* CDbCommand::ExecuteReader()
{
	assert (m_pCon);
	assert (m_sql.GetLength() > 0);

	try
	{	
		if (m_dataReader.GetRecordsetPtr()->State)
			m_dataReader.GetRecordsetPtr()->Close();		
		
		COleVariant vaRowsAffected;
		// Ϊ�˸�CDbDataReader��m_pRecordset������ֵ��ͬʱ��Ҫ������󿽱�������������ȽϹ���
		m_dataReader.GetRecordsetPtr() = m_cmd->Execute(&vaRowsAffected, NULL, adCmdText);		
		
		return &m_dataReader;
	}
	catch (_com_error e)
	{		
		_bstr_t   bstrSource(e.Source()); 
		_bstr_t   bstrDescription(e.Description()); 
		CString str1, str2;
		str1.Format(TEXT("%s"), (LPCTSTR)bstrSource);
		str2.Format(TEXT("%s"), (LPCTSTR)bstrDescription);

		CString strErrMsg;
		strErrMsg.Format(TEXT("ִ�в�ѯʧ��!��ϸ��Ϣ: %s"), str2);
		AfxMessageBox(strErrMsg);
		assert(false);

		return &m_dataReader;
	}
}

void CDbCommand::SetCommandText( const TCHAR* sql )
{
// 	if (m_sql.GetLength() > 0)
// 	{
// 		AfxMessageBox(TEXT("SetCommandText����ֻ�ܶ�CDbCommand��������״γ�ʼ��, ȷ����Ҫ��CDbCommand����ִ�ж�β�ѯ!"));
// 	}
	m_sql = sql;
	m_cmd->CommandText = m_sql.GetBuffer(m_sql.GetLength() + 1);
	m_sql.ReleaseBuffer();	
	
	m_dataReader.SetFirstLine(true);
}

bool CDbCommand::DeleteFromTable( const TCHAR* tableName )
{
	assert (m_pCon);

	try
	{		
		CString sql;
		sql.Format(TEXT("Delete From %s"), tableName);
		m_cmd->CommandText = sql.GetBuffer(sql.GetLength() + 1);
		sql.ReleaseBuffer();
		COleVariant vaRowsAffected;
		m_cmd->Execute(&vaRowsAffected, NULL, adCmdText);

		return true;
	}
	catch (_com_error e)
	{
		CString errorMsg = TEXT("CDbCommand::DeleteFromTable������, ɾ��������ʧ��!��ϸ��Ϣ: ");
		errorMsg += e.ErrorMessage();
		//AfxMessageBox(strErrMsg);
		throw;		// �����׳��쳣����������һ�㴦��

		return false;
	}
}

/*void CDbCommand::AddParameter( const char* name, DataTypeEnum type, long size, long value )
{
	// �ظ������ж�
	try
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	catch (...)		// ���ظ�����
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, type, adParamInput, size, value);
		m_cmd->Parameters->Append(para);
	}
}

void CDbCommand::AddParameter( const char* name, DataTypeEnum type, long size, const char* value )
{
	// �ظ������ж�
	try
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	catch (...)		// ���ظ�����
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, type, adParamInput, size, value);
		m_cmd->Parameters->Append(para);
	}
}*/

void CDbCommand::Init()
{
	m_cmd.CreateInstance(__uuidof(Command));
	m_cmd->ActiveConnection = m_pCon->GetConnectionPtr();
	m_cmd->CommandText = m_sql.GetBuffer(m_sql.GetLength() + 1);
	m_sql.ReleaseBuffer();	
}

void CDbCommand::AddCharParameter( const TCHAR *name, const TCHAR *value, long size /*= 50*/ )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, adChar, adParamInput, size, value);
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

void CDbCommand::AddByteParameter( const TCHAR *name, byte value )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, adTinyInt, adParamInput, 1, value);
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

void CDbCommand::AddIntParameter( const TCHAR *name, int value, bool bZeroAsNull )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue((long)value);
	}
	else
	{
		int size = 2;
		if (!m_pCon->m_bAccessConnection)
		{
			size = 4;
		}
		_ParameterPtr para;
		if (bZeroAsNull && value == 0)
		{
			_variant_t vNULL;
			vNULL.vt = VT_NULL;
			para = m_cmd->CreateParameter(_bstr_t(name), adInteger, adParamInput, size, vNULL);			
		}
		else
		{
			para = m_cmd->CreateParameter(_bstr_t(name), adInteger, adParamInput, size, (long)value);
		}
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}	
}

void CDbCommand::AddDoubleParameter( const TCHAR *name, double value )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, adDouble, adParamInput, 8, value);
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

void CDbCommand::AddLongParameter( const TCHAR *name, long value )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		int size = 4;
		if (!m_pCon->m_bAccessConnection)
		{
			size = 8;
		}
		_ParameterPtr para;
		if (value > 0)
		{
			para = m_cmd->CreateParameter(_bstr_t(name), adBigInt, adParamInput, size, value);
		}
		else
		{
			_variant_t vNULL;
			vNULL.vt = VT_NULL;
			para = m_cmd->CreateParameter(_bstr_t(name), adBigInt, adParamInput, size, vNULL);
		}
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

void CDbCommand::DeleteParameter( const TCHAR *name )
{
	try
	{
		m_cmd->Parameters->Delete(name);
		if (m_paramSet.count(name) > 0)
		{
			m_paramSet.erase(name);
		}
	}
	catch (...)
	{
		
	}
}

void CDbCommand::AddBoolParameter( const TCHAR* name, bool value )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		_ParameterPtr para = m_cmd->CreateParameter(name, adBoolean, adParamInput, 1, value);
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

void CDbCommand::AddVarcharParameter( const TCHAR *name, const TCHAR* value, long size /*= 50*/ )
{
	// �ظ������ж�
	if (m_paramSet.count(name) > 0)
	{
		_ParameterPtr para = m_cmd->Parameters->GetItem(name);
		para->PutValue(value);
	}
	else
	{
		//_ParameterPtr para = m_cmd->CreateParameter(name, adChar, adParamInput, size, value);
		_ParameterPtr para = m_cmd->CreateParameter(name, adVarChar, adParamInput, size, value);
		m_cmd->Parameters->Append(para);
		m_paramSet.insert(name);
	}
}

bool CDbCommand::UpdateBinaryField( const TCHAR* tableName, const TCHAR* pkName, const TCHAR* fieldName, 
								   int pkId, const TCHAR* fileName )
{
	bool bOk = false;
	if (_taccess(fileName, 0) != -1)
	{
		// ��ȡ�ļ�����ȫ������
		CFileException eFile;
		CFile file;
		file.Open(fileName, CFile::modeRead | CFile::shareDenyNone | CFile::typeBinary, &eFile);	
		long nLength = file.GetLength(); 	
		BYTE* pbuf = new BYTE[nLength];
		if (pbuf)    	
		{
			file.Read(pbuf, nLength);
			
			SAFEARRAYBOUND rgsabound[1];
			rgsabound[0].lLbound = 0;
			rgsabound[0].cElements = nLength;
			SAFEARRAY* psa = SafeArrayCreate(VT_UI1, 1, rgsabound);
			
			BYTE* pTempBuf = pbuf;
			for (long i = 0; i < nLength; i++)
			{
				SafeArrayPutElement(psa, &i, pTempBuf++);
			}			
			
			VARIANT varBLOB;
			varBLOB.vt = VT_ARRAY | VT_UI1;
			varBLOB.parray = psa;

			// ִ�в�ѯ
			CString sql;
			sql.Format(TEXT("Update %s Set %s = ? Where %s = %d"), tableName, fieldName, pkName, pkId);
			SetCommandText(sql);
			CString strFieldName = TEXT("@");
			strFieldName += fieldName;
			if (m_paramSet.count(strFieldName) > 0)
			{
				_ParameterPtr para = m_cmd->Parameters->GetItem(_bstr_t(strFieldName));
				para->PutValue(varBLOB);
			}
			else
			{
				_ParameterPtr para = m_cmd->CreateParameter(_bstr_t(strFieldName), adVarBinary, adParamInput, -1, varBLOB);		
				m_cmd->Parameters->Append(para);
				m_paramSet.insert(strFieldName);
			}			

			// ִ�в�ѯ
			int rows = ExecuteNonQuery();

			//SafeArrayDestroy(psa);
			delete pbuf;

			bOk = rows > 0;
		}

		file.Close();
	}

	return bOk;
}

bool CDbCommand::GetBinaryField( const TCHAR* tableName, const TCHAR* pkName, const TCHAR* fieldName, 
								int pkId, const TCHAR* fileName )
{
	bool bOk = false;
	CString sql;
	sql.Format(TEXT("Select %s From %s Where %s = %d"), fieldName, tableName, pkName, pkId);
	SetCommandText(sql);

	COleVariant vaRowsAffected;
	// Ϊ�˸�CDbDataReader��m_pRecordset������ֵ��ͬʱ��Ҫ������󿽱�������������ȽϹ���
	m_dataReader.GetRecordsetPtr() = m_cmd->Execute(&vaRowsAffected, NULL, adCmdText);
	// ��ȡ�������ļ�
	long  lDataLength = m_dataReader.GetRecordsetPtr()->GetFields()->GetItem(_variant_t(fieldName))->ActualSize;	
	if (lDataLength > 0)		
	{		
		_variant_t varBLOB = m_dataReader.GetRecordsetPtr()->GetFields()->GetItem(_variant_t(fieldName))->GetChunk(lDataLength); 
		
		if (varBLOB.vt== (VT_ARRAY | VT_UI1) && varBLOB.vt != VT_EMPTY && varBLOB.vt != VT_NULL)			
		{	
			BYTE *pBuf = NULL;
			SafeArrayAccessData(varBLOB.parray, (void**)&pBuf);
			// �洢�������ļ�
			CString strFileName = fileName;
			CFile outFile(strFileName, CFile::modeCreate | CFile::modeWrite);	// �������ļ�������ļ����ڣ��򳤶ȱ�Ϊ0 		
			outFile.Write(pBuf, lDataLength);
			outFile.Close();
			SafeArrayUnaccessData(varBLOB.parray);
			
			bOk = true;
		}
	}
	m_dataReader.GetRecordsetPtr()->Close();

	return bOk;
}
