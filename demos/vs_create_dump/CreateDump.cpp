#include "CreateDump.h"

#include <conio.h>
#include  <io.h>
#include <iostream>
#include <windows.h> 
#include <Dbghelp.h>

namespace CreateDump{
std::string dumpSavePath;
std::string dumpName;

LONG WINAPI UnhandledExceptionFilterEx(struct _EXCEPTION_POINTERS* ExceptionInfo)
{
	HANDLE			hDumpFile = INVALID_HANDLE_VALUE;
	TCHAR			szDumpFileName[1024] = { 0 };
	SYSTEMTIME		sysTime;

	// ≈‰÷√dmp¬∑æ∂
	sprintf_s(szDumpFileName, "%s\\", dumpSavePath.c_str());
	if (-1 == _access(szDumpFileName, 0))
	{
		CreateDirectory(szDumpFileName, NULL);
	}

	GetLocalTime(&sysTime);
	sprintf_s(szDumpFileName, "%s%s%d%d%d_%d%d%d.dmp", szDumpFileName, dumpName.c_str(),
		sysTime.wYear, sysTime.wMonth, sysTime.wDay, sysTime.wHour, sysTime.wMinute, sysTime.wSecond);

	hDumpFile = CreateFile(szDumpFileName, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (INVALID_HANDLE_VALUE != hDumpFile)
	{
		// Dump–≈œ¢  
		MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
		dumpInfo.ExceptionPointers = ExceptionInfo;
		dumpInfo.ThreadId = GetCurrentThreadId();
		dumpInfo.ClientPointers = TRUE;

		MiniDumpWriteDump(
			GetCurrentProcess(),
			GetCurrentProcessId(),
			hDumpFile,
			(MINIDUMP_TYPE)(MiniDumpWithDataSegs | MiniDumpWithFullMemory | MiniDumpWithHandleData),
			&dumpInfo, 0, 0);
		CloseHandle(hDumpFile);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}

void configure(std::string dump_save_path, std::string dump_name) {
	dumpSavePath = dump_save_path;
	dumpName = dump_name;

	SetUnhandledExceptionFilter(UnhandledExceptionFilterEx);
}
}