
#include <windows.h>
#include <psapi.h>
#include <iostream>
#include <tlhelp32.h>
#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

static ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;
static HANDLE hProcess;

void init(){
	SYSTEM_INFO sysInfo;
	FILETIME ftime, fsys, fuser;

	GetSystemInfo(&sysInfo);
	numProcessors = sysInfo.dwNumberOfProcessors;

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&lastCPU, &ftime, sizeof(FILETIME));

	GetProcessTimes(hProcess, &ftime, &ftime, &fsys, &fuser);
	memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
	memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));
}

double getCurrentValue(){
	FILETIME ftime, fsys, fuser;
	ULARGE_INTEGER now, sys, user;
	double percent;

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&now, &ftime, sizeof(FILETIME));

	GetProcessTimes(hProcess, &ftime, &ftime, &fsys, &fuser);
	memcpy(&sys, &fsys, sizeof(FILETIME));
	memcpy(&user, &fuser, sizeof(FILETIME));
	percent = (sys.QuadPart - lastSysCPU.QuadPart) +
		(user.QuadPart - lastUserCPU.QuadPart);
	percent /= (now.QuadPart - lastCPU.QuadPart);
	percent /= numProcessors;
	lastCPU = now;
	lastUserCPU = user;
	lastSysCPU = sys;

	return percent * 100;
}


int main(int argc, char* argv[])
{
	boost::property_tree::ptree pt;
	boost::property_tree::json_parser::read_json("config.json", pt);
	std::string exe_name = pt.get<std::string>("exe_name", "StarLoc3DGrace.exe");
	int time_elapse = pt.get<int>("time_elapse", 1000);

	std::cout << "exe_name:" << exe_name << std::endl;
	std::cout << "time_elapse:" << time_elapse << " ms" << std::endl;
	std::ofstream record_file("record.csv");
	record_file << "memory(KB) , cpu_usage" << std::endl;
	PROCESSENTRY32 entry;
	entry.dwSize = sizeof(PROCESSENTRY32);
	HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);
	if (Process32First(snapshot, &entry) == TRUE)
	{
		while (Process32Next(snapshot, &entry) == TRUE)
		{
			if (stricmp(entry.szExeFile, exe_name.c_str()) == 0)
			{
				hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, entry.th32ProcessID);
				init();
				do
				{
					PROCESS_MEMORY_COUNTERS_EX pmc;
					GetProcessMemoryInfo(hProcess, (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
					double virtualMemUsedByMe = pmc.PrivateUsage / 1024.0;
					std::cout << "PrivateUsage:" << virtualMemUsedByMe << " KB" << std::endl;
					double tmp_usage = getCurrentValue();
					std::cout << "cpu usage:" << tmp_usage << std::endl;
					record_file << virtualMemUsedByMe << ", " << tmp_usage << std::endl;
					record_file.flush();
					std::this_thread::sleep_for(std::chrono::milliseconds(time_elapse));
				} while (true);
				CloseHandle(hProcess);
			}
		}
	}
	CloseHandle(snapshot);
	std::cout << "No such process is running!!!" << std::endl;
	return 0;
}
