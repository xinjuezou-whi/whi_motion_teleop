/******************************************************************
CUtility class for general function

Features:
- Format SQL query command
- Format date
- Format GET protocol
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2018-08-20: Initial version
2018-08-xx: xxx
******************************************************************/

#pragma once
#include <string>
#include <vector>
using namespace std;

class CUtility
{
protected:
	CUtility();
	~CUtility();

public:
	static string formatDate(int Year, int Month, int Day);
	static bool readCsv(const char* FileName, vector<vector<string> >& ContentArr);
	static void recordCurDate();
	static bool isDayCrossed(int& CurHour, int& CurMin);

public:
	static const char* ASC_DATE_FORMAT; // Mon Feb 16 11:29:26 2009
	static const char* FILE_DATE_FORMAT; // 2018.09.01

public:
	static int curYear;
	static int curMonth;
	static int curDay;
	static string curDate;
};

