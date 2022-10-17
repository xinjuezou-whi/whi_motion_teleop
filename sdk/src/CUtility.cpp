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
******************************************************************/

#include "../include/CUtility.h"
#include <fstream>
#include <string.h>


const char* CUtility::ASC_DATE_FORMAT = "%a %B %d %H:%M:%S %Y"; // Mon Feb 16 11:29:26 2009
const char* CUtility::FILE_DATE_FORMAT = "%Y.%m.%d"; // 2018.09.01
int CUtility::curYear;
int CUtility::curMonth;
int CUtility::curDay;
string CUtility::curDate;
CUtility::CUtility()
{
}

CUtility::~CUtility()
{
}

string CUtility::formatDate(int Year, int Month, int Day)
{
	string strDate(to_string(Year) + ".");
	if (Month < 10)
	{
		strDate.append("0");
	}
	strDate.append(to_string(Month) + ".");
	if (Day < 10)
	{
		strDate.append("0");
	}
	strDate.append(to_string(Day));

	return strDate;
}

bool CUtility::readCsv(const char* FileName, vector<vector<string> >& ContentArr)
{
	ContentArr.clear();

	ifstream ifs(FileName, ios::in);
	if (!ifs.good())
	{
		return false;
	}

	static const size_t BUF_LEN = 512;
	char lineBuf[BUF_LEN] = { 0 };

	while (!ifs.eof())
	{
		ifs.getline(lineBuf, BUF_LEN, '\n');
		if (strlen(lineBuf) > 0 && lineBuf[0] != '#')
		{
			vector<string> rowContents;

			const char* first = lineBuf;
			const char* delimComma = strchr(lineBuf, ',');
			while (delimComma != NULL)
			{
				rowContents.push_back(string(first, delimComma));

				first = delimComma + 1;
				delimComma = strchr(first, ',');
			}

			rowContents.push_back(string(first));

			// there is \r at the end of line, if such file is editted by windows
			if ((rowContents.end() - 1)->find('\r') != string::npos)
			{
				(rowContents.end() - 1)->pop_back();
			}

			size_t contentSize = 0;
			for (vector<string>::const_iterator iter = rowContents.begin(); iter != rowContents.end(); ++iter)
			{
				contentSize += iter->length();
			}
			if (contentSize > 0)
			{
				ContentArr.push_back(rowContents);
			}
		}
	}
	ifs.close();

	return true;
}

void CUtility::recordCurDate()
{
	time_t tNow = time(NULL);
	struct tm* tmNow = localtime(&tNow);

	curYear = tmNow->tm_year + 1900;
	curMonth = tmNow->tm_mon + 1;
	curDay = tmNow->tm_mday;
	curDate.assign(formatDate(curYear, curMonth, curDay));
}

bool CUtility::isDayCrossed(int& CurHour, int& CurMin)
{
	time_t tNow = time(NULL);
	struct tm* timeStruct = localtime(&tNow);
	CurHour = timeStruct->tm_hour;
	CurMin = timeStruct->tm_min;

	if (curYear < timeStruct->tm_year + 1900 ||
		curMonth < timeStruct->tm_mon + 1 ||
		curDay < timeStruct->tm_mday)
	{
		curYear = timeStruct->tm_year + 1900;
		curMonth = timeStruct->tm_mon + 1;
		curDay = timeStruct->tm_mday;
		curDate = formatDate(curYear, curMonth, curDay);

		return true;
	}

	return false;
}
