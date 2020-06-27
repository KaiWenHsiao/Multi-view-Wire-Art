#pragma once
/************************************************************************/
/*
Author:	Lu Liu
Date:	12/17/09

Some frequently used functions
*/
/************************************************************************/
#include <algorithm>
using namespace std;

namespace MyAlgorithmNSpace
{
	//get the most frequent value T in the range [begin, end), the passed in var is changed
	template <typename RandomAccessIterator, typename T>
	void max_frequence(RandomAccessIterator begin, RandomAccessIterator end, T& result)
	{
		//sort
		sort(begin, end);

		//go through the items, count the ones that have the same value
		T mostFreqVal = *begin;
		int maxCount = 0;
		T curVal = *begin;
		int curCount = 1;
		RandomAccessIterator curIter = begin;
		curIter++;
		while (curIter != end)
		{
			if (*curIter != curVal)	//they are different
			{
				if (curCount > maxCount)
				{
					maxCount = curCount;
					mostFreqVal = curVal;
				}

				curCount = 1;
				curVal = *curIter;
			}
			else
			{
				curCount++;
			}

			curIter++;
		}
		result = mostFreqVal;
	}

	//get the most frequent value T in the range [begin, end), the passed in var is changed
	template <typename RandomAccessIterator, typename T>
	void mean(RandomAccessIterator begin, RandomAccessIterator end, T& result)
	{
		result = 0;
		int count = 0;
		while (begin != end)
		{
			result += *begin++;
			count++;
		}
		result /= count;
	}
}