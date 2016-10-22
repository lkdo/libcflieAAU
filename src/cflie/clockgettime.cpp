// Copyright (c) 2016, Luminita C. Totu <lct@es.aau.dk>, Aalborg University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Luminita C. Totu, Aalborg University */

#include "cflie/clockgettime.h"

#ifdef _WIN32
	LARGE_INTEGER
	getFILETIMEoffset()
	{
		SYSTEMTIME s;
		FILETIME f;
		LARGE_INTEGER t;

		s.wYear = 1970;
		s.wMonth = 1;
		s.wDay = 1;
		s.wHour = 0;
		s.wMinute = 0;
		s.wSecond = 0;
		s.wMilliseconds = 0;
		SystemTimeToFileTime(&s, &f);
		t.QuadPart = f.dwHighDateTime;
		t.QuadPart <<= 32;
		t.QuadPart |= f.dwLowDateTime;
		return (t);
	}

	int
	win_clockgettime(int X, struct timeval *tv)
	{
		LARGE_INTEGER           t;
		FILETIME            f;
		double                  microseconds;
		static LARGE_INTEGER    offset;
		static double           frequencyToMicroseconds;
		static int              initialized = 0;
		static BOOL             usePerformanceCounter = 0;

		if (!initialized) {
			LARGE_INTEGER performanceFrequency;
			initialized = 1;
			usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
			if (usePerformanceCounter) {
				QueryPerformanceCounter(&offset);
				frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
			} else {
				offset = getFILETIMEoffset();
				frequencyToMicroseconds = 10.;
			}
		}
		if (usePerformanceCounter) QueryPerformanceCounter(&t);
		else {
			GetSystemTimeAsFileTime(&f);
			t.QuadPart = f.dwHighDateTime;
			t.QuadPart <<= 32;
			t.QuadPart |= f.dwLowDateTime;
		}

		t.QuadPart -= offset.QuadPart;
		microseconds = (double)t.QuadPart / frequencyToMicroseconds;
		t.QuadPart = microseconds;
		tv->tv_sec = t.QuadPart / 1000000;
		tv->tv_usec = t.QuadPart % 1000000;
		return (0);
	}

	double currentTime() 
	{
		struct timeval tvTime;
		win_clockgettime(0, &tvTime);
		return tvTime.tv_sec + double(tvTime.tv_usec) * 1000 / NSEC_PER_SEC;
	}

#else
	
	double currentTime() {
	  struct timespec tsTime;
	  clock_gettime(CLOCK_MONOTONIC, &tsTime);
	  return tsTime.tv_sec + double(tsTime.tv_nsec) / NSEC_PER_SEC;
	}

#endif
