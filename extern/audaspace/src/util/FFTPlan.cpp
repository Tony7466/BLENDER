/*******************************************************************************
* Copyright 2015-2016 Juan Francisco Crespo Galán
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

#include "util/FFTPlan.h"

AUD_NAMESPACE_BEGIN
FFTPlan::FFTPlan(double measureTime) :
	FFTPlan(DEFAULT_N, measureTime)
{
}

//	internal_buffer = fftw_malloc(sizeof(fftw_complex)*m_N);  // I think that with m_N should be enough
#ifdef AUDASPACE_FFTW

FFTPlan::FFTPlan(int n, double measureTime) :
	m_N(n), m_bufferSize((( n / 2) + 1) * 2 * sizeof(fftw_complex))
{
	fftw_set_timelimit(measureTime);
	void* buf = fftw_malloc(sizeof(fftw_complex)*m_bufferSize);
	m_fftPlanR2C = fftw_plan_dft_r2c_1d(m_N, (fft_real *)buf, (fftw_complex *)buf, FFTW_EXHAUSTIVE);
	m_fftPlanC2R = fftw_plan_dft_c2r_1d(m_N, (fftw_complex *)buf, (fft_real *)buf, FFTW_EXHAUSTIVE);
	fftw_free(buf);
}

#else

FFTPlan::FFTPlan(int n, double measureTime) :
	m_N(n), m_bufferSize((( n / 2) + 1) * 2 * sizeof(fftwf_complex))
{
	fftwf_set_timelimit(measureTime);
	void* buf = fftw_malloc(sizeof(fftwf_complex) * m_bufferSize);
	m_fftPlanR2C = fftwf_plan_dft_r2c_1d(m_N, (fft_real *)buf, (fftwf_complex *)buf, FFTW_EXHAUSTIVE);
	m_fftPlanC2R = fftwf_plan_dft_c2r_1d(m_N, (fftwf_complex *)buf, (fft_real *)buf, FFTW_EXHAUSTIVE);
	fftw_free(buf);
}
#endif

FFTPlan::~FFTPlan()
{
#ifdef AUDASPACE_FFTW
        fftw_destroy_plan(m_fftPlanC2R);
        fftw_destroy_plan(m_fftPlanR2C);
#else
        fftwf_destroy_plan(m_fftPlanC2R);
        fftwf_destroy_plan(m_fftPlanR2C);
#endif
}

int FFTPlan::getSize()
{
	return m_N;
}

/*
 * Input is Float / Double .... (sample_t)
 * Output is fftwf_complex / fftw_complex / ...
 */
void FFTPlan::FFT(void* buffer)
{
#ifdef AUDASPACE_FFTW
	void *internal_buffer = &(((fftwf_complex *)buffer)[m_bufferSize]);
	// Translate forward
	sample_t *pt_buffer = (sample_t *)buffer;
	fft_real *internal_pt_buffer = (fft_real *)internal_buffer;

	// Translate from float to wide float
	for(int i = 0; i < m_N; i++)
		//internal_pt_buffer[i]=(fft_real)pt_buffer[i];
		*internal_pt_buffer++ = (fft_real)(*pt_buffer++);

	fftw_execute_dft_r2c(m_fftPlanR2C, (fft_real *)internal_buffer,
	                     (fftw_complex *)internal_buffer);

	// Translate backward
	fftwf_complex *pt_buffer2 = (fftwf_complex *)buffer;
	fftw_complex *internal_pt_buffer2 = (fftw_complex *)internal_buffer;

	for(int i = 0; i < m_N; i++) {
		pt_buffer2[i][0] = (sample_t)internal_pt_buffer2[i][0];
		pt_buffer2[i][1] = (sample_t)internal_pt_buffer2[i][1];
	}

#else
  fftwf_execute_dft_r2c(m_fftPlanR2C, (fft_real *)buffer,
                        (fftwf_complex *)buffer);
#endif
}

/*
 * Input is fftwf_complex / fftw_complex / ...
 * Output is Float / Double .... (sample_t)
 */
void FFTPlan::IFFT(void *buffer)
{
#ifdef AUDASPACE_FFTW
	void *internal_buffer = &(((fftwf_complex *)buffer)[m_bufferSize]);
	// Translate forward
	fftwf_complex *pt_buffer = (fftwf_complex *)buffer;
	fftw_complex *internal_pt_buffer = (fftw_complex *)internal_buffer;
	for(int i = 0; i < m_N; i++) {
		internal_pt_buffer[i][0] = (fft_real)pt_buffer[i][0];
		internal_pt_buffer[i][1] = (fft_real)pt_buffer[i][1];
	}

  fftw_execute_dft_c2r(m_fftPlanC2R, (fftw_complex *)internal_buffer,
                       (fft_real *)internal_buffer);


	sample_t *pt_buffer2 = (sample_t *)buffer;
	fft_real *internal_pt_buffer2 = (fft_real *)internal_buffer;
	for(int i = 0; i < m_N; i++)
		pt_buffer2[i] = (sample_t)internal_pt_buffer2[i];

#else
  fftwf_execute_dft_c2r(m_fftPlanC2R, (fftwf_complex *)buffer,
                        (fft_real *)buffer);
#endif
}

void *FFTPlan::getBuffer()
{
#ifdef AUDASPACE_FFTW
  return fftw_malloc(sizeof(fftwf_complex) * m_bufferSize +
                     sizeof(fftw_complex) * m_bufferSize);
#else
	return fftw_malloc(sizeof(fftwf_complex) * m_bufferSize);
#endif
}

void FFTPlan::freeBuffer(void *buffer)
{
	fftw_free(buffer);
}

AUD_NAMESPACE_END
