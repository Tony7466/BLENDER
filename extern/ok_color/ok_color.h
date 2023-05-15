// Copyright (c) 2021 Björn Ottosson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ok_color_hsv_to_rgb(float h, float s, float v, float *r_r, float *r_g, float *r_b);
void ok_color_hsv_to_rgb_v(const float hsv[3], float r_rgb[3]);

void ok_color_hsl_to_rgb(float h, float s, float l, float *r_r, float *r_g, float *r_b);
void ok_color_hsl_to_rgb_v(const float hsl[3], float r_rgb[3]);

void ok_color_rgb_to_hsv(float r, float g, float b, float *r_h, float *r_s, float *r_v);
void ok_color_rgb_to_hsv_v(const float rgb[3], float r_hsv[3]);

void ok_color_rgb_to_hsl(float r, float g, float b, float *r_h, float *r_s, float *r_l);
void ok_color_rgb_to_hsl_v(const float rgb[3], float r_hsl[3]);

#ifdef __cplusplus
}
#endif
