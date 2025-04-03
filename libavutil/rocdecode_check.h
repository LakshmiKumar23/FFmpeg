/*
* This file is part of FFmpeg.
*
* FFmpeg is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* FFmpeg is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with FFmpeg; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef AVUTIL_ROCDECODE_CHECK_H
#define AVUTIL_ROCDECODE_CHECK_H

#include "error.h"
#include "libavutil/log.h"
#include <hip/hip_runtime.h>
#include "rocdecode/rocdecode.h"

typedef hipError_t hip_check_GetErrorName(hipError_t error, const char** pstr);
typedef hipError_t hip_check_GetErrorString(hipError_t error, const char** pstr);
typedef hipError_t rocdecode_check_GetErrorName(rocDecStatus error, const char** pstr);

/**
 * Wrap a HIP function call and print error information if it fails.
 */
static inline int ff_hip_check(void *avctx, void *hipGetErrorName_fn, void *hipGetErrorString_fn,
    hipError_t err, const char *func) {

    const char *err_name = NULL;
    const char *err_string = NULL;
    av_log(avctx, AV_LOG_TRACE, "Calling %s\n", func);

    if (err == hipSuccess)
        return 0;

    ((hip_check_GetErrorName *)hipGetErrorName_fn)(err, &err_name);
    ((hip_check_GetErrorString *)hipGetErrorString_fn)(err, &err_string);
    av_log(avctx, AV_LOG_ERROR, "%s failed", func);
    if (err_name && err_string)
        av_log(avctx, AV_LOG_ERROR, " -> %s: %s", err_name, err_string);
    av_log(avctx, AV_LOG_ERROR, "\n");

    return AVERROR_EXTERNAL;
}

/**
 * Wrap a rocDecode function call and print error information if it fails.
 */
static inline int ff_rocdecode_check(void *rocDecGetErrorName_fn, rocDecStatus err, const char *func) {

    void *avctx = NULL;
    const char *err_name = NULL;
    av_log(avctx, AV_LOG_TRACE, "Calling %s\n", func);

    if (err == ROCDEC_SUCCESS)
        return 0;

    ((rocdecode_check_GetErrorName *)rocDecGetErrorName_fn)(err, &err_name);
    av_log(avctx, AV_LOG_ERROR, "%s failed", func);
    if (err_name)
        av_log(avctx, AV_LOG_ERROR, " -> %s", err_name);
    av_log(avctx, AV_LOG_ERROR, "\n");

    return AVERROR_EXTERNAL;
}


#define FF_HIP_CHECK(avclass, x) ff_hip_check(avclass, hipGetErrorName, hipGetErrorString, (x), #x)
#define FF_ROCDECODE_CHECK(x) ff_rocdecode_check(rocDecGetErrorName, (x), #x)

#endif /* AVUTIL_ROCDECODE_CHECK_H */