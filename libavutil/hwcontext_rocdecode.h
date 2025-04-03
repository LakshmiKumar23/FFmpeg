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


#ifndef AVUTIL_HWCONTEXT_ROCDECODE_H
#define AVUTIL_HWCONTEXT_ROCDECODE_H


#include <hip/hip_runtime.h>
#include "pixfmt.h"

/**
 * @file
 * An API-specific header for AV_HWDEVICE_TYPE_ROCDECODE.
 *
 * This API supports dynamic frame pools. AVHWFramesContext.pool must return
 * AVBufferRefs whose data pointer is a CUdeviceptr.
 */

typedef struct AVRocDecodeDeviceContextInternal AVRocDecodeDeviceContextInternal;

struct AVRocDecodeDeviceContextInternal {
    int flags;
    int is_allocated;
    hipDevice_t device;
};


/**
 * @file
 * An API-specific header for AV_HWDEVICE_TYPE_ROCDECODE.
 *
 */

/**
 * This struct is allocated as AVHWDeviceContext.hwctx
 */
typedef struct AVRocDecodeDeviceContext {
    hipStream_t stream;
    AVRocDecodeDeviceContextInternal *internal;

} AVRocDecodeDeviceContext;

#endif /* AVUTIL_HWCONTEXT_ROCDECODE_H */