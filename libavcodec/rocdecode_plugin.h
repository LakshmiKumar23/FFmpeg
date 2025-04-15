/*
* HW decode acceleration through AMD's rocDecode
*
* Copyright (c) 2025 AMD
*
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

#ifndef AVCODEC_ROCDEC_H
#define AVCODEC_ROCDEC_H

#include "rocdecode/rocdecode.h"

#include <stdint.h>

#include "libavutil/buffer.h"
#include "libavutil/frame.h"

#include "avcodec.h"

typedef struct RocDecFrame {
    unsigned int idx;
    unsigned int ref_idx;
    unsigned int *idx_ref;         ///< RefStruct reference
    unsigned int *ref_idx_ref;     ///< RefStruct reference
    struct RocDecDecoder  *decoder; ///< RefStruct reference
} RocDecFrame;

typedef struct RocDecContext {
    RocdecPicParams pic_params;

    struct AVRefStructPool *decoder_pool;

    struct RocDecDecoder  *decoder; ///< RefStruct reference

    const uint8_t *bitstream;
    int           bitstream_len;
    unsigned int  bitstream_allocated;
    uint8_t      *bitstream_internal;

    unsigned     *slice_offsets;
    int           nb_slices;
    unsigned int  slice_offsets_allocated;

    int           supports_444;
} RocDecContext;

int ff_amd_gpu_decode_init(AVCodecContext *avctx);
int ff_amd_gpu_decode_uninit(AVCodecContext *avctx);
int ff_amd_gpu_start_frame(AVCodecContext *avctx, AVFrame *frame);
int ff_amd_gpu_start_frame_sep_ref(AVCodecContext *avctx, AVFrame *frame, int has_sep_ref);
int ff_amd_gpu_end_frame(AVCodecContext *avctx);
int ff_amd_gpu_simple_end_frame(AVCodecContext *avctx);
int ff_amd_gpu_simple_decode_slice(AVCodecContext *avctx, const uint8_t *buffer,
                                uint32_t size);
int ff_amd_gpu_frame_params(AVCodecContext *avctx,
                        AVBufferRef *hw_frames_ctx,
                        int dpb_size,
                        int supports_444);
int ff_amd_gpu_get_ref_idx(AVFrame *frame);

#endif /* AVCODEC_ROCDEC_H */
