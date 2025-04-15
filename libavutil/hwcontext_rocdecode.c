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


#include "buffer.h"
#include "common.h"
#include "hwcontext.h"
#include "hwcontext_internal.h"
#include "hwcontext_rocdecode.h"
#include "rocdecode_check.h"
#include "mem.h"
#include "pixdesc.h"
#include "pixfmt.h"
#include "imgutils.h"
#include <stdio.h>

typedef struct RocDecodeFramesContext {
    int shift_width, shift_height;
    // TODO: check if needed
    int tex_alignment;
} RocDecodeFramesContext;

typedef struct RocDecodeDeviceContext {
    AVRocDecodeDeviceContext p;
    AVRocDecodeDeviceContextInternal internal;
} RocDecodeDeviceContext;

static const enum AVPixelFormat supported_formats[] = {
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_NV16,
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_P016,
    AV_PIX_FMT_P216LE,
    AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_0RGB32,
    AV_PIX_FMT_0BGR32,
    AV_PIX_FMT_RGB32,
    AV_PIX_FMT_BGR32,
};

#define CHECK_HIP_CALL(x) FF_HIP_CHECK(device_ctx, x)

static int rocdecode_frames_get_constraints(AVHWDeviceContext *ctx,
                                    const void *hwconfig,
                                    AVHWFramesConstraints *constraints) {
    constraints->valid_sw_formats = av_malloc_array(FF_ARRAY_ELEMS(supported_formats) + 1,
                                                    sizeof(*constraints->valid_sw_formats));
    if (!constraints->valid_sw_formats)
        return AVERROR(ENOMEM);

    for (int i = 0; i < FF_ARRAY_ELEMS(supported_formats); i++)
        constraints->valid_sw_formats[i] = supported_formats[i];
    constraints->valid_sw_formats[FF_ARRAY_ELEMS(supported_formats)] = AV_PIX_FMT_NONE;

    constraints->valid_hw_formats = av_malloc_array(2, sizeof(*constraints->valid_hw_formats));
    if (!constraints->valid_hw_formats)
        return AVERROR(ENOMEM);

    constraints->valid_hw_formats[0] = AV_PIX_FMT_AMD_GPU;
    constraints->valid_hw_formats[1] = AV_PIX_FMT_NONE;

    return 0;
}

static void rocdecode_buffer_free(void *opaque, uint8_t *data) {
    AVHWFramesContext        *ctx = opaque;
    AVHWDeviceContext *device_ctx = ctx->device_ctx;

    CHECK_HIP_CALL(hipFree(data));
}

static AVBufferRef *rocdecode_pool_alloc(void *opaque, size_t size)
{
    AVHWFramesContext        *ctx = opaque;
    AVHWDeviceContext *device_ctx = ctx->device_ctx;

    AVBufferRef *ret = NULL;
    void* data;

    CHECK_HIP_CALL(hipMalloc(&data, size));

    ret = av_buffer_create((uint8_t*)data, size, rocdecode_buffer_free, ctx, 0);
    if (!ret)
        CHECK_HIP_CALL(hipFree(data));

    return ret;
}

static int rocdecode_frames_init(AVHWFramesContext *ctx)
{
    AVHWDeviceContext *device_ctx = ctx->device_ctx;
    AVRocDecodeDeviceContext    *hwctx = device_ctx->hwctx;
    RocDecodeFramesContext       *priv = ctx->hwctx;
    int err, i;

    for (i = 0; i < FF_ARRAY_ELEMS(supported_formats); i++) {
        if (ctx->sw_format == supported_formats[i])
            break;
    }
    if (i == FF_ARRAY_ELEMS(supported_formats)) {
        av_log(ctx, AV_LOG_ERROR, "Pixel format '%s' is not supported\n",
            av_get_pix_fmt_name(ctx->sw_format));
        return AVERROR(ENOSYS);
    }

    err = CHECK_HIP_CALL(hipDeviceGetAttribute(&priv->tex_alignment,
                                            hipDeviceAttributeTextureAlignment ,
                                            hwctx->internal->device));
    if (err < 0)
        return err;

    av_log(ctx, AV_LOG_DEBUG, "HIP texture alignment: %d\n", priv->tex_alignment);

    av_pix_fmt_get_chroma_sub_sample(ctx->sw_format, &priv->shift_width, &priv->shift_height);

    if (!ctx->pool) {
        int size = av_image_get_buffer_size(ctx->sw_format, ctx->width, ctx->height, priv->tex_alignment);
        if (size < 0)
            return size;

        ffhwframesctx(ctx)->pool_internal =
            av_buffer_pool_init2(size, ctx, rocdecode_pool_alloc, NULL);
        if (!ffhwframesctx(ctx)->pool_internal)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static int rocdecode_get_buffer(AVHWFramesContext *ctx, AVFrame *frame)
{
    RocDecodeFramesContext *priv = ctx->hwctx;
    int res;

    frame->buf[0] = av_buffer_pool_get(ctx->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    res = av_image_fill_arrays(frame->data, frame->linesize, frame->buf[0]->data,
                            ctx->sw_format, ctx->width, ctx->height, priv->tex_alignment);
    if (res < 0)
        return res;

    // YUV420P is a special case.
    // Nvenc expects the U/V planes in swapped order from how ffmpeg expects them, also chroma is half-aligned
    if (ctx->sw_format == AV_PIX_FMT_YUV420P) {
        frame->linesize[1] = frame->linesize[2] = frame->linesize[0] / 2;
        frame->data[2]     = frame->data[1];
        frame->data[1]     = frame->data[2] + frame->linesize[2] * (ctx->height / 2);
    }

    frame->format = AV_PIX_FMT_CUDA;
    frame->width  = ctx->width;
    frame->height = ctx->height;

    return 0;
}

static int rocdecode_transfer_get_formats(AVHWFramesContext *ctx,
                                    enum AVHWFrameTransferDirection dir,
                                    enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;

    fmts = av_malloc_array(2, sizeof(*fmts));
    if (!fmts)
        return AVERROR(ENOMEM);

    fmts[0] = ctx->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    *formats = fmts;

    return 0;
}

// TODO: port to rocm
static int rocdecode_transfer_data(AVHWFramesContext *ctx, AVFrame *dst,
                                const AVFrame *src)
{
    /*RocDecodeFramesContext       *priv = ctx->hwctx;
    AVHWDeviceContext *device_ctx = ctx->device_ctx;
    AVRocDecodeDeviceContext    *hwctx = device_ctx->hwctx;

    int i, ret;

    if ((src->hw_frames_ctx && ((AVHWFramesContext*)src->hw_frames_ctx->data)->format != AV_PIX_FMT_AMD_GPU) ||
        (dst->hw_frames_ctx && ((AVHWFramesContext*)dst->hw_frames_ctx->data)->format != AV_PIX_FMT_AMD_GPU))
        return AVERROR(ENOSYS);

    for (i = 0; i < FF_ARRAY_ELEMS(src->data) && src->data[i]; i++) {
        CUDA_MEMCPY2D cpy = {
            .srcPitch      = src->linesize[i],
            .dstPitch      = dst->linesize[i],
            .WidthInBytes  = FFMIN(src->linesize[i], dst->linesize[i]),
            .Height        = src->height >> ((i == 0 || i == 3) ? 0 : priv->shift_height),
        };

        if (src->hw_frames_ctx) {
            cpy.srcMemoryType = CU_MEMORYTYPE_DEVICE;
            cpy.srcDevice     = (CUdeviceptr)src->data[i];
        } else {
            cpy.srcMemoryType = CU_MEMORYTYPE_HOST;
            cpy.srcHost       = src->data[i];
        }

        if (dst->hw_frames_ctx) {
            cpy.dstMemoryType = CU_MEMORYTYPE_DEVICE;
            cpy.dstDevice     = (CUdeviceptr)dst->data[i];
        } else {
            cpy.dstMemoryType = CU_MEMORYTYPE_HOST;
            cpy.dstHost       = dst->data[i];
        }

        ret = CHECK_CU(cu->cuMemcpy2DAsync(&cpy, hwctx->stream));
        if (ret < 0)
            goto exit;
    }

    if (!dst->hw_frames_ctx) {
        ret = CHECK_CU(cu->cuStreamSynchronize(hwctx->stream));
        if (ret < 0)
            goto exit;
    }
    */
    return 0;
}


static void rocdecode_device_uninit(AVHWDeviceContext *device_ctx)
{
    RocDecodeDeviceContext *hwctx = device_ctx->hwctx;

    if (hwctx->p.internal) {
        memset(&hwctx->p, 0, sizeof(hwctx->p));
        hwctx->p.internal = NULL;
    }
    av_log(hwctx, AV_LOG_VERBOSE, "rocdecode_device_uninit: Uninitialize device successful\n");
}

static int rocdecode_device_init(AVHWDeviceContext *ctx)
{
    RocDecodeDeviceContext *hwctx = ctx->hwctx;

    hwctx->p.internal = &hwctx->internal;

    if (!hwctx) {
        av_log(ctx, AV_LOG_ERROR, "rocdecode_device_init: Could not initialize device\n");
        return -1;
    }
    av_log(ctx, AV_LOG_VERBOSE, "rocdecode_device_init: Initialize device successful\n");
    return 0;
}

static int rocdecode_device_create(AVHWDeviceContext *device_ctx,
                                    const char *device,
                                    AVDictionary *opts, int flags)
{
    AVRocDecodeDeviceContext *hwctx = device_ctx->hwctx;
    int ret, device_idx = 0;

    if (device)
        device_idx = strtol(device, NULL, 0);

    ret = rocdecode_device_init(device_ctx);
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }

    ret = CHECK_HIP_CALL(hipInit(device_idx));
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }

    ret = CHECK_HIP_CALL(hipDeviceGet(&hwctx->internal->device, device_idx));
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }
    av_log(hwctx, AV_LOG_VERBOSE, "rocdecode_device_create: Created device on %d\n", device_idx);
    return 0;
}

static int rocdecode_device_derive(AVHWDeviceContext *device_ctx,
                            AVHWDeviceContext *src_ctx, AVDictionary *opts,
                            int flags) {
    AVRocDecodeDeviceContext *hwctx = device_ctx->hwctx;
    const char *src_uuid = NULL;
    int ret, i, device_count;

    ret = rocdecode_device_init(device_ctx);
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }

    ret = CHECK_HIP_CALL(hipInit(hwctx->internal->device));
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }

    ret = CHECK_HIP_CALL(hipGetDeviceCount(&device_count));
    if (ret < 0) {
        rocdecode_device_uninit(device_ctx);
        return ret;
    }

    hwctx->internal->device = -1;
    for (i = 0; i < device_count; i++) {
        hipDevice_t dev;
        hipUUID uuid;

        ret = CHECK_HIP_CALL(hipDeviceGet(&dev, i));
        if (ret < 0) {
            rocdecode_device_uninit(device_ctx);
            return ret;
        }

        ret = CHECK_HIP_CALL(hipDeviceGetUuid(&uuid, dev));
        if (ret < 0) {
            rocdecode_device_uninit(device_ctx);
            return ret;
        }

        if (memcmp(src_uuid, uuid.bytes, sizeof (uuid.bytes)) == 0) {
            hwctx->internal->device = dev;
            break;
        }
    }

    if (hwctx->internal->device == -1) {
        av_log(device_ctx, AV_LOG_ERROR, "Could not derive HIP device.\n");
        rocdecode_device_uninit(device_ctx);
        return -1;
    }
    av_log(hwctx, AV_LOG_VERBOSE, "rocdecode_device_derive: Derived device on %s\n", src_uuid);
    return 0;
}

const HWContextType ff_hwcontext_type_amd_gpu = {
    .type                   = AV_HWDEVICE_TYPE_AMD_GPU,
    .name                   = "ROCDECODE",

    .device_hwctx_size      = sizeof(RocDecodeDeviceContext),
    .frames_hwctx_size      = sizeof(RocDecodeFramesContext),

    .device_create          = rocdecode_device_create,
    .device_derive          = rocdecode_device_derive,
    .device_init            = rocdecode_device_init,
    .device_uninit          = rocdecode_device_uninit,
    .frames_get_constraints = rocdecode_frames_get_constraints,
    .frames_init            = rocdecode_frames_init,
    .frames_get_buffer      = rocdecode_get_buffer,
    .transfer_get_formats   = rocdecode_transfer_get_formats,
    .transfer_data_to       = rocdecode_transfer_data,
    .transfer_data_from     = rocdecode_transfer_data,

    .pix_fmts               = (const enum AVPixelFormat[]){ AV_PIX_FMT_AMD_GPU, AV_PIX_FMT_NONE },
};
