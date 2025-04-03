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

#include "config.h"
#include "config_components.h"

#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/hwcontext.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"

#include "avcodec.h"
#include "decode.h"
#include "internal.h"
#include "libavutil/refstruct.h"

#include "rocdecode_plugin.h"
#include "libavutil/rocdecode_check.h"
#include "libavutil/hwcontext_rocdecode.h"

#define CHECK_ROCDECODE(x) FF_ROCDECODE_CHECK(x)

typedef struct RocDecDecoder {
    rocDecDecoderHandle decoder;

    AVBufferRef *hw_device_ref;
    AVBufferRef *real_hw_frames_ref;
    hipStream_t stream;

    int unsafe_output;
} RocDecDecoder;

typedef struct rocDecFramePool {
    unsigned int dpb_size;
    unsigned int nb_allocated;
} rocDecFramePool;


static int map_avcodec_id(enum AVCodecID id)
{
    switch (id) {
    case AV_CODEC_ID_AV1:        return rocDecVideoCodec_AV1;
    case AV_CODEC_ID_H264:       return rocDecVideoCodec_AVC;
    case AV_CODEC_ID_HEVC:       return rocDecVideoCodec_HEVC;
    case AV_CODEC_ID_MJPEG:      return rocDecVideoCodec_JPEG;
    case AV_CODEC_ID_MPEG1VIDEO: return rocDecVideoCodec_MPEG1;
    case AV_CODEC_ID_MPEG2VIDEO: return rocDecVideoCodec_MPEG2;
    case AV_CODEC_ID_MPEG4:      return rocDecVideoCodec_MPEG4;
    case AV_CODEC_ID_VP8:        return rocDecVideoCodec_VP8;
    case AV_CODEC_ID_VP9:        return rocDecVideoCodec_VP9;
    }
    return -1;
}

static int map_chroma_format(enum AVPixelFormat pix_fmt)
{
    int shift_h = 0, shift_v = 0;

    if (av_pix_fmt_count_planes(pix_fmt) == 1)
        return rocDecVideoChromaFormat_Monochrome;

    av_pix_fmt_get_chroma_sub_sample(pix_fmt, &shift_h, &shift_v);

    if (shift_h == 1 && shift_v == 1)
        return rocDecVideoChromaFormat_420;
    else if (shift_h == 1 && shift_v == 0)
        return rocDecVideoChromaFormat_422;
    else if (shift_h == 0 && shift_v == 0)
        return rocDecVideoChromaFormat_444;

    return -1;
}

static int rocdec_test_capabilities(RocDecDecoder *decoder,
                                    RocDecoderCreateInfo *params, void *logctx)
{
    int ret;
    RocdecDecodeCaps caps = { 0 };

    caps.codec_type      = params->codec_type;
    caps.chroma_format   = params->chroma_format;
    caps.bit_depth_minus_8 = params->bit_depth_minus_8;

    ret = CHECK_ROCDECODE(rocDecGetDecoderCaps(&caps));
    if (ret < 0)
        return ret;

    av_log(logctx, AV_LOG_VERBOSE, "RocDecode capabilities:\n");
    av_log(logctx, AV_LOG_VERBOSE, "min_width: %d, max_width: %d\n",
           caps.min_width, caps.max_width);
    av_log(logctx, AV_LOG_VERBOSE, "min_height: %d, max_height: %d\n",
           caps.min_height, caps.max_height);

    if (!caps.is_supported) {
        av_log(logctx, AV_LOG_ERROR, "Hardware is lacking required capabilities\n");
        return AVERROR(EINVAL);
    }

    if (params->width > caps.max_height || params->width < caps.min_height) {
        av_log(logctx, AV_LOG_ERROR, "Video width %d not within range from %d to %d\n",
               (int)params->width, caps.min_height, caps.max_height);
        return AVERROR(EINVAL);
    }

    if (params->height > caps.max_height || params->height < caps.min_height) {
        av_log(logctx, AV_LOG_ERROR, "Video height %d not within range from %d to %d\n",
               (int)params->height, caps.min_height, caps.max_height);
        return AVERROR(EINVAL);
    }

    return 0;
}

static void rocdec_decoder_free(AVRefStructOpaque unused, void *obj)
{
    RocDecDecoder *decoder = obj;

    if (decoder->decoder) {
        CHECK_ROCDECODE(rocDecDestroyDecoder(decoder->decoder));
    }

    av_buffer_unref(&decoder->real_hw_frames_ref);
    av_buffer_unref(&decoder->hw_device_ref);
}

static int rocdec_decoder_create(RocDecDecoder **out, AVBufferRef *hw_device_ref,
                                RocDecoderCreateInfo *params, void *logctx)
{
    AVHWDeviceContext  *hw_device_ctx = (AVHWDeviceContext*)hw_device_ref->data;
    AVRocDecodeDeviceContext *device_hwctx = hw_device_ctx->hwctx;

    RocDecDecoder *decoder;

    int ret;

    decoder = av_refstruct_alloc_ext(sizeof(*decoder), 0,
                                     NULL, rocdec_decoder_free);
    if (!decoder)
        return AVERROR(ENOMEM);

    decoder->hw_device_ref = av_buffer_ref(hw_device_ref);
    if (!decoder->hw_device_ref) {
        av_refstruct_unref(&decoder);
        return AVERROR(ENOMEM);
    }
    decoder->stream = device_hwctx->stream;

    ret = rocdec_test_capabilities(decoder, params, logctx);
    if (ret < 0) {
        av_refstruct_unref(&decoder);
        return ret;
    }

    ret = CHECK_ROCDECODE(rocDecCreateDecoder(&decoder->decoder, params));
    if (ret < 0) {
        av_refstruct_unref(&decoder);
        return ret;
    }

    *out = decoder;

    return 0;
}

static int rocdec_decoder_frame_init(AVRefStructOpaque opaque, void *obj)
{
    rocDecFramePool *pool = opaque.nc;
    unsigned int *intp = obj;

    if (pool->nb_allocated >= pool->dpb_size)
        return AVERROR(ENOMEM);

    *intp = pool->nb_allocated++;

    return 0;
}

static void rocdec_decoder_frame_pool_free(AVRefStructOpaque opaque)
{
    av_free(opaque.nc);
}

int ff_rocdec_decode_uninit(AVCodecContext *avctx)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;

    av_freep(&ctx->bitstream_internal);
    ctx->bitstream           = NULL;
    ctx->bitstream_len       = 0;
    ctx->bitstream_allocated = 0;

    av_freep(&ctx->slice_offsets);
    ctx->nb_slices               = 0;
    ctx->slice_offsets_allocated = 0;

    av_refstruct_unref(&ctx->decoder);
    av_refstruct_pool_uninit(&ctx->decoder_pool);

    return 0;
}

static void rocdec_free_dummy(struct AVHWFramesContext *ctx)
{
    av_buffer_pool_uninit(&ctx->pool);
}

static AVBufferRef *rocdec_alloc_dummy(size_t size)
{
    return av_buffer_create(NULL, 0, NULL, NULL, 0);
}

static int rocdec_init_hwframes(AVCodecContext *avctx, AVBufferRef **out_frames_ref, int dummy)
{
    AVHWFramesContext *frames_ctx;
    int ret;

    ret = avcodec_get_hw_frames_parameters(avctx,
                                           avctx->hw_device_ctx,
                                           avctx->hwaccel->pix_fmt,
                                           out_frames_ref);
    if (ret < 0)
        return ret;

    frames_ctx = (AVHWFramesContext*)(*out_frames_ref)->data;

    if (dummy) {
        // Copied from ff_decode_get_hw_frames_ctx for compatibility
        frames_ctx->initial_pool_size += 3;

        frames_ctx->free = rocdec_free_dummy;
        frames_ctx->pool = av_buffer_pool_init(0, rocdec_alloc_dummy);

        if (!frames_ctx->pool) {
            av_buffer_unref(out_frames_ref);
            return AVERROR(ENOMEM);
        }
    } else {
        // This is normally not used to actually allocate frames from
        frames_ctx->initial_pool_size = 0;
    }

    ret = av_hwframe_ctx_init(*out_frames_ref);
    if (ret < 0) {
        av_buffer_unref(out_frames_ref);
        return ret;
    }

    return 0;
}

int ff_rocdec_decode_init(AVCodecContext *avctx)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;

    RocDecDecoder        *decoder;
    AVBufferRef          *real_hw_frames_ref;
    rocDecFramePool      *pool;
    AVHWFramesContext    *frames_ctx;
    const AVPixFmtDescriptor *sw_desc;

    RocDecoderCreateInfo params = { 0 };

    rocDecVideoSurfaceFormat output_format;
    int rocddec_codec_type, rocddec_chroma_format, chroma_444;
    int ret = 0;

    int unsafe_output = !!(avctx->hwaccel_flags & AV_HWACCEL_FLAG_UNSAFE_OUTPUT);

    sw_desc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);
    if (!sw_desc)
        return AVERROR_BUG;

    rocddec_codec_type = map_avcodec_id(avctx->codec_id);
    if (rocddec_codec_type < 0) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported codec ID\n");
        return AVERROR_BUG;
    }

    rocddec_chroma_format = map_chroma_format(avctx->sw_pix_fmt);
    if (rocddec_chroma_format < 0) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported chroma format\n");
        return AVERROR(ENOSYS);
    }
    chroma_444 = ctx->supports_444 && rocddec_chroma_format == rocDecVideoSurfaceFormat_YUV444;

    if (!avctx->hw_frames_ctx) {
        ret = rocdec_init_hwframes(avctx, &avctx->hw_frames_ctx, 1);
        if (ret < 0)
            return ret;

        ret = rocdec_init_hwframes(avctx, &real_hw_frames_ref, 0);
        if (ret < 0)
            return ret;
    } else {
        real_hw_frames_ref = av_buffer_ref(avctx->hw_frames_ctx);
        if (!real_hw_frames_ref)
            return AVERROR(ENOMEM);
    }

    switch (sw_desc->comp[0].depth) {
    case 8:
        if (chroma_444) {
            output_format = rocDecVideoSurfaceFormat_YUV444;
        } else {
            output_format = rocDecVideoSurfaceFormat_NV12;
        }
        break;
    case 10:
    case 12:
        if (chroma_444) {
            output_format = rocDecVideoSurfaceFormat_YUV444_16Bit;
        } else {
            output_format = rocDecVideoSurfaceFormat_P016;
        }
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported bit depth\n");
        av_buffer_unref(&real_hw_frames_ref);
        return AVERROR(ENOSYS);
    }

    frames_ctx = (AVHWFramesContext*)avctx->hw_frames_ctx->data;

    params.width               = avctx->coded_width;
    params.height              = avctx->coded_height;
    params.target_width        = avctx->coded_width;
    params.target_height       = avctx->coded_height;
    params.bit_depth_minus_8   = sw_desc->comp[0].depth - 8;
    params.output_format       = output_format;
    params.codec_type          = rocddec_codec_type;
    params.chroma_format       = rocddec_chroma_format;
    params.num_decode_surfaces = frames_ctx->initial_pool_size;
    params.num_output_surfaces = unsafe_output ? frames_ctx->initial_pool_size : 1;

    ret = rocdec_decoder_create(&ctx->decoder, frames_ctx->device_ref, &params, avctx);
    if (ret < 0) {
        //TODO: Check max number of surfaces
        if (params.num_decode_surfaces > 7) {
            av_log(avctx, AV_LOG_WARNING, "Using more than 7 (%d) decode surfaces might cause rocdec to fail.\n",
                   (int)params.num_decode_surfaces);
            av_log(avctx, AV_LOG_WARNING, "Try lowering the amount of threads. Using %d right now.\n",
                   avctx->thread_count);
        }
        av_buffer_unref(&real_hw_frames_ref);
        return ret;
    }

    decoder = ctx->decoder;
    decoder->unsafe_output = unsafe_output;
    decoder->real_hw_frames_ref = real_hw_frames_ref;
    real_hw_frames_ref = NULL;

    pool = av_mallocz(sizeof(*pool));
    if (!pool) {
        ff_rocdec_decode_uninit(avctx);
        return AVERROR(ENOMEM);;
    }
    pool->dpb_size = frames_ctx->initial_pool_size;

    ctx->decoder_pool = av_refstruct_pool_alloc_ext(sizeof(unsigned int), 0, pool,
                                                    rocdec_decoder_frame_init,
                                                    NULL, NULL, rocdec_decoder_frame_pool_free);
    if (!ctx->decoder_pool) {
        ff_rocdec_decode_uninit(avctx);
        return AVERROR(ENOMEM);
    }

    return 0;
}

static void rocdec_fdd_priv_free(void *priv)
{
    RocDecFrame *cf = priv;

    if (!cf)
        return;

    av_refstruct_unref(&cf->idx_ref);
    av_refstruct_unref(&cf->ref_idx_ref);
    av_refstruct_unref(&cf->decoder);

    av_freep(&priv);
}

static void rocdec_unmap_mapped_frame(void *opaque, uint8_t *data)
{
    RocDecFrame *unmap_data = (RocDecFrame*)data;

    // TODO: Check how to unmap Frame
    //RocDecDecoder *decoder = unmap_data->decoder;
    //hipDeviceptr_t devptr = (hipDeviceptr_t)opaque;
    //CHECK_ROCDECODE(decoder->cvdl->cuvidUnmapVideoFrame(decoder->decoder, devptr));

    av_refstruct_unref(&unmap_data->idx_ref);
    av_refstruct_unref(&unmap_data->ref_idx_ref);
    av_refstruct_unref(&unmap_data->decoder);
    av_free(unmap_data);
}

static int rocdec_retrieve_data(void *logctx, AVFrame *frame)
{
    FrameDecodeData  *fdd = (FrameDecodeData*)frame->private_ref->data;
    RocDecFrame        *cf = (RocDecFrame*)fdd->hwaccel_priv;
    RocDecDecoder *decoder = cf->decoder;

    AVHWFramesContext *hwctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;

    RocdecProcParams vpp = { 0 };
    RocDecFrame *unmap_data = NULL;

    // TODO: check for device ptr
    void* devptr[3] = { 0 };;

    uint32_t pitch;
    unsigned int offset = 0;
    int i, shift_h = 0, shift_v = 0;
    int ret = 0;

    vpp.progressive_frame = 1;

    // TODO: Check how to map frames? GetFrame?
    ret = CHECK_ROCDECODE(rocDecGetVideoFrame(decoder->decoder,
                                                     cf->idx, devptr,
                                                     &pitch, &vpp));
    if (ret < 0){
        if (decoder->unsafe_output)
            return ret;

        return av_frame_make_writable(frame);
    }

    unmap_data = av_mallocz(sizeof(*unmap_data));
    if (!unmap_data) {
        ret = AVERROR(ENOMEM);
        if (!frame->buf[1]) {
            // TODO: CHeck how to unmap a frame
            //CHECK_CU(decoder->cvdl->cuvidUnmapVideoFrame(decoder->decoder, devptr));
            av_freep(&unmap_data);
        } else {
            av_buffer_unref(&frame->buf[1]);
        }
    }

    frame->buf[1] = av_buffer_create((uint8_t *)unmap_data, sizeof(*unmap_data),
                                     rocdec_unmap_mapped_frame, (void*)devptr,
                                     AV_BUFFER_FLAG_READONLY);
    if (!frame->buf[1]) {
        ret = AVERROR(ENOMEM);
        if (!frame->buf[1]) {
            // TODO: CHeck how to unmap a frame
            //CHECK_CU(decoder->cvdl->cuvidUnmapVideoFrame(decoder->decoder, devptr));
            av_freep(&unmap_data);
        } else {
            av_buffer_unref(&frame->buf[1]);
        }
    }

    ret = av_buffer_replace(&frame->hw_frames_ctx, decoder->real_hw_frames_ref);
    if (ret < 0) {
        if (!frame->buf[1]) {
            // TODO: CHeck how to unmap a frame
            //CHECK_CU(decoder->cvdl->cuvidUnmapVideoFrame(decoder->decoder, devptr));
            av_freep(&unmap_data);
        } else {
            av_buffer_unref(&frame->buf[1]);
        }
    }

    unmap_data->idx = cf->idx;
    unmap_data->idx_ref = av_refstruct_ref(cf->idx_ref);
    unmap_data->decoder = av_refstruct_ref(cf->decoder);

    av_pix_fmt_get_chroma_sub_sample(hwctx->sw_format, &shift_h, &shift_v);
    for (i = 0; frame->linesize[i]; i++) {
        frame->data[i] = (uint8_t*)(devptr + offset);
        frame->linesize[i] = pitch;
        offset += pitch * (frame->height >> (i ? shift_v : 0));
    }

    if (decoder->unsafe_output)
        return ret;

    return av_frame_make_writable(frame);
}

int ff_rocdec_start_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    FrameDecodeData *fdd = (FrameDecodeData*)frame->private_ref->data;
    RocDecFrame *cf = NULL;
    int ret;

    ctx->bitstream_len = 0;
    ctx->nb_slices     = 0;

    if (fdd->hwaccel_priv)
        return 0;

    cf = av_mallocz(sizeof(*cf));
    if (!cf)
        return AVERROR(ENOMEM);

    cf->decoder = av_refstruct_ref(ctx->decoder);

    cf->idx_ref = av_refstruct_pool_get(ctx->decoder_pool);
    if (!cf->idx_ref) {
        av_log(avctx, AV_LOG_ERROR, "No decoder surfaces left\n");
        ret = AVERROR(ENOMEM);
        rocdec_fdd_priv_free(cf);
        return ret;
    }
    cf->ref_idx = cf->idx = *cf->idx_ref;

    fdd->hwaccel_priv      = cf;
    fdd->hwaccel_priv_free = rocdec_fdd_priv_free;
    fdd->post_process      = rocdec_retrieve_data;

    return 0;

}

int ff_rocdec_start_frame_sep_ref(AVCodecContext *avctx, AVFrame *frame, int has_sep_ref)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    FrameDecodeData *fdd = (FrameDecodeData*)frame->private_ref->data;
    RocDecFrame *cf;
    int ret;

    ret = ff_rocdec_start_frame(avctx, frame);
    if (ret < 0)
        return ret;

    cf = fdd->hwaccel_priv;

    if (has_sep_ref) {
        if (!cf->ref_idx_ref) {
            cf->ref_idx_ref = av_refstruct_pool_get(ctx->decoder_pool);
            if (!cf->ref_idx_ref) {
                av_log(avctx, AV_LOG_ERROR, "No decoder surfaces left\n");
                ret = AVERROR(ENOMEM);
                rocdec_fdd_priv_free(cf);
                return ret;
            }
        }
        cf->ref_idx = *cf->ref_idx_ref;
    } else {
        av_refstruct_unref(&cf->ref_idx_ref);
        cf->ref_idx = cf->idx;
    }

    return 0;
}

int ff_rocdec_end_frame(AVCodecContext *avctx)
{
    RocDecContext     *ctx = avctx->internal->hwaccel_priv_data;
    RocDecDecoder *decoder = ctx->decoder;
    RocdecPicParams    *pp = &ctx->pic_params;

    int ret = 0;

    pp->bitstream_data_len = ctx->bitstream_len;
    pp->bitstream_data    = ctx->bitstream;
    pp->num_slices        = ctx->nb_slices;
    // TODO: Found only in particular codec picParams; do I check codec and call based on that?
    //pp->slice_data_offset = ctx->slice_offsets;

    ret = CHECK_ROCDECODE(rocDecDecodeFrame(decoder->decoder, &ctx->pic_params));

    return ret;
}

int ff_rocdec_simple_end_frame(AVCodecContext *avctx)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret = ff_rocdec_end_frame(avctx);
    ctx->bitstream = NULL;
    ctx->bitstream_len = 0;
    ctx->nb_slices = 0;
    return ret;
}

int ff_rocdec_simple_decode_slice(AVCodecContext *avctx, const uint8_t *buffer,
                                 uint32_t size)
{
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    void *tmp;

    tmp = av_fast_realloc(ctx->slice_offsets, &ctx->slice_offsets_allocated,
                          (ctx->nb_slices + 1) * sizeof(*ctx->slice_offsets));
    if (!tmp)
        return AVERROR(ENOMEM);
    ctx->slice_offsets = tmp;

    if (!ctx->bitstream)
        ctx->bitstream = buffer;

    ctx->slice_offsets[ctx->nb_slices] = buffer - ctx->bitstream;
    ctx->bitstream_len += size;
    ctx->nb_slices++;

    return 0;
}

int ff_rocdec_frame_params(AVCodecContext *avctx,
                          AVBufferRef *hw_frames_ctx,
                          int dpb_size,
                          int supports_444)
{
    AVHWFramesContext *frames_ctx = (AVHWFramesContext*)hw_frames_ctx->data;
    const AVPixFmtDescriptor *sw_desc;
    int rocddec_codec_type, rocddec_chroma_format, chroma_444;

    sw_desc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);
    if (!sw_desc)
        return AVERROR_BUG;

    rocddec_codec_type = map_avcodec_id(avctx->codec_id);
    if (rocddec_codec_type < 0) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported codec ID\n");
        return AVERROR_BUG;
    }

    rocddec_chroma_format = map_chroma_format(avctx->sw_pix_fmt);
    if (rocddec_chroma_format < 0) {
        av_log(avctx, AV_LOG_VERBOSE, "Unsupported chroma format\n");
        return AVERROR(EINVAL);
    }
    chroma_444 = supports_444 && rocddec_chroma_format == rocDecVideoChromaFormat_444;

    frames_ctx->format            = AV_PIX_FMT_CUDA;
    frames_ctx->width             = (avctx->coded_width + 1) & ~1;
    frames_ctx->height            = (avctx->coded_height + 1) & ~1;
    /*
     * We add two extra frames to the pool to account for deinterlacing filters
     * holding onto their frames.
     */
    frames_ctx->initial_pool_size = dpb_size + 2;

    switch (sw_desc->comp[0].depth) {
    case 8:
        if (chroma_444) {
            frames_ctx->sw_format = AV_PIX_FMT_YUV444P;
        } else {
            frames_ctx->sw_format = AV_PIX_FMT_NV12;
        }
        break;
    case 10:
    case 12:
        if (chroma_444) {
            frames_ctx->sw_format = AV_PIX_FMT_YUV444P16;
        } else {
            frames_ctx->sw_format = AV_PIX_FMT_P016LE;
        }
        break;
    default:
        return AVERROR(EINVAL);
    }

    return 0;
}

int ff_rocdec_get_ref_idx(AVFrame *frame)
{
    FrameDecodeData *fdd;
    RocDecFrame *cf;

    if (!frame || !frame->private_ref)
        return -1;

    fdd = (FrameDecodeData*)frame->private_ref->data;
    cf  = (RocDecFrame*)fdd->hwaccel_priv;
    if (!cf)
        return -1;

    return cf->ref_idx;
}
