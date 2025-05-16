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

#include "config_components.h"

#include "libavutil/buffer.h"
#include "libavutil/mathematics.h"
#include "libavutil/hwcontext.h"
#include "libavutil/rocdecode_check.h"
#include "libavutil/hwcontext_rocdecode.h"
#include "libavutil/fifo.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include <hip/driver_types.h>

#include "avcodec.h"
#include "bsf.h"
#include "codec_internal.h"
#include "decode.h"
#include "hwconfig.h"
#include "internal.h"

typedef struct RocdecContext
{
    AVClass *avclass;

    RocdecVideoParser rocdecparser;
    rocDecDecoderHandle rocdecdecoder;

    /* This packet coincides with AVCodecInternal.in_pkt
     * and is not owned by us. */
    AVPacket *pkt;

    char *amd_gpu;
    int nb_surfaces;
    char *crop_expr;
    char *resize_expr;

    struct {
        int left;
        int top;
        int right;
        int bottom;
    } crop;

    struct {
        int width;
        int height;
    } resize;

    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;

    AVFifo      *frame_queue;

    int deint_mode;
    int deint_mode_current;
    int64_t prev_pts;
    int progressive_sequence;

    int internal_error;
    int decoder_flushing;

    int *key_frame;

    rocDecVideoCodec codec_type;
    rocDecVideoChromaFormat chroma_format;

    RocdecDecodeCaps caps8, caps10;

    RocdecParserParams rocdec_parse_info;
    RocdecVideoFormatEx *rocdec_parse_ext;

} RocdecContext;

#define CHECK_ROCDECODE(x) FF_ROCDECODE_CHECK(x)
#define CHECK_HIP(x) FF_HIP_CHECK(avctx, x)

// NV recommends [2;4] range
#define ROCDEC_MAX_DISPLAY_DELAY (4)

// Actual pool size will be determined by parser.
#define ROCDEC_DEFAULT_NUM_SURFACES (ROCDEC_MAX_DISPLAY_DELAY + 1)


static int ROCDECAPI rocdec_handle_video_sequence(void *opaque, RocdecVideoFormat* format) 
{
    AVCodecContext *avctx = opaque;
    RocdecContext *ctx = avctx->priv_data;
    AVHWFramesContext *hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    RocdecDecodeCaps *caps = NULL;
    RocDecoderCreateInfo rocdecinfo;
    int surface_fmt;
    int chroma_444;
    int old_nb_surfaces, fifo_size_inc, fifo_size_mul = 1;

    int old_width = avctx->width;
    int old_height = avctx->height;

    enum AVPixelFormat pix_fmts[3] = { AV_PIX_FMT_AMD_GPU,
                                       AV_PIX_FMT_NONE,  // Will be updated below
                                       AV_PIX_FMT_NONE };

    av_log(avctx, AV_LOG_VERBOSE, "pfnSequenceCallback, progressive_sequence=%d\n", format->progressive_sequence);

    memset(&rocdecinfo, 0, sizeof(rocdecinfo));

    ctx->internal_error = 0;

    avctx->coded_width = rocdecinfo.width = format->coded_width;
    avctx->coded_height = rocdecinfo.height = format->coded_height;

    // apply cropping
    rocdecinfo.display_rect.left = format->display_area.left + ctx->crop.left;
    rocdecinfo.display_rect.top = format->display_area.top + ctx->crop.top;
    rocdecinfo.display_rect.right = format->display_area.right - ctx->crop.right;
    rocdecinfo.display_rect.bottom = format->display_area.bottom - ctx->crop.bottom;

    // width and height need to be set before calling ff_get_format
    if (ctx->resize_expr) {
        avctx->width = ctx->resize.width;
        avctx->height = ctx->resize.height;
    } else {
        avctx->width = rocdecinfo.display_rect.right - rocdecinfo.display_rect.left;
        avctx->height = rocdecinfo.display_rect.bottom - rocdecinfo.display_rect.top;
    }

    // target width/height need to be multiples of two
    rocdecinfo.target_width = avctx->width = (avctx->width + 1) & ~1;
    rocdecinfo.target_height = avctx->height = (avctx->height + 1) & ~1;

    // aspect ratio conversion, 1:1, depends on scaled resolution
    rocdecinfo.target_rect.left = 0;
    rocdecinfo.target_rect.top = 0;
    rocdecinfo.target_rect.right = rocdecinfo.target_width;
    rocdecinfo.target_rect.bottom = rocdecinfo.target_height;

    chroma_444 = format->chroma_format == rocDecVideoChromaFormat_444;

    switch (format->bit_depth_luma_minus8) {
        case 0: // 8-bit
            if (chroma_444) {
                pix_fmts[1] = AV_PIX_FMT_YUV444P;
            } else {
                pix_fmts[1] = AV_PIX_FMT_NV12;
            }
            caps = &ctx->caps8;
            break;
        case 2: // 10-bit
            if (chroma_444) {
                pix_fmts[1] = AV_PIX_FMT_YUV444P16;
            } else if (format->chroma_format == rocDecVideoChromaFormat_422) {
                pix_fmts[1] = AV_PIX_FMT_P216LE;
            } else {
                pix_fmts[1] = AV_PIX_FMT_P016;
            }
            caps = &ctx->caps10;
            break;
        default:
            break;
    }

    if (!caps || !caps->is_supported) {
        av_log(avctx, AV_LOG_ERROR, "unsupported bit depth: %d\n",
               format->bit_depth_luma_minus8 + 8);
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    surface_fmt = ff_get_format(avctx, pix_fmts);
    if (surface_fmt < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed: %d\n", surface_fmt);
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    av_log(avctx, AV_LOG_VERBOSE, "Formats: Original: %s | HW: %s | SW: %s\n",
           av_get_pix_fmt_name(avctx->pix_fmt),
           av_get_pix_fmt_name(surface_fmt),
           av_get_pix_fmt_name(avctx->sw_pix_fmt));

    avctx->pix_fmt = surface_fmt;

    // Update our hwframe ctx, as the get_format callback might have refreshed it!
    if (avctx->hw_frames_ctx) {
        av_buffer_unref(&ctx->hwframe);

        ctx->hwframe = av_buffer_ref(avctx->hw_frames_ctx);
        if (!ctx->hwframe) {
            ctx->internal_error = AVERROR(ENOMEM);
            return 0;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    }

    ff_set_sar(avctx, av_div_q(
        (AVRational){ format->display_aspect_ratio.x, format->display_aspect_ratio.y },
        (AVRational){ avctx->width, avctx->height }));

    // TODO: Check for our default
    /*ctx->deint_mode_current = format->progressive_sequence
                              ? cudaVideoDeinterlaceMode_Weave
                              : ctx->deint_mode;
    */
    ctx->progressive_sequence = format->progressive_sequence;

    /*if (!format->progressive_sequence && ctx->deint_mode_current == cudaVideoDeinterlaceMode_Weave)
        avctx->flags |= AV_CODEC_FLAG_INTERLACED_DCT;
    else
        avctx->flags &= ~AV_CODEC_FLAG_INTERLACED_DCT;
    */

    // TODO: Can i use below instead of above?
    avctx->flags &= ~AV_CODEC_FLAG_INTERLACED_DCT;

    if (format->video_signal_description.video_full_range_flag)
        avctx->color_range = AVCOL_RANGE_JPEG;
    else
        avctx->color_range = AVCOL_RANGE_MPEG;

    avctx->color_primaries = format->video_signal_description.color_primaries;
    avctx->color_trc = format->video_signal_description.transfer_characteristics;
    avctx->colorspace = format->video_signal_description.matrix_coefficients;

    if (format->bitrate)
        avctx->bit_rate = format->bitrate;

    if (format->frame_rate.numerator && format->frame_rate.denominator) {
        avctx->framerate.num = format->frame_rate.numerator;
        avctx->framerate.den = format->frame_rate.denominator;
    }

    if (ctx->rocdecdecoder
            && avctx->coded_width == format->coded_width
            && avctx->coded_height == format->coded_height
            && avctx->width == old_width
            && avctx->height == old_height
            && ctx->chroma_format == format->chroma_format
            && ctx->codec_type == format->codec)
        return 1;

    if (ctx->rocdecdecoder) {
        av_log(avctx, AV_LOG_VERBOSE, "Re-initializing decoder\n");
        ctx->internal_error = CHECK_ROCDECODE(rocDecDestroyDecoder(ctx->rocdecdecoder));
        if (ctx->internal_error < 0)
            return 0;
        ctx->rocdecdecoder = NULL;
    }

    if (hwframe_ctx->pool && (
            hwframe_ctx->width < avctx->width ||
            hwframe_ctx->height < avctx->height ||
            hwframe_ctx->format != AV_PIX_FMT_AMD_GPU ||
            hwframe_ctx->sw_format != avctx->sw_pix_fmt)) {
        av_log(avctx, AV_LOG_ERROR, "AVHWFramesContext is already initialized with incompatible parameters\n");
        av_log(avctx, AV_LOG_DEBUG, "width: %d <-> %d\n", hwframe_ctx->width, avctx->width);
        av_log(avctx, AV_LOG_DEBUG, "height: %d <-> %d\n", hwframe_ctx->height, avctx->height);
        av_log(avctx, AV_LOG_DEBUG, "format: %s <-> rocdec\n", av_get_pix_fmt_name(hwframe_ctx->format));
        av_log(avctx, AV_LOG_DEBUG, "sw_format: %s <-> %s\n",
               av_get_pix_fmt_name(hwframe_ctx->sw_format), av_get_pix_fmt_name(avctx->sw_pix_fmt));
        ctx->internal_error = AVERROR(EINVAL);
        return 0;
    }

    ctx->chroma_format = format->chroma_format;

    rocdecinfo.codec_type = ctx->codec_type = format->codec;
    rocdecinfo.chroma_format = format->chroma_format;

    switch (avctx->sw_pix_fmt) {
        case AV_PIX_FMT_NV12:
            rocdecinfo.output_format = rocDecVideoSurfaceFormat_NV12;
            break;
        case AV_PIX_FMT_P010:
        case AV_PIX_FMT_P016:
            rocdecinfo.output_format = rocDecVideoSurfaceFormat_P016;
            break;
        case AV_PIX_FMT_YUV444P:
            rocdecinfo.output_format = rocDecVideoSurfaceFormat_YUV444;
            break;
        case AV_PIX_FMT_YUV444P16:
            rocdecinfo.output_format = rocDecVideoSurfaceFormat_YUV444_16Bit;
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "Unsupported output format: %s\n",
                av_get_pix_fmt_name(avctx->sw_pix_fmt));
            ctx->internal_error = AVERROR(EINVAL);
            return 0;
    }

    old_nb_surfaces = ctx->nb_surfaces;
    ctx->nb_surfaces = FFMAX(ctx->nb_surfaces, format->min_num_decode_surfaces + 3);
    if (avctx->extra_hw_frames > 0)
        ctx->nb_surfaces += avctx->extra_hw_frames;

    fifo_size_inc = ctx->nb_surfaces * fifo_size_mul - av_fifo_can_read(ctx->frame_queue) - av_fifo_can_write(ctx->frame_queue);
    if (fifo_size_inc > 0 && av_fifo_grow2(ctx->frame_queue, fifo_size_inc) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to grow frame queue on video sequence callback\n");
        ctx->internal_error = AVERROR(ENOMEM);
        return 0;
    }

    if (ctx->nb_surfaces > old_nb_surfaces && av_reallocp_array(&ctx->key_frame, ctx->nb_surfaces, sizeof(int)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to grow key frame array on video sequence callback\n");
        ctx->internal_error = AVERROR(ENOMEM);
        return 0;
    }

    rocdecinfo.num_decode_surfaces = ctx->nb_surfaces;
    rocdecinfo.num_output_surfaces = 1;
    rocdecinfo.bit_depth_minus_8 = format->bit_depth_luma_minus8;

    ctx->internal_error = CHECK_ROCDECODE(rocDecCreateDecoder(&ctx->rocdecdecoder, &rocdecinfo));
    if (ctx->internal_error < 0)
        return 0;

    if (!hwframe_ctx->pool) {
        hwframe_ctx->format = AV_PIX_FMT_AMD_GPU;
        hwframe_ctx->sw_format = avctx->sw_pix_fmt;
        hwframe_ctx->width = avctx->width;
        hwframe_ctx->height = avctx->height;

        if ((ctx->internal_error = av_hwframe_ctx_init(ctx->hwframe)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_ctx_init failed\n");
            return 0;
        }
    }

    if(ctx->rocdec_parse_info.max_num_decode_surfaces != rocdecinfo.num_decode_surfaces) {
        ctx->rocdec_parse_info.max_num_decode_surfaces = rocdecinfo.num_decode_surfaces;
        return rocdecinfo.num_decode_surfaces;
    }

    return 1;
}

static int ROCDECAPI rocdec_handle_picture_decode(void *opaque, RocdecPicParams* picparams)
{
    AVCodecContext *avctx = opaque;
    RocdecContext *ctx = avctx->priv_data;

    av_log(avctx, AV_LOG_VERBOSE, "pfnDecodePicture\n");

    if(picparams->intra_pic_flag)
        ctx->key_frame[picparams->curr_pic_idx] = picparams->intra_pic_flag;

    ctx->internal_error = CHECK_ROCDECODE(rocDecDecodeFrame(ctx->rocdecdecoder, picparams));
    if (ctx->internal_error < 0)
        return 0;

    return 1;
}

static int ROCDECAPI rocdec_handle_picture_display(void *opaque, RocdecParserDispInfo* dispinfo)
{
    AVCodecContext *avctx = opaque;
    RocdecContext *ctx = avctx->priv_data;
    ctx->internal_error = 0;

    // For some reason, dispinfo->progressive_frame is sometimes wrong.
    dispinfo->progressive_frame = ctx->progressive_sequence;

    av_fifo_write(ctx->frame_queue, dispinfo, 1);

    return 1;
}

static int rocdec_is_buffer_full(AVCodecContext *avctx)
{
    RocdecContext *ctx = avctx->priv_data;

    int shift = 0;

    // shift/divide frame count to ensure the buffer is still signalled full if one half-frame has already been returned when deinterlacing.
    return ((av_fifo_can_read(ctx->frame_queue) + shift) >> shift) + ctx->rocdec_parse_info.max_display_delay >= ctx->nb_surfaces;
}

static int rocdec_decode_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    RocdecContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVRocDecodeDeviceContext *device_hwctx = device_ctx->hwctx;
    RocdecSourceDataPacket rocdec_pkt;
    int ret = 0, eret = 0, is_flush = ctx->decoder_flushing;

    av_log(avctx, AV_LOG_VERBOSE, "rocdec_decode_packet\n");

    if (is_flush && avpkt && avpkt->size)
        return AVERROR_EOF;

    if (rocdec_is_buffer_full(avctx) && avpkt && avpkt->size)
        return AVERROR(EAGAIN);

    memset(&rocdec_pkt, 0, sizeof(rocdec_pkt));

    if (avpkt && avpkt->size) {
        rocdec_pkt.payload_size = avpkt->size;
        rocdec_pkt.payload = avpkt->data;

        if (avpkt->pts != AV_NOPTS_VALUE) {
            rocdec_pkt.flags = ROCDEC_PKT_TIMESTAMP;
            if (avctx->pkt_timebase.num && avctx->pkt_timebase.den)
                rocdec_pkt.pts = av_rescale_q(avpkt->pts, avctx->pkt_timebase, (AVRational){1, 10000000});
            else
                rocdec_pkt.pts = avpkt->pts;
        }
    } else {
        rocdec_pkt.flags = ROCDEC_PKT_ENDOFSTREAM;
        ctx->decoder_flushing = 1;
    }

    ret = CHECK_ROCDECODE(rocDecParseVideoData(ctx->rocdecparser, &rocdec_pkt));
    if (ret < 0) {
        if (is_flush)
            return AVERROR_EOF;
        else
            return ret;
    }

    // rocDecParseVideoData doesn't return an error just because stuff failed...
    if (ctx->internal_error) {
        av_log(avctx, AV_LOG_ERROR, "rocDecode callback error\n");
        ret = ctx->internal_error;
        if (ret < 0) {
            if (is_flush)
                return AVERROR_EOF;
            else
                return ret;
        }
    }
    return 0;
}

static int rocdec_output_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RocdecContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVRocDecodeDeviceContext *device_hwctx = device_ctx->hwctx;
    RocdecParserDispInfo parsed_frame;
    int ret = 0, eret = 0;

    av_log(avctx, AV_LOG_VERBOSE, "rocdec_output_frame with pixel format %s\n", av_get_pix_fmt_name(avctx->pix_fmt));

    if (ctx->decoder_flushing) {
        ret = rocdec_decode_packet(avctx, NULL);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
    }

    if (!rocdec_is_buffer_full(avctx)) {
        AVPacket *const pkt = ctx->pkt;
        ret = ff_decode_get_packet(avctx, pkt);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
        ret = rocdec_decode_packet(avctx, pkt);
        av_packet_unref(pkt);
        // rocdec_is_buffer_full() should avoid this.
        if (ret == AVERROR(EAGAIN))
            ret = AVERROR_EXTERNAL;
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;
    }

    if (av_fifo_read(ctx->frame_queue, &parsed_frame, 1) >= 0) {
        const AVPixFmtDescriptor *pixdesc;
        RocdecProcParams params;
        void * src_dev_ptr[3] = { 0 };
        uint32_t src_pitch[3] = { 0 };
        int offset = 0;
        int i;

        memset(&params, 0, sizeof(params));
        params.progressive_frame = parsed_frame.progressive_frame;
        params.top_field_first = parsed_frame.top_field_first;

        ret = CHECK_ROCDECODE(rocDecGetVideoFrame(ctx->rocdecdecoder, parsed_frame.picture_index,
                            src_dev_ptr, src_pitch, &params));
        if (ret < 0) {
            av_frame_unref(frame);
            return ret;
        }

        RocdecDecodeStatus dec_status;
        memset(&dec_status, 0, sizeof(dec_status));
        rocDecStatus result = rocDecGetDecodeStatus(ctx->rocdecdecoder, parsed_frame.picture_index, &dec_status);
        if (result == ROCDEC_SUCCESS && (dec_status.decode_status == rocDecodeStatus_Error || dec_status.decode_status == rocDecodeStatus_Error_Concealed)) {
            av_log(avctx, AV_LOG_ERROR, "RocDecode -- Decode Error occurred for picture: %d", parsed_frame.picture_index);
        }

        if (avctx->pix_fmt == AV_PIX_FMT_AMD_GPU) {
            ret = av_hwframe_get_buffer(ctx->hwframe, frame, 0);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "av_hwframe_get_buffer failed\n");
                av_frame_unref(frame);
                return ret;
            }

            ret = ff_decode_frame_props(avctx, frame);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "ff_decode_frame_props failed\n");
                av_frame_unref(frame);
                return ret;
            }

            pixdesc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);

            uint32_t byte_per_pixel_ = ctx->rocdec_parse_info.ext_video_info->format.bit_depth_luma_minus8 > 0 ? 2 : 1;
            uint8_t *p_src_ptr_y = (uint8_t *)(src_dev_ptr[0]) + 
                                (ctx->rocdec_parse_info.ext_video_info->format.display_area.top + ctx->crop.top) * src_pitch[0] + 
                                (ctx->rocdec_parse_info.ext_video_info->format.display_area.left + ctx->crop.left) * byte_per_pixel_;

            for (i = 0; i < pixdesc->nb_components; i++) {
                int height = avctx->height >> (i ? pixdesc->log2_chroma_h : 0);
                hip_Memcpy2D cpy = {
                    .srcMemoryType = hipMemoryTypeDevice,
                    .dstMemoryType = hipMemoryTypeDevice,
                    .srcDevice     = src_dev_ptr[i],
                    .dstDevice     = (void *)frame->data[i],
                    .srcPitch      = src_pitch[i],
                    .dstPitch      = frame->linesize[i],
                    .srcY          = 0,
                    .WidthInBytes  = FFMIN(src_pitch[i], frame->linesize[i]),
                    .Height        = height,
                };

                ret = CHECK_HIP(hipMemcpyParam2DAsync(&cpy, device_hwctx->stream));
                if (ret < 0) {
                    av_frame_unref(frame);
                    return ret;
                }
            }
        } else if (avctx->pix_fmt == AV_PIX_FMT_NV12      ||
                   avctx->pix_fmt == AV_PIX_FMT_P010      ||
                   avctx->pix_fmt == AV_PIX_FMT_P016      ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P   ||
                   avctx->pix_fmt == AV_PIX_FMT_YUV444P16) {
            unsigned int offset = 0;
            AVFrame *tmp_frame = av_frame_alloc();
            if (!tmp_frame) {
                av_log(avctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
                ret = AVERROR(ENOMEM);
                av_frame_unref(frame);
                return ret;
            }

            pixdesc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);

            tmp_frame->format        = AV_PIX_FMT_AMD_GPU;
            tmp_frame->hw_frames_ctx = av_buffer_ref(ctx->hwframe);
            if (!tmp_frame->hw_frames_ctx) {
                ret = AVERROR(ENOMEM);
                av_frame_free(&tmp_frame);
                av_frame_unref(frame);
                return ret;
            }

            tmp_frame->width         = avctx->width;
            tmp_frame->height        = avctx->height;

            /*
             * Note that the following logic would not work for three plane
             * YUV420 because the pitch value is different for the chroma
             * planes.
             */
            uint32_t byte_per_pixel_ = ctx->rocdec_parse_info.ext_video_info->format.bit_depth_luma_minus8 > 0 ? 2 : 1;

            for (i = 0; i < pixdesc->nb_components; i++) {
                uint8_t *p_src_ptr_y = (uint8_t *)(src_dev_ptr[i]) + 
                                (ctx->rocdec_parse_info.ext_video_info->format.display_area.top + ctx->crop.top) * src_pitch[i] + 
                                (ctx->rocdec_parse_info.ext_video_info->format.display_area.left + ctx->crop.left) * byte_per_pixel_;
                tmp_frame->data[i]     = (uint8_t*)p_src_ptr_y;
                tmp_frame->linesize[i] = src_pitch[i];
                //tmp_frame->height = avctx->height >> (i ? pixdesc->log2_chroma_h : 0);
            }

            ret = ff_get_buffer(avctx, frame, 0);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "ff_get_buffer failed\n");
                av_frame_free(&tmp_frame);
                av_frame_unref(frame);
                return ret;
            }

            ret = av_hwframe_transfer_data(frame, tmp_frame, 0);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "av_hwframe_transfer_data failed\n");
                av_frame_free(&tmp_frame);
                av_frame_unref(frame);
                return ret;
            }
            av_frame_free(&tmp_frame);
        } else {
            ret = AVERROR_BUG;
            av_frame_unref(frame);
            return ret;
        }

        if (ctx->key_frame[parsed_frame.picture_index])
            frame->flags |= AV_FRAME_FLAG_KEY;
        else
            frame->flags &= ~AV_FRAME_FLAG_KEY;
        ctx->key_frame[parsed_frame.picture_index] = 0;

        frame->width = avctx->width;
        frame->height = avctx->height;
        if (avctx->pkt_timebase.num && avctx->pkt_timebase.den)
            frame->pts = av_rescale_q(parsed_frame.pts, (AVRational){1, 10000000}, avctx->pkt_timebase);
        else
            frame->pts = parsed_frame.pts;

        /* CUVIDs opaque reordering breaks the internal pkt logic.
         * So set pkt_pts and clear all the other pkt_ fields.
         */
        frame->duration = 0;

        if (!parsed_frame.progressive_frame)
            frame->flags |= AV_FRAME_FLAG_INTERLACED;

        if ((frame->flags & AV_FRAME_FLAG_INTERLACED) && parsed_frame.top_field_first)
            frame->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;
    } else if (ctx->decoder_flushing) {
        ret = AVERROR_EOF;
    } else {
        ret = AVERROR(EAGAIN);
    }
    return ret;
}

static av_cold int rocdec_decode_end(AVCodecContext *avctx)
{
    RocdecContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = ctx->hwdevice ? (AVHWDeviceContext *)ctx->hwdevice->data : NULL;
    AVRocDecodeDeviceContext *device_hwctx = device_ctx ? device_ctx->hwctx : NULL;

    av_fifo_freep2(&ctx->frame_queue);

    if (device_hwctx) {
        if (ctx->rocdecparser) {
            rocDecDestroyVideoParser(ctx->rocdecparser);
            ctx->rocdecparser = NULL;
        }

        if (ctx->rocdecdecoder) {
            rocDecDestroyDecoder(ctx->rocdecdecoder);
            ctx->rocdecdecoder = NULL;
        }
    }

    av_buffer_unref(&ctx->hwframe);
    av_buffer_unref(&ctx->hwdevice);

    av_freep(&ctx->key_frame);
    av_freep(&ctx->rocdec_parse_ext);

    return 0;
}

static int rocdec_test_capabilities(AVCodecContext *avctx,
    const RocDecoderCreateInfo *rocdec_parse_info,
    int probed_width,
    int probed_height,
    int bit_depth, int is_yuv422, int is_yuv444) 
{

    RocdecContext *ctx = avctx->priv_data;
    RocdecDecodeCaps *caps;
    int res8 = 0, res10 = 0;


    ctx->caps8.codec_type = ctx->caps10.codec_type = rocdec_parse_info->codec_type;

    ctx->caps8.chroma_format = ctx->caps10.chroma_format = 
    is_yuv444 ? rocDecVideoChromaFormat_444 :
    (is_yuv422 ? rocDecVideoChromaFormat_422 : rocDecVideoChromaFormat_420);


    ctx->caps8.bit_depth_minus_8 = 0;
    ctx->caps10.bit_depth_minus_8 = 2;

    res8 = CHECK_ROCDECODE(rocDecGetDecoderCaps(&ctx->caps8));
    res10 = CHECK_ROCDECODE(rocDecGetDecoderCaps(&ctx->caps10));

    av_log(avctx, AV_LOG_VERBOSE, "rocDecode capabilities for %s:\n", avctx->codec->name);
    av_log(avctx, AV_LOG_VERBOSE, "8 bit: supported: %d, min_width: %d, max_width: %d, min_height: %d, max_height: %d\n",
    ctx->caps8.is_supported, ctx->caps8.min_width, ctx->caps8.max_width, ctx->caps8.min_height, ctx->caps8.max_height);
    av_log(avctx, AV_LOG_VERBOSE, "10 bit: supported: %d, min_width: %d, max_width: %d, min_height: %d, max_height: %d\n",
    ctx->caps10.is_supported, ctx->caps10.min_width, ctx->caps10.max_width, ctx->caps10.min_height, ctx->caps10.max_height);

    switch (bit_depth) {
        case 10:
            caps = &ctx->caps10;
            if (res10 < 0)
                return res10;
            break;
        default:
            caps = &ctx->caps8;
            if (res8 < 0)
                return res8;
            break;
    }

    if (!ctx->caps8.is_supported) {
        av_log(avctx, AV_LOG_ERROR, "Codec %s is not supported with this chroma format.\n", avctx->codec->name);
        return AVERROR(EINVAL);
    }

    if (!caps->is_supported) {
        av_log(avctx, AV_LOG_ERROR, "Bit depth %d with this chroma format is not supported.\n", bit_depth);
        return AVERROR(EINVAL);
    }

    if (probed_width > caps->max_width || probed_width < caps->min_width) {
        av_log(avctx, AV_LOG_ERROR, "Video width %d not within range from %d to %d\n",
        probed_width, caps->min_width, caps->max_width);
        return AVERROR(EINVAL);
    }

    if (probed_height > caps->max_height || probed_height < caps->min_height) {
        av_log(avctx, AV_LOG_ERROR, "Video height %d not within range from %d to %d\n",
        probed_height, caps->min_height, caps->max_height);
        return AVERROR(EINVAL);
    }

    // TODO: Check if we have equivalent of this
    /*if ((probed_width * probed_height) / 256 > caps->nMaxMBCount) {
        av_log(avctx, AV_LOG_ERROR, "Video macroblock count %d exceeds maximum of %d\n",
        (int)(probed_width * probed_height) / 256, caps->nMaxMBCount);
        return AVERROR(EINVAL);
    }*/

    return 0;
}

static av_cold int rocdec_decode_init(AVCodecContext *avctx)
{
    RocdecContext *ctx = avctx->priv_data;
    AVRocDecodeDeviceContext *device_hwctx;
    AVHWDeviceContext *device_ctx;
    AVHWFramesContext *hwframe_ctx;
    RocdecSourceDataPacket seq_pkt;
    uint8_t *extradata;
    int extradata_size;
    int ret = 0;

    enum AVPixelFormat pix_fmts[3] = { AV_PIX_FMT_AMD_GPU,
                                       AV_PIX_FMT_NONE,
                                       AV_PIX_FMT_NONE };

    int probed_width = avctx->coded_width ? avctx->coded_width : 1280;
    int probed_height = avctx->coded_height ? avctx->coded_height : 720;
    int probed_bit_depth = 8, is_yuv444 = 0, is_yuv422 = 0;

    const AVPixFmtDescriptor *probe_desc = av_pix_fmt_desc_get(avctx->pix_fmt);
    if (probe_desc && probe_desc->nb_components)
        probed_bit_depth = probe_desc->comp[0].depth;

    if (probe_desc && !probe_desc->log2_chroma_w && !probe_desc->log2_chroma_h)
        is_yuv444 = 1;

    if (probe_desc && probe_desc->log2_chroma_w && !probe_desc->log2_chroma_h)
        is_yuv422 = 1;


    // Pick pixel format based on bit depth and chroma sampling.
    switch (probed_bit_depth) {
        case 10:
            pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P16 : AV_PIX_FMT_P010;
            break;
        case 12:
            pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P16 : AV_PIX_FMT_P016;
            break;
        default:
            pix_fmts[1] = is_yuv444 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_NV12;
            break;
    }

    ctx->pkt = avctx->internal->in_pkt;
    // TODO: Check if we need this
    // Accelerated transcoding scenarios with 'ffmpeg' require that the
    // pix_fmt be set to AV_PIX_FMT_AMD_GPU early. The sw_pix_fmt, and the
    // pix_fmt for non-accelerated transcoding, do not need to be correct
    // but need to be set to something.
    ret = ff_get_format(avctx, pix_fmts);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed: %d\n", ret);
        return ret;
    }
    avctx->pix_fmt = ret;

    if (ctx->resize_expr && sscanf(ctx->resize_expr, "%dx%d",
                                   &ctx->resize.width, &ctx->resize.height) != 2) {
        av_log(avctx, AV_LOG_ERROR, "Invalid resize expressions\n");
        ret = AVERROR(EINVAL);
        rocdec_decode_end(avctx);
        return ret;
    }

    if (ctx->crop_expr && sscanf(ctx->crop_expr, "%dx%dx%dx%d",
                                 &ctx->crop.top, &ctx->crop.bottom,
                                 &ctx->crop.left, &ctx->crop.right) != 4) {
        av_log(avctx, AV_LOG_ERROR, "Invalid cropping expressions\n");
        ret = AVERROR(EINVAL);
        rocdec_decode_end(avctx);
        return ret;
    }

    // respect the deprecated "surfaces" option if non-default value is given by user;
    if(ctx->nb_surfaces < 0)
        ctx->nb_surfaces = ROCDEC_DEFAULT_NUM_SURFACES;

    ctx->frame_queue = av_fifo_alloc2(ctx->nb_surfaces, sizeof(RocdecParserDispInfo), 0);
    if (!ctx->frame_queue) {
        ret = AVERROR(ENOMEM);
        rocdec_decode_end(avctx);
        return ret;
    }

    if (avctx->hw_frames_ctx) {
        ctx->hwframe = av_buffer_ref(avctx->hw_frames_ctx);
        if (!ctx->hwframe) {
            ret = AVERROR(ENOMEM);
            rocdec_decode_end(avctx);
            return ret;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;

        ctx->hwdevice = av_buffer_ref(hwframe_ctx->device_ref);
        if (!ctx->hwdevice) {
            ret = AVERROR(ENOMEM);
            rocdec_decode_end(avctx);
            return ret;
        }
    } else {
        if (avctx->hw_device_ctx) {
            ctx->hwdevice = av_buffer_ref(avctx->hw_device_ctx);
            if (!ctx->hwdevice) {
                ret = AVERROR(ENOMEM);
                rocdec_decode_end(avctx);
                return ret;
            }
        } else {
            ret = av_hwdevice_ctx_create(&ctx->hwdevice, AV_HWDEVICE_TYPE_AMD_GPU, ctx->amd_gpu, NULL, 0);
            if (ret < 0) {
                rocdec_decode_end(avctx);
                return ret;
            }
        }

        ctx->hwframe = av_hwframe_ctx_alloc(ctx->hwdevice);
        if (!ctx->hwframe) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_ctx_alloc failed\n");
            ret = AVERROR(ENOMEM);
            rocdec_decode_end(avctx);
            return ret;
        }

        hwframe_ctx = (AVHWFramesContext*)ctx->hwframe->data;
    }

    device_ctx = hwframe_ctx->device_ctx;
    device_hwctx = device_ctx->hwctx;

    memset(&ctx->rocdec_parse_info, 0, sizeof(ctx->rocdec_parse_info));
    memset(&seq_pkt, 0, sizeof(seq_pkt));

    switch (avctx->codec->id) {
#if CONFIG_H264_ROCDEC_DECODER
    case AV_CODEC_ID_H264:
        ctx->rocdec_parse_info.codec_type = rocDecVideoCodec_AVC;
        break;
#endif
#if CONFIG_HEVC_ROCDEC_DECODER
    case AV_CODEC_ID_HEVC:
    ctx->rocdec_parse_info.codec_type = rocDecVideoCodec_HEVC;
        break;
#endif
#if CONFIG_VP9_ROCDEC_DECODER
    case AV_CODEC_ID_VP9:
    ctx->rocdec_parse_info.codec_type = rocDecVideoCodec_VP9;
        break;
#endif
#if CONFIG_AV1_ROCDEC_DECODER
    case AV_CODEC_ID_AV1:
        ctx->rocdec_parse_info.codec_type = rocDecVideoCodec_AV1;
        break;
#endif
    default:
        av_log(avctx, AV_LOG_ERROR, "Invalid rocDecode codec!\n");
        return AVERROR_BUG;
    }

    if (ffcodec(avctx->codec)->bsfs) {
        const AVCodecParameters *par = avctx->internal->bsf->par_out;
        extradata = par->extradata;
        extradata_size = par->extradata_size;
    } else {
        extradata = avctx->extradata;
        extradata_size = avctx->extradata_size;
    }

    // TODO: check if this is true for us
    // Check first bit to determine whether it's AV1CodecConfigurationRecord.
    // Skip first 4 bytes of AV1CodecConfigurationRecord to keep configOBUs
    // only, otherwise rocDecParseVideoData report unknown error.
    if (avctx->codec->id == AV_CODEC_ID_AV1 &&
            extradata_size > 4 &&
            extradata[0] & 0x80) {
        extradata += 4;
        extradata_size -= 4;
    }

    ctx->rocdec_parse_ext = av_mallocz(sizeof(*ctx->rocdec_parse_ext)
            + FFMAX(extradata_size - (int)sizeof(ctx->rocdec_parse_ext->raw_seqhdr_data), 0));
    if (!ctx->rocdec_parse_ext) {
        ret = AVERROR(ENOMEM);
        rocdec_decode_end(avctx);
        return ret;
    }

    if (extradata_size > 0)
        memcpy(ctx->rocdec_parse_ext->raw_seqhdr_data, extradata, extradata_size);
    ctx->rocdec_parse_ext->format.seqhdr_data_length = extradata_size;

    ctx->rocdec_parse_info.ext_video_info = ctx->rocdec_parse_ext;

    ctx->key_frame = av_mallocz(ctx->nb_surfaces * sizeof(int));
    if (!ctx->key_frame) {
        ret = AVERROR(ENOMEM);
        rocdec_decode_end(avctx);
        return ret;
    }

    ctx->rocdec_parse_info.max_num_decode_surfaces = 1;
    ctx->rocdec_parse_info.max_display_delay = (avctx->flags & AV_CODEC_FLAG_LOW_DELAY) ? 0 : ROCDEC_MAX_DISPLAY_DELAY;
    ctx->rocdec_parse_info.user_data = avctx;
    ctx->rocdec_parse_info.pfn_sequence_callback = rocdec_handle_video_sequence;
    ctx->rocdec_parse_info.pfn_decode_picture = rocdec_handle_picture_decode;
    ctx->rocdec_parse_info.pfn_display_picture = rocdec_handle_picture_display;

    ret = rocdec_test_capabilities(avctx, &ctx->rocdec_parse_info,
                                  probed_width,
                                  probed_height,
                                  probed_bit_depth, is_yuv422, is_yuv444);
    if (ret < 0) {
        rocdec_decode_end(avctx);
        return ret;
    }

    ret = CHECK_ROCDECODE(rocDecCreateVideoParser(&ctx->rocdecparser, &ctx->rocdec_parse_info));
    if (ret < 0){
        rocdec_decode_end(avctx);
        return ret;
    }

    seq_pkt.payload = ctx->rocdec_parse_ext->raw_seqhdr_data;
    seq_pkt.payload_size = ctx->rocdec_parse_ext->format.seqhdr_data_length;

    if (seq_pkt.payload && seq_pkt.payload_size) {
        ret = CHECK_ROCDECODE(rocDecParseVideoData(ctx->rocdecparser, &seq_pkt));
        if (ret < 0){
            rocdec_decode_end(avctx);
            return ret;
        }
    }

    ctx->prev_pts = INT64_MIN;

    if (!avctx->pkt_timebase.num || !avctx->pkt_timebase.den)
        av_log(avctx, AV_LOG_WARNING, "Invalid pkt_timebase, passing timestamps as-is.\n");

    return 0;
}

static void rocdec_flush(AVCodecContext *avctx)
{
    RocdecContext *ctx = avctx->priv_data;
    AVHWDeviceContext *device_ctx = (AVHWDeviceContext*)ctx->hwdevice->data;
    AVRocDecodeDeviceContext *device_hwctx = device_ctx->hwctx;
    RocdecSourceDataPacket seq_pkt = { 0 };
    int ret;

    av_fifo_reset2(ctx->frame_queue);

    if (ctx->rocdecdecoder) {
        rocDecDestroyDecoder(ctx->rocdecdecoder);
        ctx->rocdecdecoder = NULL;
    }

    if (ctx->rocdecparser) {
        rocDecDestroyVideoParser(ctx->rocdecparser);
        ctx->rocdecparser = NULL;
    }

    ret = CHECK_ROCDECODE(rocDecCreateVideoParser(&ctx->rocdecparser, &ctx->rocdec_parse_info));
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "rocDecode reinit on flush failed\n");

    seq_pkt.payload = ctx->rocdec_parse_ext->raw_seqhdr_data;
    seq_pkt.payload_size = ctx->rocdec_parse_ext->format.seqhdr_data_length;

    if (seq_pkt.payload && seq_pkt.payload_size) {
        ret = CHECK_ROCDECODE(rocDecParseVideoData(ctx->rocdecparser, &seq_pkt));
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "rocDecode reinit on flush failed\n");
    }

    ctx->prev_pts = INT64_MIN;
    ctx->decoder_flushing = 0;

    return;
}


#define OFFSET(x) offsetof(RocdecContext, x)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    { "gpu",      "GPU to be used for decoding", OFFSET(amd_gpu), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { "surfaces", "Maximum surfaces to be used for decoding", OFFSET(nb_surfaces), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, INT_MAX, VD | AV_OPT_FLAG_DEPRECATED },
    { "crop",     "Crop (top)x(bottom)x(left)x(right)", OFFSET(crop_expr), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { "resize",   "Resize (width)x(height)", OFFSET(resize_expr), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, VD },
    { NULL }
};

static const AVCodecHWConfigInternal *const rocdec_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_AMD_GPU,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_INTERNAL,
            .device_type = AV_HWDEVICE_TYPE_AMD_GPU
        },
        .hwaccel = NULL,
    },
    NULL
};

#define DEFINE_ROCDEC_CODEC(x, X, bsf_name) \
    static const AVClass x##_rocdec_class = { \
        .class_name = #x "_rocdec", \
        .item_name = av_default_item_name, \
        .option = options, \
        .version = LIBAVUTIL_VERSION_INT, \
    }; \
    const FFCodec ff_##x##_rocdec_decoder = { \
        .p.name         = #x "_rocdec", \
        CODEC_LONG_NAME("AMD ROCDEC " #X " decoder"), \
        .p.type         = AVMEDIA_TYPE_VIDEO, \
        .p.id           = AV_CODEC_ID_##X, \
        .priv_data_size = sizeof(RocdecContext), \
        .p.priv_class   = &x##_rocdec_class, \
        .init           = rocdec_decode_init, \
        .close          = rocdec_decode_end, \
        FF_CODEC_RECEIVE_FRAME_CB(rocdec_output_frame), \
        .flush          = rocdec_flush, \
        .bsfs           = bsf_name, \
        .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE | \
                          FF_CODEC_CAP_SETS_FRAME_PROPS, \
        .hw_configs     = rocdec_hw_configs, \
        .p.wrapper_name = "rocdec", \
    };

#if CONFIG_AV1_ROCDEC_DECODER
    DEFINE_ROCDEC_CODEC(av1, AV1, NULL)
#endif

#if CONFIG_HEVC_ROCDEC_DECODER
    DEFINE_ROCDEC_CODEC(hevc, HEVC, "hevc_mp4toannexb")
#endif

#if CONFIG_H264_ROCDEC_DECODER
    DEFINE_ROCDEC_CODEC(h264, H264, "h264_mp4toannexb")
#endif

#if CONFIG_VP9_ROCDEC_DECODER
    DEFINE_ROCDEC_CODEC(vp9, VP9, NULL)
#endif

