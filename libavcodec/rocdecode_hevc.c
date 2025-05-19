/*
* HEVC HW decode acceleration through ROCDECODE
*
* Copyright (c) 2025
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

#include <stdint.h>
#include <string.h>

#include "libavutil/mem.h"
#include "avcodec.h"
#include "rocdecode_plugin.h"
#include "decode.h"
#include "internal.h"
#include "hevc/hevcdec.h"
#include "hevc/data.h"
#include "hwaccel_internal.h"

static void dpb_add(RocdecHevcPicParams *pp, int idx, const HEVCFrame *src)
{
    FrameDecodeData *fdd = src->f->private_ref;
    const RocDecFrame *cf = fdd->hwaccel_priv;

    // TODO -- equivalent of RefPicIdx, PicOrderCntVal & IsLongTerm?
    /*pp->RefPicIdx[idx]      = cf ? cf->idx : -1;
    pp->PicOrderCntVal[idx] = src->poc;
    pp->IsLongTerm[idx]     = !!(src->flags & HEVC_FRAME_FLAG_LONG_REF);
    */
}

static void fill_scaling_lists(RocdecPicParams *ppc, const HEVCContext *s)
{
    const ScalingList *sl = s->pps->scaling_list_data_present_flag ?
                            &s->pps->scaling_list : &s->pps->sps->scaling_list;
    int i, j, pos;
    RocdecHevcIQMatrix hevc_iq_matrix = ppc->iq_matrix.hevc;

    for (i = 0; i < 6; i++) {
        for (j = 0; j < 16; j++) {
            //pos = 4 * ff_hevc_diag_scan4x4_y[j] + ff_hevc_diag_scan4x4_x[j];
            hevc_iq_matrix.scaling_list_4x4[i][j] = sl->sl[0][i][j];
        }

        for (j = 0; j < 64; j++) {
            //pos = 8 * ff_hevc_diag_scan8x8_y[j] + ff_hevc_diag_scan8x8_x[j];
            hevc_iq_matrix.scaling_list_8x8[i][j]   = sl->sl[1][i][j];
            hevc_iq_matrix.scaling_list_16x16[i][j] = sl->sl[2][i][j];

            if (i < 2)
            hevc_iq_matrix.scaling_list_32x32[i][j] = sl->sl[3][i * 3][j];
        }

        hevc_iq_matrix.scaling_list_dc_16x16[i] = sl->sl_dc[0][i];
        if (i < 2)
            hevc_iq_matrix.scaling_list_dc_32x32[i] = sl->sl_dc[1][i * 3];
    }
}

static int rocdec_hevc_start_frame(AVCodecContext *avctx,
                                const AVBufferRef *buffer_ref,
                                const uint8_t *buffer, uint32_t size)
{
    const HEVCContext *s = avctx->priv_data;
    const HEVCLayerContext *hevc_layer = &s->layers[s->cur_layer];
    const HEVCPPS *pps = s->pps;
    const HEVCSPS *sps = pps->sps;

    RocDecContext       *ctx = avctx->internal->hwaccel_priv_data;
    RocdecPicParams      *pp = &ctx->pic_params;
    RocdecHevcPicParams *ppc = &pp->pic_params.hevc;
    FrameDecodeData *fdd;
    RocDecFrame *cf;

    int i, j, dpb_size, ret;

    ret = ff_rocdec_start_frame(avctx, s->cur_frame->f);
    if (ret < 0)
        return ret;

    fdd = s->cur_frame->f->private_ref;
    cf  = (RocDecFrame*)fdd->hwaccel_priv;

    *pp = (RocdecPicParams) {
        .pic_width     = sps->width  / 16,
        .pic_height    = sps->height / 16,
        .curr_pic_idx        = cf->idx,
        .ref_pic_flag      = 1,
        .intra_pic_flag    = IS_IRAP(s),

        .pic_params.hevc = {
            .picture_width_in_luma_samples                  = sps->width,
            .picture_height_in_luma_samples                 = sps->height,
            .log2_min_luma_coding_block_size_minus3         = sps->log2_min_cb_size - 3,
            .log2_diff_max_min_luma_coding_block_size       = sps->log2_diff_max_min_coding_block_size,
            .log2_min_luma_transform_block_size_minus2      = sps->log2_min_tb_size - 2,
            .log2_diff_max_min_luma_transform_block_size    = sps->log2_max_trafo_size - sps->log2_min_tb_size,
            .log2_min_pcm_luma_coding_block_size_minus3     = sps->pcm_enabled ? sps->pcm.log2_min_pcm_cb_size - 3 : 0,
            .log2_diff_max_min_pcm_luma_coding_block_size   = sps->pcm.log2_max_pcm_cb_size - sps->pcm.log2_min_pcm_cb_size,
            .pcm_sample_bit_depth_luma_minus1               = sps->pcm_enabled ? sps->pcm.bit_depth - 1 : 0,
            .pcm_sample_bit_depth_chroma_minus1             = sps->pcm_enabled ? sps->pcm.bit_depth_chroma - 1 : 0,
            .max_transform_hierarchy_depth_intra            = sps->max_transform_hierarchy_depth_intra,
            .max_transform_hierarchy_depth_inter            = sps->max_transform_hierarchy_depth_inter,
            .log2_max_pic_order_cnt_lsb_minus4              = sps->log2_max_poc_lsb - 4,
            .num_short_term_ref_pic_sets                    = sps->nb_st_rps,
            .num_long_term_ref_pic_sps                      = sps->num_long_term_ref_pics_sps,
            .bit_depth_luma_minus8                          = sps->bit_depth - 8,
            .bit_depth_chroma_minus8                        = sps->bit_depth - 8,
            .diff_cu_qp_delta_depth                         = pps->diff_cu_qp_delta_depth,
            .init_qp_minus26                                = pps->pic_init_qp_minus26,
            .pps_cb_qp_offset                               = pps->cb_qp_offset,
            .pps_cr_qp_offset                               = pps->cr_qp_offset,
            .log2_parallel_merge_level_minus2               = pps->log2_parallel_merge_level - 2,
            .num_extra_slice_header_bits                    = pps->num_extra_slice_header_bits,
            .num_ref_idx_l0_default_active_minus1           = pps->num_ref_idx_l0_default_active - 1,
            .num_ref_idx_l1_default_active_minus1           = pps->num_ref_idx_l1_default_active - 1,
            .pps_beta_offset_div2                           = pps->beta_offset / 2,
            .pps_tc_offset_div2                             = pps->tc_offset / 2,
            .sps_max_dec_pic_buffering_minus1             = sps->temporal_layer[sps->max_sub_layers - 1].max_dec_pic_buffering - 1,
            // TODO -- commented seciton but no variable
            //.uniform_spacing_flag                         = pps->uniform_spacing_flag,
            .num_short_term_ref_pic_sets                    = s->sh.short_term_rps ? s->sh.short_term_ref_pic_set_size : 0,
            .slice_parsing_fields.bits = {
                .long_term_ref_pics_present_flag            = sps->long_term_ref_pics_present,
                .sps_temporal_mvp_enabled_flag              = sps->temporal_mvp_enabled,
                .sample_adaptive_offset_enabled_flag        = sps->sao_enabled,
                .rap_pic_flag                               = IS_IRAP(s),
                .idr_pic_flag                               = IS_IDR(s),
                .intra_pic_flag                             = IS_IRAP(s),
                .dependent_slice_segments_enabled_flag      = pps->dependent_slice_segments_enabled_flag,
                .slice_segment_header_extension_present_flag  = pps->slice_header_extension_present_flag,
                .output_flag_present_flag                   = pps->output_flag_present_flag,
                .lists_modification_present_flag            = pps->lists_modification_present_flag,
                .cabac_init_present_flag                    = pps->cabac_init_present_flag,
                .pps_slice_chroma_qp_offsets_present_flag   = pps->pic_slice_level_chroma_qp_offsets_present_flag,
                .deblocking_filter_override_enabled_flag    = pps->deblocking_filter_override_enabled_flag,
                .pps_disable_deblocking_filter_flag         = pps->disable_dbf,
            },
            .pic_fields.bits = {
                .pcm_enabled_flag                           = sps->pcm_enabled,
                .strong_intra_smoothing_enabled_flag        = sps->strong_intra_smoothing_enabled,
                .pcm_loop_filter_disabled_flag              = sps->pcm_loop_filter_disabled,
                .amp_enabled_flag                           = sps->amp_enabled,
                .separate_colour_plane_flag                 = sps->separate_colour_plane,
                .sign_data_hiding_enabled_flag              = pps->sign_data_hiding_flag,
                .cu_qp_delta_enabled_flag                   = pps->cu_qp_delta_enabled_flag,
                .constrained_intra_pred_flag                = pps->constrained_intra_pred_flag,
                .weighted_pred_flag                         = pps->weighted_pred_flag,
                .weighted_bipred_flag                       = pps->weighted_bipred_flag,
                .transform_skip_enabled_flag                = pps->transform_skip_enabled_flag,
                .transquant_bypass_enabled_flag             = pps->transquant_bypass_enable_flag,
                .entropy_coding_sync_enabled_flag           = pps->entropy_coding_sync_enabled_flag,
                .loop_filter_across_tiles_enabled_flag      = pps->loop_filter_across_tiles_enabled_flag,
                .pps_loop_filter_across_slices_enabled_flag = pps->seq_loop_filter_across_slices_enabled_flag,
                .tiles_enabled_flag                         = pps->tiles_enabled_flag,
                .scaling_list_enabled_flag                  = sps->scaling_list_enabled,
                .chroma_format_idc                          = sps->chroma_format_idc,
            },
            // TODO -- equivalent?
            /*.NumDeltaPocsOfRefRpsIdx                    = s->sh.short_term_rps ? s->sh.short_term_rps->rps_idx_num_delta_pocs : 0,
            .NumPocTotalCurr                              = ff_hevc_frame_nb_refs(&s->sh, pps, s->cur_layer),
            .NumPocStCurrBefore                           = s->rps[ST_CURR_BEF].nb_refs,
            .NumPocStCurrAfter                            = s->rps[ST_CURR_AFT].nb_refs,
            .NumPocLtCurr                                 = s->rps[LT_CURR].nb_refs,
            .CurrPicOrderCntVal                           = s->cur_frame->poc,
            */
        },
    };

    if (pps->tiles_enabled_flag) {
        ppc->num_tile_columns_minus1 = pps->num_tile_columns - 1;
        ppc->num_tile_rows_minus1    = pps->num_tile_rows - 1;

        for (i = 0; i < pps->num_tile_columns; i++)
            ppc->column_width_minus1[i] = pps->column_width[i] - 1;

        for (i = 0; i < pps->num_tile_rows; i++)
            ppc->row_height_minus1[i] = pps->row_height[i] - 1;
    }

    if (s->sh.short_term_ref_pic_set_sps_flag == 0 && s->sh.short_term_rps) {
        ppc->st_rps_bits = s->sh.short_term_ref_pic_set_size;
    } else {
        ppc->st_rps_bits = 0;
    }

    fill_scaling_lists(ppc, s);


/* TODO - find equivalent for these
    if (s->rps[LT_CURR].nb_refs     > FF_ARRAY_ELEMS(ppc->RefPicSetLtCurr)       ||
        s->rps[ST_CURR_BEF].nb_refs > FF_ARRAY_ELEMS(ppc->RefPicSetStCurrBefore) ||
        s->rps[ST_CURR_AFT].nb_refs > FF_ARRAY_ELEMS(ppc->RefPicSetStCurrAfter)) {
        av_log(avctx, AV_LOG_ERROR, "Too many reference frames\n");
        return AVERROR(ENOSYS);
    }
*/


            /* TODO - find equivalent for these
    /*dpb_size = 0;
    for (i = 0; i < FF_ARRAY_ELEMS(hevc_layer->DPB); i++) {
        const HEVCFrame *ref = &hevc_layer->DPB[i];
        if (!(ref->flags & (HEVC_FRAME_FLAG_SHORT_REF | HEVC_FRAME_FLAG_LONG_REF)))
            continue;

        if (dpb_size >= FF_ARRAY_ELEMS(ppc->RefPicIdx)) {
            av_log(avctx, AV_LOG_ERROR, "Too many reference frames\n");
            return AVERROR_INVALIDDATA;
        }

        dpb_add(ppc, dpb_size++, ref);

    }

    for (i = dpb_size; i < FF_ARRAY_ELEMS(ppc->RefPicIdx); i++)
        ppc->RefPicIdx[i] = -1;

    for (i = 0; i < s->rps[ST_CURR_BEF].nb_refs; i++) {
        for (j = 0; j < dpb_size; j++) {
            if (ppc->PicOrderCntVal[j] == s->rps[ST_CURR_BEF].list[i]) {
                ppc->RefPicSetStCurrBefore[i] = j;
                break;
            }
        }
    }
    for (i = 0; i < s->rps[ST_CURR_AFT].nb_refs; i++) {
        for (j = 0; j < dpb_size; j++) {
            if (ppc->PicOrderCntVal[j] == s->rps[ST_CURR_AFT].list[i]) {
                ppc->RefPicSetStCurrAfter[i] = j;
                break;
            }
        }
    }
    for (i = 0; i < s->rps[LT_CURR].nb_refs; i++) {
        for (j = 0; j < dpb_size; j++) {
            if (ppc->PicOrderCntVal[j] == s->rps[LT_CURR].list[i]) {
                ppc->RefPicSetLtCurr[i] = j;
                break;
            }
        }
    }
    */

    return 0;
}

static void fill_pred_weight_table(AVCodecContext *avctx,
    const HEVCContext *h,
    const SliceHeader *sh,
    RocdecHevcSliceParams *slice_param)
{
    int i;

    memset(slice_param->delta_luma_weight_l0,   0, sizeof(slice_param->delta_luma_weight_l0));
    memset(slice_param->delta_luma_weight_l1,   0, sizeof(slice_param->delta_luma_weight_l1));
    memset(slice_param->luma_offset_l0,         0, sizeof(slice_param->luma_offset_l0));
    memset(slice_param->luma_offset_l1,         0, sizeof(slice_param->luma_offset_l1));
    memset(slice_param->delta_chroma_weight_l0, 0, sizeof(slice_param->delta_chroma_weight_l0));
    memset(slice_param->delta_chroma_weight_l1, 0, sizeof(slice_param->delta_chroma_weight_l1));
    memset(slice_param->chroma_offset_l0,       0, sizeof(slice_param->chroma_offset_l0));
    memset(slice_param->chroma_offset_l1,       0, sizeof(slice_param->chroma_offset_l1));

    slice_param->delta_chroma_log2_weight_denom = 0;
    slice_param->luma_log2_weight_denom         = 0;

    if (sh->slice_type == HEVC_SLICE_I ||
        (sh->slice_type == HEVC_SLICE_P && !h->pps->weighted_pred_flag) ||
        (sh->slice_type == HEVC_SLICE_B && !h->pps->weighted_bipred_flag))
        return;

    slice_param->luma_log2_weight_denom = sh->luma_log2_weight_denom;

    if (h->pps->sps->chroma_format_idc) {
        slice_param->delta_chroma_log2_weight_denom = sh->chroma_log2_weight_denom - sh->luma_log2_weight_denom;
    }

    for (i = 0; i < 15 && i < sh->nb_refs[L0]; i++) {
        slice_param->delta_luma_weight_l0[i] = sh->luma_weight_l0[i] - (1 << sh->luma_log2_weight_denom);
        slice_param->delta_chroma_weight_l0[i][0] = sh->chroma_weight_l0[i][0] - (1 << sh->chroma_log2_weight_denom);
        slice_param->delta_chroma_weight_l0[i][1] = sh->chroma_weight_l0[i][1] - (1 << sh->chroma_log2_weight_denom);
    }

    if (sh->slice_type == HEVC_SLICE_B) {
        for (i = 0; i < 15 && i < sh->nb_refs[L1]; i++) {
            slice_param->delta_luma_weight_l1[i] = sh->luma_weight_l1[i] - (1 << sh->luma_log2_weight_denom);
            slice_param->delta_chroma_weight_l1[i][0] = sh->chroma_weight_l1[i][0] - (1 << sh->chroma_log2_weight_denom);
            slice_param->delta_chroma_weight_l1[i][1] = sh->chroma_weight_l1[i][1] - (1 << sh->chroma_log2_weight_denom);
        }
    }
}

static int rocdec_hevc_decode_slice(AVCodecContext *avctx, const uint8_t *buffer,
                                uint32_t size)
{
    const HEVCContext        *h = avctx->priv_data;
    const SliceHeader       *sh = &h->sh;
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    RocdecPicParams      *pp = &ctx->pic_params;
    RocdecHevcSliceParams *last_slice_param = &pp->slice_params.hevc;
    int err, i, list_idx;

    int nb_list = (sh->slice_type == HEVC_SLICE_B) ?
                  2 : (sh->slice_type == HEVC_SLICE_I ? 0 : 1);

    void *tmp;

    tmp = av_fast_realloc(ctx->bitstream_internal, &ctx->bitstream_allocated,
                        ctx->bitstream_len + size + 3);
    if (!tmp)
        return AVERROR(ENOMEM);
    ctx->bitstream = ctx->bitstream_internal = tmp;

    tmp = av_fast_realloc(ctx->slice_offsets, &ctx->slice_offsets_allocated,
                        (ctx->nb_slices + 1) * sizeof(*ctx->slice_offsets));
    if (!tmp)
        return AVERROR(ENOMEM);
    ctx->slice_offsets = tmp;


    *last_slice_param = (RocdecHevcSliceParams) {
        .slice_data_size               = size,
        .slice_data_offset             = 0,
        .slice_data_flag               = 0x00, //VA_SLICE_DATA_FLAG_ALL
        .slice_data_byte_offset        = sh->data_offset,
        .slice_segment_address         = sh->slice_segment_addr,
        .slice_qp_delta                = sh->slice_qp_delta,
        .slice_cb_qp_offset            = sh->slice_cb_qp_offset,
        .slice_cr_qp_offset            = sh->slice_cr_qp_offset,
        .slice_beta_offset_div2        = sh->beta_offset / 2,
        .slice_tc_offset_div2          = sh->tc_offset / 2,
        .collocated_ref_idx            = sh->slice_temporal_mvp_enabled_flag ? sh->collocated_ref_idx : 0xFF,
        .five_minus_max_num_merge_cand = sh->slice_type == HEVC_SLICE_I ? 0 : 5 - sh->max_num_merge_cand,
        .num_ref_idx_l0_active_minus1  = sh->nb_refs[L0] ? sh->nb_refs[L0] - 1 : 0,
        .num_ref_idx_l1_active_minus1  = sh->nb_refs[L1] ? sh->nb_refs[L1] - 1 : 0,

        .long_slice_flags.fields = {
            .dependent_slice_segment_flag                 = sh->dependent_slice_segment_flag,
            .slice_type                                   = sh->slice_type,
            .color_plane_id                               = sh->colour_plane_id,
            .mvd_l1_zero_flag                             = sh->mvd_l1_zero_flag,
            .cabac_init_flag                              = sh->cabac_init_flag,
            .slice_temporal_mvp_enabled_flag              = sh->slice_temporal_mvp_enabled_flag,
            .slice_deblocking_filter_disabled_flag        = sh->disable_deblocking_filter_flag,
            .collocated_from_l0_flag                      = sh->collocated_list == L0 ? 1 : 0,
            .slice_loop_filter_across_slices_enabled_flag = sh->slice_loop_filter_across_slices_enabled_flag,
            .slice_sao_luma_flag                          = sh->slice_sample_adaptive_offset_flag[0],
            .slice_sao_chroma_flag                        = sh->slice_sample_adaptive_offset_flag[1],
        },
    };

    fill_pred_weight_table(avctx, h, sh, last_slice_param);
    AV_WB24(ctx->bitstream_internal + ctx->bitstream_len, 1);
    memcpy(ctx->bitstream_internal + ctx->bitstream_len + 3, buffer, size);
    ctx->slice_offsets[ctx->nb_slices] = ctx->bitstream_len ;
    ctx->bitstream_len += size + 3;
    ctx->nb_slices++;

    return 0;
}

static int rocdec_hevc_frame_params(AVCodecContext *avctx,
                                AVBufferRef *hw_frames_ctx)
{
    const HEVCContext *s = avctx->priv_data;
    const HEVCSPS *sps = s->pps->sps;
    return ff_rocdec_frame_params(avctx, hw_frames_ctx, sps->temporal_layer[sps->max_sub_layers - 1].max_dec_pic_buffering + 1, 1);
}

static int rocdec_hevc_decode_init(AVCodecContext *avctx) {
    RocDecContext *ctx = avctx->internal->hwaccel_priv_data;
    ctx->supports_444 = 1;

    if (avctx->profile != AV_PROFILE_HEVC_MAIN &&
        avctx->profile != AV_PROFILE_HEVC_MAIN_10 &&
        avctx->profile != AV_PROFILE_HEVC_MAIN_STILL_PICTURE) {
        av_log(avctx, AV_LOG_ERROR, "Unsupported HEVC profile: %d\n", avctx->profile);
        return AVERROR(ENOTSUP);
    }
    av_log(avctx, AV_LOG_VERBOSE, "rocdec_hevc_decode_init: HEVC profile: %d\n", avctx->profile);
    return ff_rocdec_decode_init(avctx);
}

const FFHWAccel ff_hevc_rocdec_hwaccel = {
    .p.name               = "hevc_rocdec",
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_HEVC,
    .p.pix_fmt            = AV_PIX_FMT_HIP,
    .start_frame          = rocdec_hevc_start_frame,
    .end_frame            = ff_rocdec_end_frame,
    .decode_slice         = rocdec_hevc_decode_slice,
    .frame_params         = rocdec_hevc_frame_params,
    .init                 = rocdec_hevc_decode_init,
    .uninit               = ff_rocdec_decode_uninit,
    .priv_data_size       = sizeof(RocDecContext),
};
 