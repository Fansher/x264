/*****************************************************************************
 * x264: h264 encoder
 *****************************************************************************
 * Copyright (C) 2003 Laurent Aimar
 * $Id: encoder.c,v 1.1 2004/06/03 19:27:08 fenrir Exp $
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111, USA.
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <math.h>

#include "common/common.h"
#include "common/cpu.h"

#include "set.h"
#include "analyse.h"
#include "ratecontrol.h"
#include "macroblock.h"

//#define DEBUG_MB_TYPE
//#define DEBUG_DUMP_FRAME
//#define DEBUG_BENCHMARK

#ifdef DEBUG_BENCHMARK
static int64_t i_mtime_encode_frame = 0;
static int64_t i_mtime_analyse = 0;
static int64_t i_mtime_encode = 0;
static int64_t i_mtime_write = 0;
static int64_t i_mtime_filter = 0;
#define TIMER_START( d ) \
    { \
        int64_t d##start = x264_mdate();

#define TIMER_STOP( d ) \
        d += x264_mdate() - d##start;\
    }
#else
#define TIMER_START( d )
#define TIMER_STOP( d )
#endif


/****************************************************************************
 *
 ******************************* x264 libs **********************************
 *
 ****************************************************************************/
static int64_t x264_sqe( uint8_t *pix1, int i_pix_stride, uint8_t *pix2, int i_pix2_stride, int i_width, int i_height )
{
    int64_t i_sqe = 0;

    int x, y;

    for( y = 0; y < i_height; y++ )
    {
        for( x = 0; x < i_width; x++ )
        {
            int tmp;

            tmp = pix1[y*i_pix_stride+x] - pix2[y*i_pix2_stride+x];

            i_sqe += tmp * tmp;
        }
    }
    return i_sqe;
}

static float x264_mse( int64_t i_sqe, int64_t i_size )
{
    return (double)i_sqe / ((double)65025.0 * (double)i_size);
}

static float x264_psnr( int64_t i_sqe, int64_t i_size )
{
    double f_mse = (double)i_sqe / ((double)65025.0 * (double)i_size);
    if( f_mse <= 0.0000000001 ) /* Max 100dB */
        return 100;

    return (float)(-10.0 * log( f_mse ) / log( 10.0 ));
}

#ifdef DEBUG_DUMP_FRAME
static void x264_frame_dump( x264_t *h, x264_frame_t *fr, char *name )
{
    FILE * f = fopen( name, "a" );
    int i, y;

    fseek( f, 0, SEEK_END );

    for( i = 0; i < fr->i_plane; i++ )
    {
        for( y = 0; y < h->param.i_height / ( i == 0 ? 1 : 2 ); y++ )
        {
            fwrite( &fr->plane[i][y*fr->i_stride[i]], 1, h->param.i_width / ( i == 0 ? 1 : 2 ), f );
        }
    }
    fclose( f );
}
#endif


/* Fill "default" values */
static void x264_slice_header_init( x264_slice_header_t *sh, x264_param_t *param,
                                    x264_sps_t *sps, x264_pps_t *pps,
                                    int i_type, int i_idr_pic_id, int i_frame )
{
    /* First we fill all field */
    sh->sps = sps;
    sh->pps = pps;

    sh->i_type      = i_type;
    sh->i_first_mb  = 0;
    sh->i_pps_id    = pps->i_id;

    sh->i_frame_num = i_frame;

    sh->b_field_pic = 0;    /* Not field support for now */
    sh->b_bottom_field = 1; /* not yet used */

    sh->i_idr_pic_id = i_idr_pic_id;

    /* poc stuff, fixed later */
    sh->i_poc_lsb = 0;
    sh->i_delta_poc_bottom = 0;
    sh->i_delta_poc[0] = 0;
    sh->i_delta_poc[1] = 0;

    sh->i_redundant_pic_cnt = 0;

    sh->b_direct_spatial_mv_pred = ( param->analyse.i_direct_mv_pred == X264_DIRECT_PRED_SPATIAL );

    sh->b_num_ref_idx_override = 0;
    sh->i_num_ref_idx_l0_active = 1;
    sh->i_num_ref_idx_l1_active = 1;

    sh->i_cabac_init_idc = param->i_cabac_init_idc;

    sh->i_qp_delta = 0;
    sh->b_sp_for_swidth = 0;
    sh->i_qs_delta = 0;

    if( param->b_deblocking_filter )
    {
        sh->i_disable_deblocking_filter_idc = 0;
    }
    else
    {
        sh->i_disable_deblocking_filter_idc = 1;
    }
    sh->i_alpha_c0_offset = param->i_deblocking_filter_alphac0 << 1;
    sh->i_beta_offset = param->i_deblocking_filter_beta << 1;
}

static void x264_slice_header_write( bs_t *s, x264_slice_header_t *sh, int i_nal_ref_idc )
{
    bs_write_ue( s, sh->i_first_mb );
    bs_write_ue( s, sh->i_type + 5 );   /* same type things */
    bs_write_ue( s, sh->i_pps_id );
    bs_write( s, sh->sps->i_log2_max_frame_num, sh->i_frame_num );

    if( sh->i_idr_pic_id >= 0 ) /* NAL IDR */
    {
        bs_write_ue( s, sh->i_idr_pic_id );
    }

    if( sh->sps->i_poc_type == 0 )
    {
        bs_write( s, sh->sps->i_log2_max_poc_lsb, sh->i_poc_lsb );
        if( sh->pps->b_pic_order && !sh->b_field_pic )
        {
            bs_write_se( s, sh->i_delta_poc_bottom );
        }
    }
    else if( sh->sps->i_poc_type == 1 && !sh->sps->b_delta_pic_order_always_zero )
    {
        bs_write_se( s, sh->i_delta_poc[0] );
        if( sh->pps->b_pic_order && !sh->b_field_pic )
        {
            bs_write_se( s, sh->i_delta_poc[1] );
        }
    }

    if( sh->pps->b_redundant_pic_cnt )
    {
        bs_write_ue( s, sh->i_redundant_pic_cnt );
    }

    if( sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_direct_spatial_mv_pred );
    }
    if( sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_SP || sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_num_ref_idx_override );
        if( sh->b_num_ref_idx_override )
        {
            bs_write_ue( s, sh->i_num_ref_idx_l0_active - 1 );
            if( sh->i_type == SLICE_TYPE_B )
            {
                bs_write_ue( s, sh->i_num_ref_idx_l1_active - 1 );
            }
        }
    }

    /* ref pic list reordering */
    if( sh->i_type != SLICE_TYPE_I )
    {
        int b_ref_pic_list_reordering_l0 = 0;
        bs_write1( s, b_ref_pic_list_reordering_l0 );
        if( b_ref_pic_list_reordering_l0 )
        {
            /* FIXME */
        }
    }
    if( sh->i_type == SLICE_TYPE_B )
    {
        int b_ref_pic_list_reordering_l1 = 0;
        bs_write1( s, b_ref_pic_list_reordering_l1 );
        if( b_ref_pic_list_reordering_l1 )
        {
            /* FIXME */
        }
    }

    if( ( sh->pps->b_weighted_pred && ( sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_SP ) ) ||
        ( sh->pps->b_weighted_bipred == 1 && sh->i_type == SLICE_TYPE_B ) )
    {
        /* FIXME */
    }

    if( i_nal_ref_idc != 0 )
    {
        if( sh->i_idr_pic_id >= 0 )
        {
            bs_write1( s, 0 );  /* no output of prior pics flag */
            bs_write1( s, 0 );  /* long term reference flag */
        }
        else
        {
            bs_write1( s, 0 );  /* adaptive_ref_pic_marking_mode_flag */
            /* FIXME */
        }
    }

    if( sh->pps->b_cabac && sh->i_type != SLICE_TYPE_I )
    {
        bs_write_ue( s, sh->i_cabac_init_idc );
    }
    bs_write_se( s, sh->i_qp_delta );      /* slice qp delta */
#if 0
    if( sh->i_type == SLICE_TYPE_SP || sh->i_type == SLICE_TYPE_SI )
    {
        if( sh->i_type == SLICE_TYPE_SP )
        {
            bs_write1( s, sh->b_sp_for_swidth );
        }
        bs_write_se( s, sh->i_qs_delta );
    }
#endif

    if( sh->pps->b_deblocking_filter_control )
    {
        bs_write_ue( s, sh->i_disable_deblocking_filter_idc );
        if( sh->i_disable_deblocking_filter_idc != 1 )
        {
            bs_write_se( s, sh->i_alpha_c0_offset >> 1 );
            bs_write_se( s, sh->i_beta_offset >> 1 );
        }
    }
}

/****************************************************************************
 *
 ****************************************************************************
 ****************************** External API*********************************
 ****************************************************************************
 *
 ****************************************************************************/

/****************************************************************************
 * x264_encoder_open:
 ****************************************************************************/
x264_t *x264_encoder_open   ( x264_param_t *param )
{
    x264_t *h = x264_malloc( sizeof( x264_t ) );
    int i, i_slice;

    /* Create a copy of param */
    memcpy( &h->param, param, sizeof( x264_param_t ) );
    if( h->param.rc.psz_stat_out )
        h->param.rc.psz_stat_out = strdup( h->param.rc.psz_stat_out );
    if( h->param.rc.psz_stat_in )
        h->param.rc.psz_stat_in = strdup( h->param.rc.psz_stat_in );
    if( h->param.rc.psz_rc_eq )
        h->param.rc.psz_rc_eq = strdup( h->param.rc.psz_rc_eq );

    /* Check parameters validity */
    if( param->i_width <= 0  || param->i_height <= 0 )
    {
        x264_log( h, X264_LOG_ERROR, "invalid width x height (%dx%d)\n",
                  param->i_width, param->i_height );
        free( h );
        return NULL;
    }

    if( param->i_width % 16 != 0 || param->i_height % 16 != 0 )
    {
        x264_log( h, X264_LOG_ERROR, "width %% 16 != 0 or height %% 16 != 0 (%dx%d)\n",
                 param->i_width, param->i_height );
        free( h );
        return NULL;
    }
    if( param->i_csp != X264_CSP_I420 )
    {
        x264_log( h, X264_LOG_ERROR, "invalid CSP (only I420 supported)\n" );
        free( h );
        return NULL;
    }

    /* Fix parameters values */
    h->param.i_frame_reference = x264_clip3( h->param.i_frame_reference, 1, 15 );
    if( h->param.i_keyint_max <= 0 )
        h->param.i_keyint_max = 1;
    h->param.i_keyint_min = x264_clip3( h->param.i_keyint_min, 1, h->param.i_keyint_max/2+1 );
    h->param.i_bframe = x264_clip3( h->param.i_bframe, 0, X264_BFRAME_MAX );

    h->param.i_deblocking_filter_alphac0 = x264_clip3( h->param.i_deblocking_filter_alphac0, -6, 6 );
    h->param.i_deblocking_filter_beta    = x264_clip3( h->param.i_deblocking_filter_beta, -6, 6 );

    h->param.i_cabac_init_idc = x264_clip3( h->param.i_cabac_init_idc, -1, 2 );

    h->param.analyse.i_subpel_refine = x264_clip3( h->param.analyse.i_subpel_refine, 1, 5 );
    if( h->param.analyse.inter & X264_ANALYSE_PSUB8x8 )
        h->param.analyse.inter |= X264_ANALYSE_PSUB16x16;

    if( h->param.rc.f_qblur < 0 )
        h->param.rc.f_qblur = 0;
    if( h->param.rc.f_complexity_blur < 0 )
        h->param.rc.f_complexity_blur = 0;

    /* VUI */
    if( h->param.vui.i_sar_width > 0 && h->param.vui.i_sar_height > 0 )
    {
        int i_w = param->vui.i_sar_width;
        int i_h = param->vui.i_sar_height;
        int a = i_w, b = i_h;

        while( b != 0 )
        {
            int t = a;

            a = b;
            b = t % b;
        }

        i_w /= a;
        i_h /= a;
        while( i_w > 65535 || i_h > 65535 )
        {
            i_w /= 2;
            i_h /= 2;
        }

        h->param.vui.i_sar_width = 0;
        h->param.vui.i_sar_height = 0;
        if( i_w == 0 || i_h == 0 )
        {
            x264_log( h, X264_LOG_ERROR, "cannot create valid sample aspect ratio\n" );
        }
        else if( i_w == i_h )
        {
            x264_log( h, X264_LOG_INFO, "no need for a SAR\n" );
        }
        else
        {
            x264_log( h, X264_LOG_INFO, "using SAR=%d/%d\n", i_w, i_h );
            h->param.vui.i_sar_width = i_w;
            h->param.vui.i_sar_height = i_h;
        }
    }


    /* Init x264_t */
    h->out.i_nal = 0;
    h->out.i_bitstream = 1000000; /* FIXME estimate max size (idth/height) */
    h->out.p_bitstream = x264_malloc( h->out.i_bitstream );

    h->i_frame = 0;
    h->i_frame_num = 0;
    h->i_poc   = 0;
    h->i_idr_pic_id = 0;

    h->sps = &h->sps_array[0];
    x264_sps_init( h->sps, 0, &h->param );

    h->pps = &h->pps_array[0];
    x264_pps_init( h->pps, 0, &h->param, h->sps);
    
    h->mb.i_mb_count = h->sps->i_mb_width * h->sps->i_mb_height;

    /* Init frames. */
    for( i = 0; i < X264_BFRAME_MAX + 1; i++ )
    {
        h->frames.current[i] = NULL;
        h->frames.next[i]    = NULL;
        h->frames.unused[i]  = NULL;
    }
    for( i = 0; i < 1 + h->param.i_bframe; i++ )
    {
        h->frames.unused[i] =  x264_frame_new( h );
    }
    for( i = 0; i < 2 + h->param.i_frame_reference; i++ )
    {
        /* 2 = 1 backward ref  + 1 fdec */
        h->frames.reference[i] = x264_frame_new( h );
    }
    h->frames.i_last_idr = - h->param.i_keyint_max;
    h->frames.i_input    = 0;

    h->i_ref0 = 0;
    h->i_ref1 = 0;

    h->fdec = h->frames.reference[0];

    /* init mb cache */
    x264_macroblock_cache_init( h );

    /* init cabac adaptive model */
    x264_cabac_model_init( &h->cabac );

    /* init CPU functions */
    x264_predict_16x16_init( h->param.cpu, h->predict_16x16 );
    x264_predict_8x8_init( h->param.cpu, h->predict_8x8 );
    x264_predict_4x4_init( h->param.cpu, h->predict_4x4 );

    x264_pixel_init( h->param.cpu, &h->pixf );
    x264_dct_init( h->param.cpu, &h->dctf );
    x264_mc_init( h->param.cpu, &h->mc );
    x264_csp_init( h->param.cpu, h->param.i_csp, &h->csp );

    /* rate control */
    if( x264_ratecontrol_new( h ) < 0 )
        return NULL;

    h->i_last_intra_size = 0;
    h->i_last_inter_size = 0;

    /* stat */
    for( i_slice = 0; i_slice < 5; i_slice++ )
    {
        h->stat.i_slice_count[i_slice] = 0;
        h->stat.i_slice_size[i_slice] = 0;
        h->stat.i_slice_qp[i_slice] = 0;

        h->stat.i_sqe_global[i_slice] = 0;
        h->stat.f_psnr_average[i_slice] = 0.0;
        h->stat.f_psnr_mean_y[i_slice] = h->stat.f_psnr_mean_u[i_slice] = h->stat.f_psnr_mean_v[i_slice] = 0.0;
        
        for( i = 0; i < 18; i++ )
            h->stat.i_mb_count[i_slice][i] = 0;
    }

    x264_log( h, X264_LOG_INFO, "using cpu capabilities %s%s%s%s%s%s\n",
             param->cpu&X264_CPU_MMX ? "MMX " : "",
             param->cpu&X264_CPU_MMXEXT ? "MMXEXT " : "",
             param->cpu&X264_CPU_SSE ? "SSE " : "",
             param->cpu&X264_CPU_SSE2 ? "SSE2 " : "",
             param->cpu&X264_CPU_3DNOW ? "3DNow! " : "",
             param->cpu&X264_CPU_ALTIVEC ? "Altivec " : "" );

    return h;
}

/* internal usage */
static void x264_nal_start( x264_t *h, int i_type, int i_ref_idc )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];

    nal->i_ref_idc = i_ref_idc;
    nal->i_type    = i_type;

    bs_align_0( &h->out.bs );   /* not needed */

    nal->i_payload= 0;
    nal->p_payload= &h->out.p_bitstream[bs_pos( &h->out.bs) / 8];
}
static void x264_nal_end( x264_t *h )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];

    bs_align_0( &h->out.bs );   /* not needed */

    nal->i_payload = &h->out.p_bitstream[bs_pos( &h->out.bs)/8] - nal->p_payload;

    h->out.i_nal++;
}

/****************************************************************************
 * x264_encoder_headers:
 ****************************************************************************/
int x264_encoder_headers( x264_t *h, x264_nal_t **pp_nal, int *pi_nal )
{
    /* init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    /* Put SPS and PPS */
    if( h->i_frame == 0 )
    {
        /* generate sequence parameters */
        x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
        x264_sps_write( &h->out.bs, h->sps );
        x264_nal_end( h );

        /* generate picture parameters */
        x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
        x264_pps_write( &h->out.bs, h->pps );
        x264_nal_end( h );
    }
    /* now set output*/
    *pi_nal = h->out.i_nal;
    *pp_nal = &h->out.nal[0];

    return 0;
}


static void x264_frame_put( x264_frame_t *list[X264_BFRAME_MAX], x264_frame_t *frame )
{
    int i = 0;

    while( list[i] != NULL ) i++;

    list[i] = frame;
}

static x264_frame_t *x264_frame_get( x264_frame_t *list[X264_BFRAME_MAX+1] )
{
    x264_frame_t *frame = list[0];
    int i;

    for( i = 0; i < X264_BFRAME_MAX; i++ )
    {
        list[i] = list[i+1];
    }
    list[X264_BFRAME_MAX] = NULL;

    return frame;
}

/* Sort queued frames into input order */
static void x264_frame_sort( x264_frame_t *list[X264_BFRAME_MAX+1] )
{
    int i, b_ok;
    do {
        b_ok = 1;
        for( i = 0; i < X264_BFRAME_MAX && list[i+1]; i++ )
        {
            if( list[i]->i_frame > list[i+1]->i_frame )
            {
                x264_frame_t *tmp = list[i+1];
                list[i+1] = list[i];
                list[i] = tmp;
                b_ok = 0;
            }
        }
    } while( !b_ok );
}

static inline void x264_reference_build_list( x264_t *h, int i_poc )
{
    int i;
    int b_ok;

    /* build ref list 0/1 */
    h->i_ref0 = 0;
    h->i_ref1 = 0;
    for( i = 1; i < h->param.i_frame_reference+2; i++ )
    {
        if( h->frames.reference[i]->i_poc >= 0 )
        {
            if( h->frames.reference[i]->i_poc < i_poc )
            {
                h->fref0[h->i_ref0++] = h->frames.reference[i];
            }
            else if( h->frames.reference[i]->i_poc > i_poc )
            {
                h->fref1[h->i_ref1++] = h->frames.reference[i];
            }
        }
    }
    /* Order ref0 from higher to lower poc */
    do
    {
        b_ok = 1;
        for( i = 0; i < h->i_ref0 - 1; i++ )
        {
            if( h->fref0[i]->i_poc < h->fref0[i+1]->i_poc )
            {
                x264_frame_t *tmp = h->fref0[i+1];

                h->fref0[i+1] = h->fref0[i];
                h->fref0[i] = tmp;
                b_ok = 0;
                break;
            }
        }
    } while( !b_ok );
    /* Order ref1 from lower to higher poc (bubble sort) for B-frame */
    do
    {
        b_ok = 1;
        for( i = 0; i < h->i_ref1 - 1; i++ )
        {
            if( h->fref1[i]->i_poc > h->fref1[i+1]->i_poc )
            {
                x264_frame_t *tmp = h->fref1[i+1];

                h->fref1[i+1] = h->fref1[i];
                h->fref1[i] = tmp;
                b_ok = 0;
                break;
            }
        }
    } while( !b_ok );

    if( h->i_ref0 > h->param.i_frame_reference )
    {
        h->i_ref0 = h->param.i_frame_reference;
    }
    if( h->i_ref1 > 1 )
    {
        h->i_ref1 = 1;
    }
}

static inline void x264_reference_update( x264_t *h )
{
    int i;

    /* apply deblocking filter to the current decoded picture */
    if( h->param.b_deblocking_filter )
    {
        TIMER_START( i_mtime_filter );
        x264_frame_deblocking_filter( h, h->sh.i_type );
        TIMER_STOP( i_mtime_filter );
    }
    /* expand border */
    x264_frame_expand_border( h->fdec );

    /* create filtered images */
    x264_frame_filter( h->param.cpu, h->fdec );

    /* expand border of filtered images */
    x264_frame_expand_border_filtered( h->fdec );

    /* move frame in the buffer */
    h->fdec = h->frames.reference[h->param.i_frame_reference+1];
    for( i = h->param.i_frame_reference+1; i > 0; i-- )
    {
        h->frames.reference[i] = h->frames.reference[i-1];
    }
    h->frames.reference[0] = h->fdec;
}

static inline void x264_reference_reset( x264_t *h )
{
    int i;

    /* reset ref pictures */
    for( i = 1; i < h->param.i_frame_reference+2; i++ )
    {
        h->frames.reference[i]->i_poc = -1;
    }
    h->frames.reference[0]->i_poc = 0;
}

static inline void x264_slice_init( x264_t *h, int i_nal_type, int i_slice_type, int i_global_qp )
{
    /* ------------------------ Create slice header  ----------------------- */
    if( i_nal_type == NAL_SLICE_IDR )
    {
        x264_slice_header_init( &h->sh, &h->param, h->sps, h->pps, i_slice_type, h->i_idr_pic_id, h->i_frame_num - 1 );

        /* increment id */
        h->i_idr_pic_id = ( h->i_idr_pic_id + 1 ) % 65536;
    }
    else
    {
        x264_slice_header_init( &h->sh, &h->param, h->sps, h->pps, i_slice_type, -1, h->i_frame_num - 1 );

        /* always set the real higher num of ref frame used */
        h->sh.b_num_ref_idx_override = 1;
        h->sh.i_num_ref_idx_l0_active = h->i_ref0 <= 0 ? 1 : h->i_ref0;
        h->sh.i_num_ref_idx_l1_active = h->i_ref1 <= 0 ? 1 : h->i_ref1;
    }

    if( h->sps->i_poc_type == 0 )
    {
        h->sh.i_poc_lsb = h->fdec->i_poc & ( (1 << h->sps->i_log2_max_poc_lsb) - 1 );
        h->sh.i_delta_poc_bottom = 0;   /* XXX won't work for field */
    }
    else if( h->sps->i_poc_type == 1 )
    {
        /* FIXME TODO FIXME */
    }
    else
    {
        /* Nothing to do ? */
    }

    /* global qp */
    h->sh.i_qp_delta = i_global_qp - h->pps->i_pic_init_qp;

    /* get adapative cabac model if needed */
    if( h->param.b_cabac )
    {
        if( h->param.i_cabac_init_idc == -1 )
        {
            h->sh.i_cabac_init_idc = x264_cabac_model_get( &h->cabac, i_slice_type );
        }
    }

    x264_macroblock_slice_init( h );
}

static inline void x264_slice_write( x264_t *h, int i_nal_type, int i_nal_ref_idc )
{
    int i_skip;
    int mb_xy;
    int i;

    /* Init stats */
    h->stat.frame.i_hdr_bits  =
    h->stat.frame.i_itex_bits =
    h->stat.frame.i_ptex_bits =
    h->stat.frame.i_misc_bits =
    h->stat.frame.i_intra_cost =
    h->stat.frame.i_inter_cost = 0;
    for( i = 0; i < 18; i++ )
        h->stat.frame.i_mb_count[i] = 0;

    /* Slice */
    x264_nal_start( h, i_nal_type, i_nal_ref_idc );

    /* Slice header */
    x264_slice_header_write( &h->out.bs, &h->sh, i_nal_ref_idc );
    if( h->param.b_cabac )
    {
        /* alignment needed */
        bs_align_1( &h->out.bs );

        /* init cabac */
        x264_cabac_context_init( &h->cabac, h->sh.i_type, h->sh.pps->i_pic_init_qp + h->sh.i_qp_delta, h->sh.i_cabac_init_idc );
        x264_cabac_encode_init ( &h->cabac, &h->out.bs );
    }
    h->mb.i_last_qp = h->pps->i_pic_init_qp + h->sh.i_qp_delta;
    h->mb.i_last_dqp = 0;

    for( mb_xy = 0, i_skip = 0; mb_xy < h->sps->i_mb_width * h->sps->i_mb_height; mb_xy++ )
    {
        const int i_mb_y = mb_xy / h->sps->i_mb_width;
        const int i_mb_x = mb_xy % h->sps->i_mb_width;

        int mb_spos = bs_pos(&h->out.bs);

        /* load cache */
        x264_macroblock_cache_load( h, i_mb_x, i_mb_y );

        /* analyse parameters
         * Slice I: choose I_4x4 or I_16x16 mode
         * Slice P: choose between using P mode or intra (4x4 or 16x16)
         * */
        TIMER_START( i_mtime_analyse );
        x264_macroblock_analyse( h );
        TIMER_STOP( i_mtime_analyse );

        /* encode this macrobock -> be carefull it can change the mb type to P_SKIP if needed */
        TIMER_START( i_mtime_encode );
        x264_macroblock_encode( h );
        TIMER_STOP( i_mtime_encode );

        TIMER_START( i_mtime_write );
        if( IS_SKIP( h->mb.i_type ) )
        {
            if( h->param.b_cabac )
            {
                if( mb_xy > 0 )
                {
                    /* not end_of_slice_flag */
                    x264_cabac_encode_terminal( &h->cabac, 0 );
                }

                x264_cabac_mb_skip( h, 1 );
            }
            else
            {
                i_skip++;
            }
        }
        else
        {
            if( h->param.b_cabac )
            {
                if( mb_xy > 0 )
                {
                    /* not end_of_slice_flag */
                    x264_cabac_encode_terminal( &h->cabac, 0 );
                }
                if( h->sh.i_type != SLICE_TYPE_I )
                {
                    x264_cabac_mb_skip( h, 0 );
                }
                x264_macroblock_write_cabac( h, &h->out.bs );
            }
            else
            {
                if( h->sh.i_type != SLICE_TYPE_I )
                {
                    bs_write_ue( &h->out.bs, i_skip );  /* skip run */
                    i_skip = 0;
                }
                x264_macroblock_write_cavlc( h, &h->out.bs );
            }
        }
        TIMER_STOP( i_mtime_write );

        /* save cache */
        x264_macroblock_cache_save( h );

        h->stat.frame.i_mb_count[h->mb.i_type]++;

        x264_ratecontrol_mb(h, bs_pos(&h->out.bs) - mb_spos);
    }

    if( h->param.b_cabac )
    {
        /* end of slice */
        x264_cabac_encode_terminal( &h->cabac, 1 );
    }
    else if( i_skip > 0 )
    {
        bs_write_ue( &h->out.bs, i_skip );  /* last skip run */
    }

    if( h->param.b_cabac )
    {
        int i_cabac_word;
        x264_cabac_encode_flush( &h->cabac );
        /* TODO cabac stuffing things (p209) */
        i_cabac_word = (((3 * h->cabac.i_sym_cnt - 3 * 96 * h->sps->i_mb_width * h->sps->i_mb_height)/32) - bs_pos( &h->out.bs)/8)/3;

        while( i_cabac_word > 0 )
        {
            bs_write( &h->out.bs, 16, 0x0000 );
            i_cabac_word--;
        }
    }
    else
    {
        /* rbsp_slice_trailing_bits */
        bs_rbsp_trailing( &h->out.bs );
    }

    x264_nal_end( h );

    /* Compute misc bits */
    h->stat.frame.i_misc_bits = bs_pos( &h->out.bs )
                              - h->stat.frame.i_itex_bits
                              - h->stat.frame.i_ptex_bits
                              - h->stat.frame.i_hdr_bits;
}

/****************************************************************************
 * x264_encoder_encode:
 *  XXX: i_poc   : is the poc of the current given picture
 *       i_frame : is the number of the frame being coded
 *  ex:  type frame poc
 *       I      0   2*0
 *       P      1   2*3
 *       B      2   2*1
 *       B      3   2*2
 *       P      4   2*6
 *       B      5   2*4
 *       B      6   2*5
 ****************************************************************************/
int     x264_encoder_encode( x264_t *h,
                             x264_nal_t **pp_nal, int *pi_nal,
                             x264_picture_t *pic )
{
    x264_frame_t   *frame_psnr = h->fdec; /* just to keep the current decoded frame for psnr calculation */
    int     i_nal_type;
    int     i_nal_ref_idc;
    int     i_slice_type;

    int i;

    int   i_global_qp;

    char psz_message[80];

    /* no data out */
    *pi_nal = 0;
    *pp_nal = NULL;


    /* ------------------- Setup new frame from picture -------------------- */
    TIMER_START( i_mtime_encode_frame );
    if( pic != NULL )
    {
        /* 1: Copy the picture to a frame and move it to a buffer */
        x264_frame_t *fenc = x264_frame_get( h->frames.unused );

        x264_frame_copy_picture( h, fenc, pic );

        fenc->i_frame = h->frames.i_input++;

        x264_frame_put( h->frames.next, fenc );

        if( h->frames.i_input <= h->param.i_bframe )
        {
            /* Nothing yet to encode */
            /* waiting for filling bframe buffer */
            pic->i_type = X264_TYPE_AUTO;
            return 0;
        }
    }

    if( h->frames.current[0] == NULL )
    {
        /* 2: Select frame types */
        x264_frame_t *frm;
        int bframes;

        if( h->frames.next[0] == NULL )
            return 0;

        if( h->param.rc.b_stat_read )
        {
            /* Use the frame types from the first pass */
            for( i = 0; h->frames.next[i] != NULL; i++ )
                h->frames.next[i]->i_type =
                    x264_ratecontrol_slice_type( h, h->frames.next[i]->i_frame );
        }

        for( bframes = 0;; bframes++ )
        {
            frm = h->frames.next[bframes];

            /* Limit GOP size */
            if( frm->i_frame - h->frames.i_last_idr >= h->param.i_keyint_max )
            {
                if( frm->i_type == X264_TYPE_AUTO )
                    frm->i_type = X264_TYPE_IDR;
                if( frm->i_type != X264_TYPE_IDR )
                    x264_log( h, X264_LOG_ERROR, "specified frame type (%d) is not compatible with keyframe interval\n", frm->i_type );
            }
            if( frm->i_type == X264_TYPE_IDR )
            {
                h->i_poc = 0;
                h->i_frame_num = 0;

                /* Close GOP */
                if( bframes > 0 )
                {
                    bframes--;
                    h->frames.next[bframes]->i_type = X264_TYPE_P;
                }
            }

            if( bframes == h->param.i_bframe
                || h->frames.next[bframes+1] == NULL )
            {
                if( frm->i_type == X264_TYPE_B )
                    x264_log( h, X264_LOG_ERROR, "specified frame type is not compatible with max B-frames\n" );
                if(    frm->i_type == X264_TYPE_AUTO
                    || frm->i_type == X264_TYPE_B )
                    frm->i_type = X264_TYPE_P;
            }

            frm->i_poc = h->i_poc;
            h->i_poc += 2;

            if( frm->i_type != X264_TYPE_AUTO && frm->i_type != X264_TYPE_B )
                break;

            frm->i_type = X264_TYPE_B;
        }

        /* 3: move some B-frames and 1 non-B to encode queue */
        x264_frame_put( h->frames.current, h->frames.next[bframes] );
        while( bframes-- )
            x264_frame_put( h->frames.current, x264_frame_get( h->frames.next ) );
        x264_frame_get( h->frames.next );
    }
    TIMER_STOP( i_mtime_encode_frame );

    /* ------------------- Get frame to be encoded ------------------------- */
    /* 4: get picture to encode */
    h->fenc = x264_frame_get( h->frames.current );
    if( h->fenc == NULL )
    {
        /* Nothing yet to encode (ex: waiting for I/P with B frames) */
        /* waiting for filling bframe buffer */
        pic->i_type = X264_TYPE_AUTO;
        return 0;
    }

do_encode:

    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        h->frames.i_last_idr = h->fenc->i_frame;
    }

    /* ------------------- Setup frame context ----------------------------- */
    /* 5: Init data dependant of frame type */
    TIMER_START( i_mtime_encode_frame );
    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        /* reset ref pictures */
        x264_reference_reset( h );

        i_nal_type    = NAL_SLICE_IDR;
        i_nal_ref_idc = NAL_PRIORITY_HIGHEST;
        i_slice_type = SLICE_TYPE_I;
    }
    else if( h->fenc->i_type == X264_TYPE_I )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        i_slice_type = SLICE_TYPE_I;
    }
    else if( h->fenc->i_type == X264_TYPE_P )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        i_slice_type = SLICE_TYPE_P;
    }
    else    /* B frame */
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_DISPOSABLE;
        i_slice_type = SLICE_TYPE_B;
    }

    pic->i_type     =
    h->fdec->i_type = h->fenc->i_type;
    h->fdec->i_poc  = h->fenc->i_poc;
    h->fdec->i_frame = h->fenc->i_frame;



    /* ------------------- Init                ----------------------------- */
    /* Init the rate control */
    x264_ratecontrol_start( h, i_slice_type );
    i_global_qp = x264_ratecontrol_qp( h );
    if( h->fenc->i_qpplus1 > 0 )
    {
        i_global_qp = x264_clip3( h->fenc->i_qpplus1 - 1, 0, 51 );
    }

    /* build ref list 0/1 */
    x264_reference_build_list( h, h->fdec->i_poc );

    /* increase frame num but only once for B frame */
    if( i_slice_type != SLICE_TYPE_B || h->sh.i_type != SLICE_TYPE_B )
    {
        h->i_frame_num++;
    }

    /* ------------------------ Create slice header  ----------------------- */
    x264_slice_init( h, i_nal_type, i_slice_type, i_global_qp );

    /* ---------------------- Write the bitstream -------------------------- */
    /* Init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    /* Write SPS and PPS */
    if( i_nal_type == NAL_SLICE_IDR )
    {
        /* generate sequence parameters */
        x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
        x264_sps_write( &h->out.bs, h->sps );
        x264_nal_end( h );

        /* generate picture parameters */
        x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
        x264_pps_write( &h->out.bs, h->pps );
        x264_nal_end( h );
    }

    /* Write the slice */
    x264_slice_write( h, i_nal_type, i_nal_ref_idc );

    /* restore CPU state (before using float again) */
    x264_cpu_restore( h->param.cpu );

    /* XXX: this scene cut won't work with B frame (it may never create IDR -> bad) */
    if( i_slice_type == SLICE_TYPE_P && !h->param.rc.b_stat_read 
        && h->param.i_scenecut_threshold >= 0 )
    {
        int i_mb_i = h->stat.frame.i_mb_count[I_4x4] + h->stat.frame.i_mb_count[I_16x16];
        int i_mb_p = h->stat.frame.i_mb_count[P_L0] + h->stat.frame.i_mb_count[P_8x8];
        int i_mb_s = h->stat.frame.i_mb_count[P_SKIP];
        int i_mb   = h->sps->i_mb_width * h->sps->i_mb_height;
        int64_t i_inter_cost = h->stat.frame.i_inter_cost;
        int64_t i_intra_cost = h->stat.frame.i_intra_cost;

        float f_bias;
        int i_gop_size = h->fenc->i_frame - h->frames.i_last_idr;
        float f_thresh_max = h->param.i_scenecut_threshold / 100.0;
        /* ratio of 10 pulled out of thin air */
        float f_thresh_min = f_thresh_max * h->param.i_keyint_min
                             / ( h->param.i_keyint_max * 4 );
        if( h->param.i_keyint_min == h->param.i_keyint_max )
             f_thresh_min= f_thresh_max;

        /* macroblock_analyse() doesn't further analyse skipped mbs,
         * so we have to guess their cost */
        if( i_mb_s < i_mb )
            i_intra_cost = i_intra_cost * i_mb / (i_mb - i_mb_s);

        if( i_gop_size < h->param.i_keyint_min / 4 )
            f_bias = f_thresh_min / 4;
        else if( i_gop_size <= h->param.i_keyint_min )
            f_bias = f_thresh_min * i_gop_size / h->param.i_keyint_min;
        else
        {
            f_bias = f_thresh_min
                     + ( f_thresh_max - f_thresh_min )
                       * ( i_gop_size - h->param.i_keyint_min )
                       / ( h->param.i_keyint_max - h->param.i_keyint_min );
        }
        f_bias = X264_MIN( f_bias, 1.0 );

        /* Bad P will be reencoded as I */
        if( i_mb_s < i_mb &&
            i_inter_cost >= (1.0 - f_bias) * i_intra_cost )
            /* i_mb_i >= (1.0 - f_bias) * i_mb ) */
            /*
            h->out.nal[h->out.i_nal-1].i_payload > h->i_last_intra_size +
            h->i_last_intra_size * (3+h->i_last_intra_qp - i_global_qp) / 16 &&
            i_mb_count[I_4x4] + i_mb_count[I_16x16] > i_mb_count[P_SKIP] + i_mb_count[P_L0]/2 &&
            h->out.nal[h->out.i_nal-1].i_payload > 2 * h->i_last_inter_size &&
            h->frames.i_last_i > 4)*/
        {

            x264_log( h, X264_LOG_DEBUG, "scene cut at %d size=%d Icost:%.0f Pcost:%.0f ratio:%.3f bias=%.3f lastIDR:%d (I:%d P:%d Skip:%d)\n",
                      h->fenc->i_frame,
                      h->out.nal[h->out.i_nal-1].i_payload,
                      (double)i_intra_cost, (double)i_inter_cost,
                      (double)i_inter_cost / i_intra_cost,
                      f_bias, i_gop_size,
                      i_mb_i, i_mb_p, i_mb_s );

            /* Restore frame num */
            h->i_frame_num--;

            for( i = 0; h->frames.current[i] && h->frames.current[i]->i_type == X264_TYPE_B; i++ );
            if( i > 0 )
            {
                /* If using B-frames, force GOP to be closed.
                 * Even if this frame is going to be I and not IDR, forcing a
                 * P-frame before the scenecut will probably help compression.
                 * 
                 * We don't yet know exactly which frame is the scene cut, so
                 * we can't assign an I-frame. Instead, change the previous
                 * B-frame to P, and rearrange coding order. */

                x264_frame_t *tmp = h->frames.current[i-1];
                h->frames.current[i-1] = h->fenc;
                h->fenc = tmp;
                h->fenc->i_type = X264_TYPE_P;
            }
            /* Do IDR if needed */
            else if( i_gop_size >= h->param.i_keyint_min )
            {
                x264_frame_t *tmp;

                /* Reset */
                h->i_poc       = 0;
                h->i_frame_num = 0;

                /* Reinit field of fenc */
                h->fenc->i_type = X264_TYPE_IDR;
                h->fenc->i_poc = 0;

                /* Next Poc */
                h->i_poc += 2;

                /* Put enqueued frames back in the pool */
                while( (tmp = x264_frame_get( h->frames.current ) ) != NULL )
                    x264_frame_put( h->frames.next, tmp );
                x264_frame_sort( h->frames.next );
            }
            else
            {
                h->fenc->i_type = X264_TYPE_I;
            }
            goto do_encode;
        }
        h->i_last_inter_size = h->out.nal[h->out.i_nal-1].i_payload;
    }
    else
    {
        h->i_last_intra_size = h->out.nal[h->out.i_nal-1].i_payload;
        h->i_last_intra_qp = i_global_qp;
    }

    /* End bitstream, set output  */
    *pi_nal = h->out.i_nal;
    *pp_nal = &h->out.nal[0];

    /* Set output picture properties */
    if( i_slice_type == SLICE_TYPE_I )
        pic->i_type = i_nal_type == NAL_SLICE_IDR ? X264_TYPE_IDR : X264_TYPE_I;
    else if( i_slice_type == SLICE_TYPE_P )
        pic->i_type = X264_TYPE_P;
    else
        pic->i_type = X264_TYPE_B;
    pic->i_pts = h->fenc->i_pts;

    /* ---------------------- Update encoder state ------------------------- */
    /* update cabac */
    if( h->param.b_cabac )
    {
        x264_cabac_model_update( &h->cabac, i_slice_type, h->sh.pps->i_pic_init_qp + h->sh.i_qp_delta );
    }

    /* handle references */
    if( i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE )
    {
        x264_reference_update( h );
    }

    /* increase frame count */
    h->i_frame++;

    /* restore CPU state (before using float again) */
    /* XXX: not needed? (done above) */
    x264_cpu_restore( h->param.cpu );

    /* update rc */
    x264_ratecontrol_end( h, h->out.nal[h->out.i_nal-1].i_payload * 8 );

    x264_frame_put( h->frames.unused, h->fenc );

    TIMER_STOP( i_mtime_encode_frame );

    /* ---------------------- Compute/Print statistics --------------------- */
    /* Slice stat */
    h->stat.i_slice_count[i_slice_type]++;
    h->stat.i_slice_size[i_slice_type] += bs_pos( &h->out.bs) / 8;
    h->stat.i_slice_qp[i_slice_type] += i_global_qp;

    for( i = 0; i < 18; i++ )
    {
        h->stat.i_mb_count[h->sh.i_type][i] += h->stat.frame.i_mb_count[i];
    }

    if( h->param.analyse.b_psnr )
    {
        int64_t i_sqe_y, i_sqe_u, i_sqe_v;

        /* PSNR */
        i_sqe_y = x264_sqe( frame_psnr->plane[0], frame_psnr->i_stride[0], h->fenc->plane[0], h->fenc->i_stride[0], h->param.i_width, h->param.i_height );
        i_sqe_u = x264_sqe( frame_psnr->plane[1], frame_psnr->i_stride[1], h->fenc->plane[1], h->fenc->i_stride[1], h->param.i_width/2, h->param.i_height/2);
        i_sqe_v = x264_sqe( frame_psnr->plane[2], frame_psnr->i_stride[2], h->fenc->plane[2], h->fenc->i_stride[2], h->param.i_width/2, h->param.i_height/2);

        h->stat.i_sqe_global[i_slice_type] += i_sqe_y + i_sqe_u + i_sqe_v;
        h->stat.f_psnr_average[i_slice_type] += x264_psnr( i_sqe_y + i_sqe_u + i_sqe_v, 3 * h->param.i_width * h->param.i_height / 2 );
        h->stat.f_psnr_mean_y[i_slice_type] += x264_psnr( i_sqe_y, h->param.i_width * h->param.i_height );
        h->stat.f_psnr_mean_u[i_slice_type] += x264_psnr( i_sqe_u, h->param.i_width * h->param.i_height / 4 );
        h->stat.f_psnr_mean_v[i_slice_type] += x264_psnr( i_sqe_v, h->param.i_width * h->param.i_height / 4 );

        snprintf( psz_message, 80, " PSNR Y:%2.2f U:%2.2f V:%2.2f",
                  x264_psnr( i_sqe_y, h->param.i_width * h->param.i_height ),
                  x264_psnr( i_sqe_u, h->param.i_width * h->param.i_height / 4),
                  x264_psnr( i_sqe_v, h->param.i_width * h->param.i_height / 4) );
        psz_message[79] = '\0';
    }
    else
    {
        psz_message[0] = '\0';
    }
    
    x264_log( h, X264_LOG_DEBUG,
                  "frame=%4d QP=%i NAL=%d Slice:%c Poc:%-3d I4x4:%-4d I16x16:%-4d P:%-4d SKIP:%-4d size=%d bytes%s\n",
              h->i_frame - 1,
              i_global_qp,
              i_nal_ref_idc,
              i_slice_type == SLICE_TYPE_I ? 'I' : (i_slice_type == SLICE_TYPE_P ? 'P' : 'B' ),
              frame_psnr->i_poc,
              h->stat.frame.i_mb_count[I_4x4],
              h->stat.frame.i_mb_count[I_16x16],
              h->stat.frame.i_mb_count_p,
              h->stat.frame.i_mb_count_skip,
              h->out.nal[h->out.i_nal-1].i_payload,
              psz_message );


#ifdef DEBUG_MB_TYPE
{
    static const char mb_chars[] = { 'i', 'I', 'C', 'P', '8', 'S',
        'D', '<', 'X', 'B', 'X', '>', 'B', 'B', 'B', 'B', '8', 'S' };
    int mb_xy;
    for( mb_xy = 0; mb_xy < h->sps->i_mb_width * h->sps->i_mb_height; mb_xy++ )
    {
        if( h->mb.type[mb_xy] < 18 && h->mb.type[mb_xy] >= 0 )
            fprintf( stderr, "%c ", mb_chars[ h->mb.type[mb_xy] ] );
        else
            fprintf( stderr, "? " );

        if( (mb_xy+1) % h->sps->i_mb_width == 0 )
            fprintf( stderr, "\n" );
    }
}
#endif

#ifdef DEBUG_DUMP_FRAME
    /* Dump reconstructed frame */
    x264_frame_dump( h, frame_psnr, "fdec.yuv" );
#endif
#if 0
    if( h->i_ref0 > 0 )
    {
        x264_frame_dump( h, h->fref0[0], "ref0.yuv" );
    }
    if( h->i_ref1 > 0 )
    {
        x264_frame_dump( h, h->fref1[0], "ref1.yuv" );
    }
#endif
    return 0;
}

/****************************************************************************
 * x264_encoder_close:
 ****************************************************************************/
void    x264_encoder_close  ( x264_t *h )
{
#ifdef DEBUG_BENCHMARK
    int64_t i_mtime_total = i_mtime_analyse + i_mtime_encode + i_mtime_write + i_mtime_filter + 1;
#endif
    int64_t i_yuv_size = 3 * h->param.i_width * h->param.i_height / 2;
    int i;

#ifdef DEBUG_BENCHMARK
    x264_log( h, X264_LOG_INFO,
              "analyse=%d(%lldms) encode=%d(%lldms) write=%d(%lldms) filter=%d(%lldms)\n",
              (int)(100*i_mtime_analyse/i_mtime_total), i_mtime_analyse/1000,
              (int)(100*i_mtime_encode/i_mtime_total), i_mtime_encode/1000,
              (int)(100*i_mtime_write/i_mtime_total), i_mtime_write/1000,
              (int)(100*i_mtime_filter/i_mtime_total), i_mtime_filter/1000 );
#endif

    /* Slices used and PSNR */
    for( i=0; i<5; i++ )
    {
        static const int slice_order[] = { SLICE_TYPE_I, SLICE_TYPE_SI, SLICE_TYPE_P, SLICE_TYPE_SP, SLICE_TYPE_B };
        static const char *slice_name[] = { "P", "B", "I", "SP", "SI" };
        int i_slice = slice_order[i];

        if( h->stat.i_slice_count[i_slice] > 0 )
        {
            const int i_count = h->stat.i_slice_count[i_slice];
            if( h->param.analyse.b_psnr )
            {
                x264_log( h, X264_LOG_INFO,
                          "slice %s:%-4d Avg QP:%5.2f Avg size:%6.0f PSNR Mean Y:%5.2f U:%5.2f V:%5.2f Avg:%5.2f Global:%5.2f MSE*Size:%5.3f\n",
                          slice_name[i_slice],
                          i_count,
                          (double)h->stat.i_slice_qp[i_slice] / i_count,
                          (double)h->stat.i_slice_size[i_slice] / i_count,
                          h->stat.f_psnr_mean_y[i_slice] / i_count, h->stat.f_psnr_mean_u[i_slice] / i_count, h->stat.f_psnr_mean_v[i_slice] / i_count,
                          h->stat.f_psnr_average[i_slice] / i_count,
                          x264_psnr( h->stat.i_sqe_global[i_slice], i_count * i_yuv_size ),
                          x264_mse( h->stat.i_sqe_global[i_slice], i_count * i_yuv_size ) * h->stat.i_slice_size[i_slice] / i_count );
            }
            else
            {
                x264_log( h, X264_LOG_INFO,
                          "slice %s:%-4d Avg QP:%5.2f Avg size:%6.0f\n",
                          slice_name[i_slice],
                          i_count,
                          (double)h->stat.i_slice_qp[i_slice] / i_count,
                          (double)h->stat.i_slice_size[i_slice] / i_count );
            }
        }
    }

    /* MB types used */
    if( h->stat.i_slice_count[SLICE_TYPE_I] > 0 )
    {
        const int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_I];
        const double i_count = h->stat.i_slice_count[SLICE_TYPE_I] * h->mb.i_mb_count / 100.0;
        x264_log( h, X264_LOG_INFO,
                  "slice I   Avg I4x4:%.1f%%  I16x16:%.1f%%\n",
                  i_mb_count[I_4x4]  / i_count,
                  i_mb_count[I_16x16]/ i_count );
    }
    if( h->stat.i_slice_count[SLICE_TYPE_P] > 0 )
    {
        const int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_P];
        const double i_count = h->stat.i_slice_count[SLICE_TYPE_P] * h->mb.i_mb_count / 100.0;
        x264_log( h, X264_LOG_INFO,
                  "slice P   Avg I4x4:%.1f%%  I16x16:%.1f%%  P:%.1f%%  P8x8:%.1f%%  PSKIP:%.1f%%\n",
                  i_mb_count[I_4x4]  / i_count,
                  i_mb_count[I_16x16]/ i_count,
                  i_mb_count[P_L0]   / i_count,
                  i_mb_count[P_8x8]  / i_count,
                  i_mb_count[P_SKIP] / i_count );
    }
    if( h->stat.i_slice_count[SLICE_TYPE_B] > 0 )
    {
        const int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_B];
        const double i_count = h->stat.i_slice_count[SLICE_TYPE_B] * h->mb.i_mb_count / 100.0;
        x264_log( h, X264_LOG_INFO,
                  "slice B   Avg I4x4:%.1f%%  I16x16:%.1f%%  P:%.1f%%  B:%.1f%%  B8x8:%.1f%%  DIRECT:%.1f%%  BSKIP:%.1f%%\n",
                  i_mb_count[I_4x4]    / i_count,
                  i_mb_count[I_16x16]  / i_count,
                  (i_mb_count[B_L0_L0] + i_mb_count[B_L1_L1] + i_mb_count[B_L1_L0] + i_mb_count[B_L0_L1]) / i_count,
                  (i_mb_count[B_BI_BI] + i_mb_count[B_L0_BI] + i_mb_count[B_L1_BI] + i_mb_count[B_BI_L0] + i_mb_count[B_BI_L1]) / i_count,
                  i_mb_count[B_8x8]    / i_count,
                  i_mb_count[B_DIRECT] / i_count,
                  i_mb_count[B_SKIP]   / i_count );
    }

    if( h->stat.i_slice_count[SLICE_TYPE_I] + h->stat.i_slice_count[SLICE_TYPE_P] + h->stat.i_slice_count[SLICE_TYPE_B] > 0 )
    {
        const int i_count = h->stat.i_slice_count[SLICE_TYPE_I] +
                            h->stat.i_slice_count[SLICE_TYPE_P] +
                            h->stat.i_slice_count[SLICE_TYPE_B];
        float fps = (float) h->param.i_fps_num / h->param.i_fps_den;

        if( h->param.analyse.b_psnr )
            x264_log( h, X264_LOG_INFO,
                      "PSNR Mean Y:%5.2f U:%5.2f V:%5.2f Avg:%5.2f Global:%5.2f kb/s:%.1f\n",
                      (h->stat.f_psnr_mean_y[SLICE_TYPE_I] + h->stat.f_psnr_mean_y[SLICE_TYPE_P] + h->stat.f_psnr_mean_y[SLICE_TYPE_B]) / i_count,
                      (h->stat.f_psnr_mean_u[SLICE_TYPE_I] + h->stat.f_psnr_mean_u[SLICE_TYPE_P] + h->stat.f_psnr_mean_u[SLICE_TYPE_B]) / i_count,
                      (h->stat.f_psnr_mean_v[SLICE_TYPE_I] + h->stat.f_psnr_mean_v[SLICE_TYPE_P] + h->stat.f_psnr_mean_v[SLICE_TYPE_B]) / i_count,

                      (h->stat.f_psnr_average[SLICE_TYPE_I] + h->stat.f_psnr_average[SLICE_TYPE_P] + h->stat.f_psnr_average[SLICE_TYPE_B]) / i_count,

                      x264_psnr( h->stat.i_sqe_global[SLICE_TYPE_I] + h->stat.i_sqe_global[SLICE_TYPE_P]+ h->stat.i_sqe_global[SLICE_TYPE_B],
                                 i_count * i_yuv_size ),
                      fps * 8*(h->stat.i_slice_size[SLICE_TYPE_I]+h->stat.i_slice_size[SLICE_TYPE_P]+h->stat.i_slice_size[SLICE_TYPE_B]) / i_count / 1000 );
        else
            x264_log( h, X264_LOG_INFO,
                      "kb/s:%.1f\n",
                      fps * 8*(h->stat.i_slice_size[SLICE_TYPE_I]+h->stat.i_slice_size[SLICE_TYPE_P]+h->stat.i_slice_size[SLICE_TYPE_B]) / i_count / 1000 );
    }

    /* frames */
    for( i = 0; i < X264_BFRAME_MAX + 1; i++ )
    {
        if( h->frames.current[i] ) x264_frame_delete( h->frames.current[i] );
        if( h->frames.next[i] )    x264_frame_delete( h->frames.next[i] );
        if( h->frames.unused[i] )  x264_frame_delete( h->frames.unused[i] );
    }
    /* ref frames */
    for( i = 0; i < h->param.i_frame_reference+2; i++ )
    {
        x264_frame_delete( h->frames.reference[i] );
    }

    /* rc */
    x264_ratecontrol_delete( h );

    /* param */
    if( h->param.rc.psz_stat_out )
        free( h->param.rc.psz_stat_out );
    if( h->param.rc.psz_stat_in )
        free( h->param.rc.psz_stat_in );
    if( h->param.rc.psz_rc_eq )
        free( h->param.rc.psz_rc_eq );

    x264_macroblock_cache_end( h );
    x264_free( h->out.p_bitstream );
    x264_free( h );
}

