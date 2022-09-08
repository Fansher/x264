/*****************************************************************************
 * bitstream.c: bitstream writing
 *****************************************************************************
 * Copyright (C) 2003-2022 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Fiona Glaser <fiona@x264.com>
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#include "common.h"

// 写nalu码流，该函数最关键的是H264中的“防止竞争”机制
static uint8_t *nal_escape_c( uint8_t *dst, uint8_t *src, uint8_t *end )
{
    if( src < end ) *dst++ = *src++;
    if( src < end ) *dst++ = *src++;
    while( src < end )
    {
        // 当数据中出现连续两个0x00时，需要添加一个0x03
        if( src[0] <= 0x03 && !dst[-2] && !dst[-1] )
            *dst++ = 0x03;
        *dst++ = *src++;
    }
    return dst;
}

#if HAVE_MMX
#include "x86/bitstream.h"
#endif
#if HAVE_ARMV6
#include "arm/bitstream.h"
#endif
#if HAVE_AARCH64
#include "aarch64/bitstream.h"
#endif

/****************************************************************************
 * x264_nal_encode:
 ****************************************************************************/
// 编码一个nalu（注意：一帧数据可能有多个nalu，则需要调用多次该函数）
// 调用该函数前，nal->p_payload保存的是编码后的原始数据（SODB）经字节对齐后的数据（RBSP），nal->i_payload包含的是原始数据和字节对齐的长度
void x264_nal_encode( x264_t *h, uint8_t *dst, x264_nal_t *nal )
{
    uint8_t *src = nal->p_payload;
    uint8_t *end = nal->p_payload + nal->i_payload;
    uint8_t *orig_dst = dst;

    // H264草案附录B指明(AnnexB格式)：当数据流存储在介质上时，在每一个nalu前需要添加起始码（3字节长或者4字节长）
    // 只有在一帧数据被分成多个slice后有多个nalu单元，除去第一个nalu之外的其他nalu，首部添加3字节长的起始码"0x000001"
    // 其他nalu（包括sps、pps、首个nalu）首部添加4字节长的起始码"0x00000001"，称为长前缀码
    if( h->param.b_annexb )
    {
        if( nal->b_long_startcode )
            *dst++ = 0x00;
        *dst++ = 0x00;
        *dst++ = 0x00;
        *dst++ = 0x01;
    }
    else /* save room for size later */  // 即使不是AnnexB格式（也即avcC格式），也需要预留出4字节保存数据的长度
        dst += 4;

    /* nal header */
    /* 
    ** forbidden_zero_bit ：1bit，恒为0
    ** nal_ref_idc        ：2bit，nal优先级
    ** nal_uint_type      ：5bit，当前nal类型，其中5是IDR，6是sei，7是sps，8是pps
    */
    *dst++ = ( 0x00 << 7 ) | ( nal->i_ref_idc << 5 ) | nal->i_type;

    dst = h->bsf.nal_escape( dst, src, end ); // 防竞争机制
    int size = dst - orig_dst;  
    // 如果是AnnexB格式，到这里已经编码完nalu了，size即是nalu的长度
    // 此时调用最后3行代码后，nal->p_payload保存的是rbsp经起始码和防竞争机制编码后的数据，nal->i_payload包含的是以上提及的所有字节长度（SODB、字节填充、起始码、防止竞争）
    // AnnexB格式下的数据：
    // startcode + NALU
    // NALU = NALUHeader + EBSP
    // EBSP = 防竞争码 + RBSP
    // RBSP = SODB字节填充对齐

    // 以下两个if语句主要针对AvcC格式（AnnexB直接跳过）
    /* Apply AVC-Intra padding */
    if( h->param.i_avcintra_class )
    {
        int padding = nal->i_payload + nal->i_padding + NALU_OVERHEAD - size;
        if( padding > 0 )
        {
            memset( dst, 0, padding );
            size += padding;
        }
        nal->i_padding = X264_MAX( padding, 0 );
    }
    // AvcC格式下的size具体指什么呢？上面这个if没看懂。。。

    /* Write the size header for mp4/etc */
    if( !h->param.b_annexb )
    {
        /* Size doesn't include the size of the header we're writing now. */
        int chunk_size = size - 4;
        orig_dst[0] = (uint8_t)(chunk_size >> 24);
        orig_dst[1] = (uint8_t)(chunk_size >> 16);
        orig_dst[2] = (uint8_t)(chunk_size >> 8);
        orig_dst[3] = (uint8_t)(chunk_size >> 0);
    }

    nal->i_payload = size;
    nal->p_payload = orig_dst;
    x264_emms();
}

void x264_bitstream_init( uint32_t cpu, x264_bitstream_function_t *pf )
{
    memset( pf, 0, sizeof(*pf) );

    pf->nal_escape = nal_escape_c;
#if HAVE_MMX
#if ARCH_X86_64
    pf->cabac_block_residual_internal = x264_cabac_block_residual_internal_sse2;
    pf->cabac_block_residual_rd_internal = x264_cabac_block_residual_rd_internal_sse2;
    pf->cabac_block_residual_8x8_rd_internal = x264_cabac_block_residual_8x8_rd_internal_sse2;
#endif

    if( cpu&X264_CPU_MMX2 )
        pf->nal_escape = x264_nal_escape_mmx2;
    if( cpu&X264_CPU_SSE2 )
    {
        if( cpu&X264_CPU_SSE2_IS_FAST )
            pf->nal_escape = x264_nal_escape_sse2;
    }
#if ARCH_X86_64
    if( cpu&X264_CPU_LZCNT )
    {
        pf->cabac_block_residual_internal = x264_cabac_block_residual_internal_lzcnt;
        pf->cabac_block_residual_rd_internal = x264_cabac_block_residual_rd_internal_lzcnt;
        pf->cabac_block_residual_8x8_rd_internal = x264_cabac_block_residual_8x8_rd_internal_lzcnt;
    }

    if( cpu&X264_CPU_SSSE3 )
    {
        pf->cabac_block_residual_rd_internal = x264_cabac_block_residual_rd_internal_ssse3;
        pf->cabac_block_residual_8x8_rd_internal = x264_cabac_block_residual_8x8_rd_internal_ssse3;
        if( cpu&X264_CPU_LZCNT )
        {
            pf->cabac_block_residual_rd_internal = x264_cabac_block_residual_rd_internal_ssse3_lzcnt;
            pf->cabac_block_residual_8x8_rd_internal = x264_cabac_block_residual_8x8_rd_internal_ssse3_lzcnt;
        }
    }

    if( cpu&X264_CPU_AVX2 )
    {
        pf->nal_escape = x264_nal_escape_avx2;
        pf->cabac_block_residual_internal = x264_cabac_block_residual_internal_avx2;
    }

    if( cpu&X264_CPU_AVX512 )
    {
        pf->cabac_block_residual_internal = x264_cabac_block_residual_internal_avx512;
        pf->cabac_block_residual_rd_internal = x264_cabac_block_residual_rd_internal_avx512;
        pf->cabac_block_residual_8x8_rd_internal = x264_cabac_block_residual_8x8_rd_internal_avx512;
    }
#endif
#endif
#if HAVE_ARMV6
    if( cpu&X264_CPU_NEON )
        pf->nal_escape = x264_nal_escape_neon;
#endif
#if HAVE_AARCH64
    if( cpu&X264_CPU_NEON )
        pf->nal_escape = x264_nal_escape_neon;
#endif
}
