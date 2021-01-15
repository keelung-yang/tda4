/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../include/TI_aaa_ae.h"

//#define AE_DEBUG

static void TIAE_adj_exposure(
        int32_t           currentY,
        tiae_exp_t      * prev_exp,
        tiae_exp_t      * next_exp,
        tiae_exp_prog_t * exp_prog);

void TI_AE_init(tiae_prm_t *h, tiae_exp_prog_t *prg)
{
    int i;

    h->num_history = 10;
    for (i = 0; i < h->num_history; i++)
    {
        h->history_brightness[i] = -1;
    }
    h->avg_y = -1;
    h->locked = 0;
    h->lock_cnt = 0;
    h->lock_thrld = 10;

    h->blc_enable = 0;
    h->blc_comp = 1024;

    h->prev_ae.aperture_size = 1;
    h->prev_ae.exposure_time = 100;
    h->prev_ae.analog_gain   = 1024;
    h->prev_ae.digital_gain  = 256;

    h->frame_num_start = 0;
    h->frame_num_period = 3;
    h->frame_num_count = 0;

    if (prg == NULL)
    {
        h->exposure_program.target_brightness = 0;
        h->exposure_program.target_brightness_range.min = 0;
        h->exposure_program.target_brightness_range.max = 0;
        h->exposure_program.exposure_time_step_size = 0;
        h->exposure_program.num_ranges = 0;
        h->exposure_program.aperture_size_range[0].min = 0;
        h->exposure_program.aperture_size_range[0].max = 0;
        h->exposure_program.exposure_time_range[0].min = 0;
        h->exposure_program.exposure_time_range[0].max = 0;
        h->exposure_program.analog_gain_range[0].min   = 0;
        h->exposure_program.analog_gain_range[0].max   = 0;
        h->exposure_program.digital_gain_range[0].min  = 0;
        h->exposure_program.digital_gain_range[0].max  = 0;
    }
    else
    {
        h->exposure_program = *prg;
    }
}

static inline int32_t abs1(int32_t x)
{
    if (x >= 0) return x;
    return -x;
}

static inline int32_t max2_ae(int32_t x, int32_t y)
{
    if (x >= y) return x;
    return y;
}

int TI_AE_do(                                // return 1: AE change; 0: no change
        tiae_prm_t            * h,
        h3a_aewb_paxel_data_t * h3a_data,    //must have been normalized by pix_in_pax
        int32_t                 h3a_data_x,  //H3A WINH
        int32_t                 h3a_data_y,  //H3A WINV
        uint8_t               * weight,      //relative weights for H3A windows (U8) or NULL
        uint16_t                r_gain,      //WB R gain (U16Q8)
        uint16_t                g_gain,      //WB G gain (U16Q8)
        uint16_t                b_gain,      //WB B gain (U16Q8)
        tiae_exp_t            * ae_out
)
{
    *ae_out = h->prev_ae;
    if ((h->frame_num_count - h->frame_num_start) % h->frame_num_period != 0)
    {
        h->frame_num_count++;
        return h->locked;
    }
    h->frame_num_count++;

    int i, j, jj;
    int width  = h3a_data_x;
    int height = h3a_data_y;
    int32_t rsum = 0, bsum = 0, gsum = 0, wsum = 0;
    int32_t cnt_tol = 0;

    /* now calculate the average weighted luminance value */
    int32_t rY = (0x4d * r_gain + 128) >> 8;
    int32_t gY = (0x96 * g_gain + 128) >> 8;
    int32_t bY = (0x1d * b_gain + 128) >> 8;

    /* first calculate sum of all R, G, B values */
    int zone[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
    int zth[16] = {8,16, 32,64, 128,256};

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            cnt_tol++;
            if (weight != NULL)
            {
                // 1024 * 64 * 64 * 128 = 2**29
                rsum += (int32_t)h3a_data[i * width + j].red   * weight[i * width + j];
                gsum += (int32_t)h3a_data[i * width + j].green * weight[i * width + j];
                bsum += (int32_t)h3a_data[i * width + j].blue  * weight[i * width + j];
                // 64 * 64 * 128 = 2**19
                wsum += weight[i * width + j];
            }
            else
            {
                rsum += (int32_t) h3a_data[i * width + j].red;
                gsum += (int32_t) h3a_data[i * width + j].green;
                bsum += (int32_t) h3a_data[i * width + j].blue;
                wsum += 1;
            }

            int32_t y = (h3a_data[i * width + j].red   * rY
                       + h3a_data[i * width + j].green * gY
                       + h3a_data[i * width + j].blue  * bY + 128) >> 8;

            for (jj = 0; jj < 6; jj++)
            {
                if (y < zth[jj])
                {
                    zone[jj]++;
                    break;
                }
            }
        }
    }

    if(0 == cnt_tol)
    {
        cnt_tol += 1;
    }

    for (jj = 0; jj < 6; jj++)
    {
        zone[jj] = (100*zone[jj] + (cnt_tol>>1)) / cnt_tol;
    }

    wsum = max2_ae(wsum, 1);
#ifdef AE_DEBUG
    printf("AE debug: R = %d, G = %d, B = %d\n", rsum / wsum, gsum / wsum, bsum / wsum);
#endif

    int32_t curY_sat = (((rsum>>8) * rY + (gsum>>8) * gY + (bsum>>8) * bY) + (wsum >> 1)) / wsum;
    int32_t curY = curY_sat;

    if(h->blc_enable) //use 1 for BLC
    {
        if (h->lock_cnt == 3 && h->locked)
        {
            if ((zone[0]+zone[1]>10 && zone[0]+zone[1]+zone[2]>30) && h->blc_comp < 1024*16)
            {
                h->blc_comp = h->blc_comp * 21 / 20;
                if (h->blc_comp > 1024*16) h->blc_comp = 1024*16;
                h->lock_cnt = 0;
                h->locked = 0;
            }
            else if ((zone[0]+zone[1]<10 && zone[0]+zone[1]+zone[2]<30) && h->blc_comp > 1024)
            {
                h->blc_comp = h->blc_comp * 19 / 20;
                if (h->blc_comp < 1024) h->blc_comp = 1024;
                h->lock_cnt = 0;
                h->locked = 0;
            }
        }
        curY = curY_sat * 1024 / h->blc_comp;

#ifdef AE_DEBUG
        printf("AE debug: Y=%d, satY=%d, C=%d, blc_comp=%d, lockcnt=%d, locked=%d\n",
                curY, curY_sat, cnt_tol, h->blc_comp, h->lock_cnt, h->locked);
        printf("%d, %d,  ## %d, %d ## %d, %d\n", zone[0], zone[1], zone[2], zone[3], zone[4], zone[5]);
#endif
    }

    curY = max2_ae(curY, 1);

    /* now update the history brightnesss data */
    /* check if current brightness is within range of the history average */
    int reset = (h->avg_y < 0);
    if (reset)
    {
        h->avg_y = curY * h->num_history;
        for (i = 0; i < h->num_history; i++)
        {
            h->history_brightness[i] = curY;
        }
        h->locked = 0;
        h->lock_cnt = 0;
    }
    else
    {
        h->avg_y = h->avg_y + curY - h->history_brightness[h->num_history - 1];

        for (i = h->num_history - 1; i > 0; i--)
        {
            h->history_brightness[i] = h->history_brightness[i - 1];
        }
        h->history_brightness[0] = curY;
    }
    h->avg_y = max2_ae(h->avg_y, 1);

    int32_t avgY    = h->avg_y;
    int32_t tgtY    = h->exposure_program.target_brightness;
    int32_t tgtYmin = h->exposure_program.target_brightness_range.min;
    int32_t tgtYmax = h->exposure_program.target_brightness_range.max;

#ifdef AE_DEBUG
    printf("AE debug: curY=%d, avgY=%d, locked=%d, lockcnt=%d\n", curY, avgY/h->num_history, h->locked, h->lock_cnt);
#endif

    if (!reset)
    {
        if (abs1(avgY - tgtY * h->num_history) < h->lock_thrld * h->num_history && h->locked)
        {
            return 1;
        }
        if ( ((curY > tgtYmin) && (curY < tgtYmax))
          || ((avgY > tgtYmin * h->num_history) && (avgY < tgtYmax * h->num_history)) )
        {
            h->lock_cnt++;
        }
        else if (h->lock_cnt > 0)
        {
            h->lock_cnt--;
        }

        if (h->lock_cnt >= 3)
        {
            h->lock_cnt = 3;
            h->locked = 1;
            return 1;
        }
        else if (h->lock_cnt > 0 && h->locked)
        {
            return 1;
        }
    }

    h->locked = 0;
    TIAE_adj_exposure(curY, &h->prev_ae, ae_out, &h->exposure_program);
    h->prev_ae = *ae_out;

    return 0;
}

static inline int32_t clip_int(int32_t v, int32_t vmin, int32_t vmax)
{
    if (v < vmin)
    {
        v = vmin;
    }
    if (v > vmax)
    {
        v = vmax;
    }
    return v;
}

static void search_in_range(float ex,
        int32_t Amin, int32_t Amax, int32_t* pA,
        int32_t Tmin, int32_t Tmax, int32_t Tstep, int32_t* pT,
        int32_t AGmin, int32_t AGmax, int32_t* pAG,
        int32_t DGmin, int32_t DGmax, int32_t* pDG)
{
    ex *= Amin;
    *pA = clip_int(ex, Amin, Amax);
    ex /= (*pA);

    ex *= Tmin;
    *pT = clip_int((int)(ex/Tstep)*Tstep, Tmin, Tmax);
    ex /= (*pT);

    ex *= AGmin;
    *pAG = clip_int(ex, AGmin, AGmax);
    ex /= *pAG;

    ex *= DGmin;
    *pDG = clip_int(ex, DGmin, DGmax);
}

static int search_range_ATG_dec_gain(int32_t adjRatio,
        int32_t cA, int32_t cT, int32_t cAG, int32_t cDG,
        tiae_range_t rA, tiae_range_t rT, tiae_range_t rAG,
        tiae_range_t rDG, int32_t Tstep, int32_t *pA, int32_t *pT,
        int32_t *pAG, int32_t *pDG)
{
    int32_t Tmin = (rT.min+Tstep-1) / Tstep * Tstep;
    int32_t Tmax = rT.max / Tstep * Tstep;
    if (Tmin > Tmax || Tmin <= 0)
    {
        Tmin = Tmax;
    }
#ifdef AE_DEBUG
    printf("AE debug: dec Tmin=%d, Tmax=%d, Tstep=%d\n", Tmin, Tmax, Tstep);
#endif
    float tm = (float)rA.min * Tmin * rAG.min * rDG.min;
    float tc = (float)cA * cT * cAG * cDG;

    if (tm*1024 <= adjRatio*tc)
    {
        float ex = tc * adjRatio / 1024 / tm;
        search_in_range(ex,
                rA.min, rA.max, pA,
                Tmin, Tmax, Tstep, pT,
                rAG.min, rAG.max, pAG,
                rDG.min, rDG.max, pDG);
        return 1;
    }
    *pA = rA.min;
    *pT = Tmin;
    *pAG = rAG.min;
    *pDG = rDG.min;

    return 0;
}

static int search_range_ATG_inc_gain(int32_t adjRatio,
        int32_t cA, int32_t cT, int32_t cAG, int32_t cDG,
        tiae_range_t rA, tiae_range_t rT, tiae_range_t rAG,
        tiae_range_t rDG, int Tstep, int32_t *pA, int32_t *pT,
        int32_t *pAG, int32_t *pDG)
{
    int32_t Tmin = (rT.min+Tstep-1) / Tstep * Tstep;
    int32_t Tmax = rT.max / Tstep * Tstep;
    if (Tmin > Tmax || Tmin <= 0)
    {
        Tmin = Tmax;
    }
#ifdef AE_DEBUG
    printf("AE debug: inc Tmin=%d, Tmax=%d, Tstep=%d\n", Tmin, Tmax, Tstep);
#endif
    float tm = (float)rA.max * Tmax * rAG.max * rDG.max;
    float tc = (float)cA * cT * cAG * cDG;

    if (tm*1024 >= adjRatio*tc)
    {
        float ex = tc * adjRatio / 1024 / rA.min / Tmin / rAG.min / rDG.min;
        search_in_range(ex,
                rA.min, rA.max, pA,
                Tmin, Tmax, Tstep, pT,
                rAG.min, rAG.max, pAG,
                rDG.min, rDG.max, pDG);
        return 1;
    }
    *pA = rA.max;
    *pT = Tmax;
    *pAG = rAG.max;
    *pDG = rDG.max;

    return 0;
}

/*
* At this point, the average Y value and target Y value are
* used to calculate the adjustment ratio to cur AE settings
* Q10 format is used here to allow enouth accuracy
*/

static void TIAE_adj_exposure(
        int32_t           currentY,
        tiae_exp_t      * prev_exp,
        tiae_exp_t      * next_exp,
        tiae_exp_prog_t * exp_prog)
{
    if (currentY < 1) currentY = 1;
    int32_t adjRatio = exp_prog->target_brightness * 1024 / currentY;
    int32_t delta = 1024/10;
    if (adjRatio > 4096)
    {
        adjRatio = 4096;
    }
    else if (adjRatio < 256)
    {
        adjRatio = 256;
    }
    else if (adjRatio > 2048)
    {
        adjRatio = 2048;
    }
    else if (adjRatio < 512)
    {
        adjRatio = 512;
    }
    else if (adjRatio > 1536)
    {
        adjRatio = 1178;
    }
    else if (adjRatio < 682)
    {
        adjRatio = 890;
    }
    else if (adjRatio > 1024 + delta)
    {
        adjRatio = 1024 + delta;
    }
    else if (adjRatio < 1024 - delta)
    {
        adjRatio = 1024 - delta;
    }

    /* Use the range values to calculate the actual adjustment needed */
    
    int r = 0, i;

    if (adjRatio >= 1024)  //increase gain
    {
        for (i = 0; i < exp_prog->num_ranges; i++)
        {
            r = search_range_ATG_inc_gain(
                    adjRatio,
                    prev_exp->aperture_size,
                    prev_exp->exposure_time,
                    prev_exp->analog_gain,
                    prev_exp->digital_gain,
                    exp_prog->aperture_size_range[i],
                    exp_prog->exposure_time_range[i],
                    exp_prog->analog_gain_range[i],
                    exp_prog->digital_gain_range[i],
                    exp_prog->exposure_time_step_size,
                    &next_exp->aperture_size,
                    &next_exp->exposure_time,
                    &next_exp->analog_gain,
                    &next_exp->digital_gain);
#ifdef AE_DEBUG
            printf("AE debug: i=%d, r=%d, num=%d\n", i, r, exp_prog->num_ranges);
#endif
            if (r == 1)
            {
                break;
            }
        }
    }
    else if (adjRatio < 1024) //decrease gain
    {
        for (i = exp_prog->num_ranges-1; i>=0; i--)
        {
            r = search_range_ATG_dec_gain(
                    adjRatio,
                    prev_exp->aperture_size,
                    prev_exp->exposure_time,
                    prev_exp->analog_gain,
                    prev_exp->digital_gain,
                    exp_prog->aperture_size_range[i],
                    exp_prog->exposure_time_range[i],
                    exp_prog->analog_gain_range[i],
                    exp_prog->digital_gain_range[i],
                    exp_prog->exposure_time_step_size,
                    &next_exp->aperture_size,
                    &next_exp->exposure_time,
                    &next_exp->analog_gain,
                    &next_exp->digital_gain);
#ifdef AE_DEBUG
            printf("AE debug: i=%d, r=%d, num=%d\n", i, r, exp_prog->num_ranges);
#endif
            if (r == 1)
            {
                break;
            }
        }
    }

#ifdef AE_DEBUG
    printf("AE debug: cA, cT, cAG, cDG = %d, %d, %d, %d\n",
            prev_exp->aperture_size, prev_exp->exposure_time, prev_exp->analog_gain, prev_exp->digital_gain);
    printf("AE debug: nA, nT, nAG, nDG = %d, %d, %d, %d, r=%d\n",
            next_exp->aperture_size, next_exp->exposure_time, next_exp->analog_gain, next_exp->digital_gain, r);
#endif

}


