/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#ifndef J7_IMG_AEWB_KERNELS_H_
#define J7_IMG_AEWB_KERNELS_H_

#include <VX/vx.h>
#include <VX/vx_kernels.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file
 * \brief The list of supported kernels in this kernel extension.
 */

/*! \brief Name for OpenVX Extension kernel module: imaging
 * \ingroup group_vision_function_imaging
 */
#define TIVX_MODULE_NAME_IMAGING    "imaging"

/*! \brief The list of kernels supported in imaging module
 *
 * Each kernel listed here can be used with the <tt>\ref vxGetKernelByName</tt> call.
 * When programming the parameters, use
 * \arg <tt>\ref VX_INPUT</tt> for [in]
 * \arg <tt>\ref VX_OUTPUT</tt> for [out]
 * \arg <tt>\ref VX_BIDIRECTIONAL</tt> for [in,out]
 *
 * When programming the parameters, use
 * \arg <tt>\ref VX_TYPE_IMAGE</tt> for a <tt>\ref vx_image</tt> in the size field of <tt>\ref vxGetParameterByIndex</tt> or <tt>\ref vxSetParameterByIndex</tt>
 * \arg <tt>\ref VX_TYPE_ARRAY</tt> for a <tt>\ref vx_array</tt> in the size field of <tt>\ref vxGetParameterByIndex</tt> or <tt>\ref vxSetParameterByIndex</tt>
 * \arg or other appropriate types in \ref vx_type_e.
 * \ingroup group_kernel
 */

/*! \brief aewb kernel name
 *  \see group_vision_function_imaging
 */
#define TIVX_KERNEL_AEWB_NAME     "com.ti.imaging.aewb"

/*! End of group_vision_function_imaging */

/*!
 * \brief Used for the Application to load the imaging kernels into the context.
 * \ingroup group_kernel
 */
void tivxImagingLoadKernels(vx_context context);

/*!
 * \brief Used for the Application to unload the imaging kernels from the context.
 * \ingroup group_kernel
 */
void tivxImagingUnLoadKernels(vx_context context);

/*!
 * \brief Function to register IMAGING Kernels on the aewb Target
 * \ingroup group_vision_function_imaging
 */
void tivxRegisterImagingTargetAewbKernels(void);

/*!
 * \brief Function to un-register IMAGING Kernels on the aewb Target
 * \ingroup group_vision_function_imaging
 */
void tivxUnRegisterImagingTargetAewbKernels(void);

/*!
 * \brief Used to print the performance of the kernels.
 * \ingroup group_kernel
 */
void tivxImagingPrintPerformance(vx_perf_t performance, uint32_t numPixels, const char* testName);


/*!
 * \brief \ref tivx_aewb_config_t parameters
 * \ingroup group_vision_function_imaging
 */
typedef struct
{
    uint16_t sensor_dcc_id; /*!<DCC ID of the sensor. Must correspond to the value specified in tuning XML files */
    uint8_t sensor_img_format; /*!<Image Format : BAYER = 0x0, Rest unsupported */
    uint8_t sensor_img_phase; /*!<Image Format : BGGR = 0, GBRG = 1, GRBG = 2, RGGB = 3 */
    uint8_t awb_mode; /*!<AWB Mode : 0 = Auto, 1 = Manual, 2 = Disabled */
    uint8_t ae_mode; /*!<AE Mode : 0 = Auto, 1 = Manual, 2 = Disabled */
    uint8_t awb_num_skip_frames; /*!<0 = Process every frame */
    uint8_t ae_num_skip_frames; /*!<0 = Process every frame */
    uint8_t channel_id; /*!<channel ID */
}tivx_aewb_config_t;

/**
  \brief  AE/AWB packet format for sum-only mode
*/
  typedef struct {
    uint16_t subSampleAcc[4]; /*!< Accumulator sums for 4 color components */
    uint16_t saturatorAcc[4]; /*!< Number of saturated windows for 4 color components */
 } tivx_h3aAewbOutSumModeOverlay;

/**
  \brief  AE/AWB unsaturated block count
*/
  typedef struct {
    uint16_t unsatCount[8];/*!< Number of unsaturated windows for 4 color components */
  } tivx_h3aAewbOutUnsatBlkCntOverlay;


/**
  \brief  AEWWB Data Entry Structure
*/
 typedef struct _aewDataEntry {
    uint16_t window_data[8][8];/*!< H3A data for a block of 8 windows*/
    uint16_t unsat_block_ct[8];/*!< Number of unstaurated blocks in a block of 8 windows*/
}tivx_aewDataEntry;


/*!
 * \brief \ref tivx_h3a_image_t parameters
 * \ingroup group_vision_function_imaging
 */
#define H3A_MAX_WINH  56
#define H3A_MAX_WINV  128

typedef struct
{
    uint16_t rows;/*!< Window height*/
    uint16_t cols;/*!< Window width*/
    uint8_t bpp;/*!< bits per pixel*/
    uint32_t numPixWin;/*!< Number of pixels in a window*/
    uint16_t rdata[H3A_MAX_WINH*H3A_MAX_WINV];/*!< H3A Data for RED color component*/
    uint16_t gdata[H3A_MAX_WINH*H3A_MAX_WINV];/*!< H3A Data for GREEN color component*/
    uint16_t bdata[H3A_MAX_WINH*H3A_MAX_WINV];/*!< H3A Data for BLUE color component*/
    uint16_t adata[H3A_MAX_WINH*H3A_MAX_WINV];/*!< H3A Data for 4th color component in non RGB format*/
}tivx_h3a_image_t;

/*!
 * \brief \ref tivx_aewb_hist_t parameters
 * \ingroup group_vision_function_imaging
 */
struct tivx_aewb_hist_t
{
    uint32_t histogram[256][32];/*!< Histogram data*/
}tivx_aewb_hist_t;

#define MAX_H3A_HORZ_WINDOWS     H3A_MAX_WINH
#define MAX_H3A_VERT_WINDOWS     H3A_MAX_WINV

#ifdef __cplusplus
}
#endif

#endif /* J7_IMG_AEWB_KERNELS_H_ */


