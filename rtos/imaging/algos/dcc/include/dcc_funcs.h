/*
TEXAS INSTRUMENTS TEXT FILE LICENSE

Copyright (c) [2018] – [2019] Texas Instruments Incorporated

All rights reserved not granted herein.

Limited License.  

Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.

Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution

Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:

*	No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.

*	any redistribution and use are licensed by TI for use only with TI Devices.

*	Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.

If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:

*	any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.

*	any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.

Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.

DISCLAIMER.

THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* =============================================================================
*                                  INCLUDE FILES
* =========================================================================== */

#ifndef __DCC_FUNCS_H__
#define __DCC_FUNCS_H__


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "idcc.h"
#include "dcc_defs.h"

/* =============================================================================
*                                  DEFINES
* =========================================================================== */



/* =============================================================================
*                                  STRUCTURES
* =========================================================================== */



typedef uint32_t  dcc_phspace_vals_t[DCC_PS_DIM_ID_COUNT];

/* =============================================================================
*                                  ENUMS
* =========================================================================== */
typedef enum {
    /* no error */
    DCC_GET_OK,
    /* No DCC component in DCC buff contains such sensor ID, DCC ID and vendor
    ID */
    DCC_GET_ERR_NO_SUCH_DCC,
    /* DCC component found, but it does not contain such a use case*/
    DCC_GET_ERR_NO_SUCH_USE_CASE,
    /* A broken DCC header in DCC buff found - parsing can not proceed.
    srv_process_binary_dcc sets ret_info to the DCC ID of the last valid DCC
    component found*/
    DCC_GET_ERR_INVALID_DCC,
    /* DCC component is found it seems improper or cut. sys_prm or use_case_parm
    or parpack points outside this DCC component binary*/
    DCC_GET_ERR_IMPROPER_DCC,
    /**< Improper Camera ID */
    DCC_GET_ERR_INVALID_CAMERA_ID,
    /**< Invalid Camera id for this usecase */
    DCC_GET_ERR_INVALID_DESC_ID
    /**< Wrong Descriptor id in the binary file */
} DCC_GET_STATUS;

/* =============================================================================
*                                  ROUTINES
* =========================================================================== */

/* ========================================================================== */
/**
*  process_binary_dcc();
*  reads the DCC Profile
*  returns ptr to the Use Case Specific General Parameters
*  for the use case ID passed by the fw3A in use_case_id
*  acts as wrapper for find_parpack();
*
*  @param   filename - the name of the input binary file
*
*  @return  nothing.
*/
/* ========================================================================== */
#ifdef __cplusplus
extern "C"
{
#endif
    DCC_GET_STATUS process_binary_dcc(uint8_t* dcc_buf,
                                  uint32_t dcc_bytes,
                        uint32_t                              camera_module_id,
                        dcc_descriptor_id_type              dcc_descriptor_id,
                        dcc_algorithm_vendor_id_type        algorithm_vendor_id,
                        dcc_use_case_id_type use_case_id,
                        dcc_phspace_vals_t *dim_values,
                        dcc_ptrs_t           *ret_ptrs,
                        uint32_t* ret_info);

#ifdef __cplusplus
}
#endif

#endif //__DCC_FUNCS_H__
