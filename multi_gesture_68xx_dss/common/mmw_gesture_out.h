/**
 *   @file  mmw_gesture_out.h
 *
 *   @brief
 *      Header file for the gesture output interface
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMW_GESTURE_OUT_H
#define MMW_GESTURE_OUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Bit Field definitions for gesture report */

/* Mode indicator bits */
#define GESTURE_MODE_BIT_SHIFT                  0U
#define GESTURE_MODE_BIT_MASK                   0x7U                         

#define GESTURE_ERROR_BIT_SHIFT                 3U
#define GESTURE_ERROR_BIT_MASK                  0x8U

/* Detected Gesture Type*/
#define GESTURE_TYPE_BIT_SHIFT                  1U
#define GESTURE_TYPE_BIT_MASK                   0xFE


/* Enum definitions for gesture mode information */
typedef enum Gesture_mode_type_e
{
    GESTURE_MODE_IDLE = 2,
    GESTURE_MODE_GESTURE = 4
}Gesture_mode_type;

typedef enum Gesture_gesture_type_e
{
    GESTURE_TYPE_NONE = 1,
    GESTURE_TYPE_UP_TO_DOWN = 2,
    GESTURE_TYPE_DOWN_TO_UP = 4,
    GESTURE_TYPE_LEFT_TO_RIGHT = 8,
    GESTURE_TYPE_RIGHT_TO_LEFT = 16,
    GESTURE_TYPE_CCW_ROTATION = 32, /* Counter ClockWise Rotation */
    GESTURE_TYPE_CW_ROTATION  = 64 /* ClockWise Rotation */
}Gesture_gesture_type;

/**
 * @brief
 *  Message for sensing gesture reports (state and detected gesture) over UART
 *
 * @details
 *  The structure defines the message body for the gesture reports sent over UART.
 */
typedef struct GestureDemo_gesture_report_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x12, 0x34, 0x56, 0x78} */
    uint8_t    magicWord[4];
    
    /*! @brief Current mode information
     * Bits 2:0  Mode information
     * Bit  3    Error indication (if set)
     * Bits 7:4  Reserved
     */ 
    uint8_t     mode_ind;

    /*! @brief  Detected gesture information
     * Bits 6:0  Gesture information
     * Bit  7    Reserved
     */
    uint8_t     gesture_type;

    /*! @brief  Padding */
    uint16_t    padding;

} GestureDemo_gesture_report;




#ifdef __cplusplus
}
#endif

#endif /* MMW_GESTURE_OUT_H */

