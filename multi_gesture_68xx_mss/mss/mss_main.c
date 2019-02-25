/**
 *   @file  mss_main.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Gesture Demo
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
 /** @mainpage Millimeter Wave Gesture (mmw-gesture) Demo for XWR68XX
  *
  *  @section intro_sec Introduction
  *
  *  The millimeter wave demo shows some of the gesture recognition
  *  capabilities of the XWR68xx SoC.
  *
  *  Following is a high level description of the features of this demo:
  *  - Detect a set of gestures performed directly in front of the radar.
  *    These gestures are
  *    -# right to left.
  *    -# up to down.
  *  - Presence detection - detect the presence of people nearby.
  *  - Do 1D, 2D, CFAR and Azimuth and elevation processing and optionally
  *    stream out a series of feature vectors. These feature-vectors are
  *    internally passed through a small ANN (neural network) engine to
  *    'infer' gestures.
  *
  *  @section Chirp Design. 
  *
  *	 In this application, unlike the mmw-demo, a hard-coded chirp design is
  *  used. As such, the device on powering up immediately starts sensing 
  *  without waiting for any communication from the host. 
  * 
  *  The parameters for this chirp design were selected to detect relatively 
  *  slow moving objects (like gestures are expected to be). Hence, this 
  *  design has fine range resolution (~5cm), low max-velocity (6m/sec), and 
  *  high velocity resolution (0.1m/s). While the ISR6843 has three 
  *  transmitters, only two of them are utilized in a TDM-MIMO fashion during 
  *  a frame. 
  *
  *  The following table lists all the parameters of interest of the chirp 
  *  design. 
  *  
  *  Parameter          | Value   -| Comment
  *	 ------------------ |----------|--------
  *  Slope				|103 Mhz/us| 
  *	 RF start frequency | 61 Ghz   | The 'knee of the ramp' of the chirp starts at 60.5 Ghz, however, ADC sampling only starts after 6us. 
  *	 RF stop frequency  | 64 Ghz   | 
  *  Bandwidth          | 2.9 Ghz  |
  *  Number of Tx used  | 2		   | Tx 1 and 2 are used in a TDM-MIMO fashion.
  *  Chirp repeat rate  | 105 us   | Since there are two Txs used, the actual chirp repeat rate is 210us. 
  *  Number of chirps   | 256      | Since there are two Txs used, the number of chirps per tx is 128. 
  *  Frame Rate			| 33 Hz	   |
  *  Sampling rate      |2.35 ksps |
  *	 Range resolution   |  5 cm    |
  *  Velocity resolution|  0.1 m/s |
  *  Max-velocity       |  6 m/sec |
  *
  *  The most important parameter for gesture recognition is velocity resolution. 
  *  Having a fine velocity resolution helps distingush fine movements, and 
  *  resolve slow moving objects in the velocity dimension. Once these points 
  *  are resolved in velocity, estimating their range, azimuthal angle, and  
  *  elevation angle becomes simpler. Hence, the chirp design optimized for
  *  velocity resolution. 
  *
  *  @section walk_thru Walkthrough. 
  *  The millimeter wave demo is an application with a lot of moving parts,
  *  most of which are common between the mmw-gesture application and the more
  *  general mmw-demo. They  include items like the drivers (for UART, EDMA, 
  *  etc), the OS, mailbox control ,etc. Documentation regarding the drivers 
  *  and APIs are available in the mmwave SDK and associated collateral. 
  *  
  *  The following walkthrough concerns itself only with the flow of the 
  *  gesture recognition/presence detection application. There are three major 
  *  'c' files that have been modified/added in this demo. The files, the 
  *  processing core (either MSS or DSS) and the primary functionality achieved 
  *  in said file are described in the table below. 
  *
  *  SNo | File              | Core | Functionality
  *	 ----|------------------ |------|--------------
  *	 1.  |dss/dss_data_path.c|DSS   |Performs the basic FMCW processing and generates the doppler-range-antennas radarcube.
  *	 2.  |dss/gesture.c      |DSS   |Generates the feature vectors from the radarcube. 
  *  3.  |mss/mss_main.c     |MSS   |Runs the ANN with the feature vectors from the DSS and performs inference.  
  *
  * Each row of the above table will be elaborated in later sections.
  *
  * @subsection imp_structs Important structures. 
  * 
  *  This section lists the important data structures used in the demo. 
  * 
  *  1. Features_t (used as gfeatures) -> contains limits on the maximum and 
  *     minimum range, maximum and minimum dopplers (both positive and 
  *     negative) that are used to generate feature vectors. It is defined 
  *     in dss/gesture.h.
  * 
  *  2. ANN_struct_t (used as gANN_struct_t) -> contains the weights and biases
  *     as well as the outputs for the ANN. Defined in mss_data_path.h.
  *  
  *  3. MmwDemo_DSS_MCB (used as gMmwDssMCB) -> the main control structure for
  *     DSS. It holds the chirp configuration, and options related to FMCW 
  *     processing. 
  *  
  *  4. MmwDemo_MSS_MCB (used as gMmwMssMCB) -> the main control structure for 
  *     MSS. 
  * 
  * 
  * @subsection fv_ext FMCW processing within the DSS (dss/dss_data_path.c). 
  *	
  *  The FMCW processing performed within the DSS core is a cut-down version of 
  * the processing as done in the mmw-demo, with the primary differences being
  * the following three points. 
  * 
  *  1. In dss_data_path.c::MmwDemo_interFrameProcessing(), once the 2D-FFT is 
  *     complete,  we do not compute the full detection matrix (i.e. the 
  *     logarithm of the absolute value for each range and doppler summed over 
  *     all of the 'virtual' antennas'). In fact, no detection is performed at 
  *     all. Instead, a partial detection matrix (called predetMatrixGesture) 
  *     using only one Rx antenna, and only for the range bins of interest (i.e  
  *     between gfeatures.rangebin_start and gfeatures.rangebin_stop) is    
  *     computed).
  *
  *		The reason why only a single Rx antenna is used is because there was no 
  *     performance degradation with only a single Rx antenna.
  *
  *		The reason for the limited range is because gestures are expected to be
  *		performed close to the Radar. 
  *
  *	 2. Unlike the mmw-demo, the 2D-FFT output is scaled down from 32-bit to 16
  *		bit using the function mmwavelib_scale32to16(), and stored back in the 
  *     same L3 memory (as the 2D-FFT input). This allows us to bypass  
  *		re-computing the 2D-FFT when we have to do angle estimation. 
  *
  *  3. Additionally, the energy in the non-zero velocity bins is collected by
  *	    the variable energyInNonZeroVelBinsPerBucket. Note that since clutter
  *     removal (which removes the zero velocity bin) is enabled by default, 
  *     we sum energy across all the doppler bins to get the non-zero-vel 
  *     energy.
  *
  *  The output of the 2D-FFT and the predetMatrixGesture are then used by the 
  * feature extraction functions (computeFeaturesRDIBased() & computeAngleBas-
  * edStatsAndFeatures()) to extract a series of features. 
  *
  * @subsection fv_ext Feature vector extraction (in dss/gesture.c). 
  *
  *  A classical 'gesture recognition' application using an FMCW radar would 
  * generate the radarcube, and then provide the radar-cube to a trained deep  
  * neural network (or DNN). Such a DNN would be able to extract features 
  * and perform inference. However,  The AWR6843 does not have the necessary 
  * MIPS to run a DNN. In any case, for the gestures that we are interested in,
  * a simpler approach suffices. 
  * 
  *  Our approach extracts from the radarcube a number of features. Each 
  * feature is a number corresponding to a certain measurement. These features
  * are provided to the MSS, where they are buffered. The MSS will then run a 
  * small ANN on these buffers (called  feature-vectors) and provide an 
  * inference as output. 
  *
  *  The list of features is computed over a subset of the radarcube, as we are
  * only interested in objects that lie near the radar (between 
  * gfeatures.rangebin_start, and gfeatures.rangebin_stop). We also disregard
  * very slow/stationary objects by discarding points with low dopplers. The 
  * limits for both range and doppler are set in the function  gestureInit()
  * in dss/gesture.c
  * 
  *  The list of features generated are given below. 
  * 
  *  1. Weighted doppler -> the average doppler (in indices) weighted by the 
  *     bin energy. 
  *  2. Weighted doppler (pos) -> the average doppler weighted by the bin 
  *     energy, computed over positive dopplers. 
  *  3. Weighted doppler (neg) -> the average doppler weighted by the bin 
  *     energy, computed over negative dopplers. 
  *  4. Weighted azimuth -> the weighted average azimuth of the strongest 
  *     NUM_SORTED_VALUES points in the 2D-FFT output (within the range of 
  *     interest).
  *  5. Weighted elevation -> like the weighted azimuth (but for elevation).
  *  6. Number of detections -> the number of points in the predetMatrixGesture
  *		greater than a certain threshold
  *	 7. Std of azimuth     -> the standard deviation of the azimuth (not used)  
  *	 8. Std of eleavtion   -> the standard deviation of the elevation (unused)  
  *  9. Weighted range     -> the weighted average range (within the range of 
  *     interest).
  *
  * Gestures relating to angle (azimuth & elevation) are generated in the 
  * function computeAngleBasedStatsAndFeatures(). Gestures related to range and 
  * doppler are generated in computeFeaturesRDIBased(). 
  *
  * Once these features have been computed, they are provided to the MSS using
  * the function MmwDemo_dssSendProcessOutputToMSS() in dss/dss_main.c
  * 
  * The reason for selecting the 'weighted average' as the representative 
  * feature of every dimension (doppler, range, elevation, azimuth) is that we
  * have seen (by examining the feature-vectors) that they are more than enough 
  * to visually distingush between different simple gestures - moving from 
  * right to left, up to down, vice versa.). 
  * 
  * However, for more complicated gestures more features may be necessary - 
  * for example, if there are two objects (say fingers) moving away and 
  * towards the radar, then features like 'weighted doppler (pos)' will 
  * capture the finger moving away from the radar, whereas 'weighted doppler 
  * (neg)' will capture the finger moving away from the radar. In such a way, 
  * based on the gestures that have to be differentiated, different features 
  * will need to be generated. 
  * 
  * @subsection gest_recog Augmented Features. 
  *
  * Augmented features are additional features that are provided to the ANN. 
  * They are expected to improve the functioning of the ANN by providing easily
  * discriminatable features. 
  * 
  * In our implementation, two augmented features called azimDelta and 
  * elevDelta are generated in the MSS. They are generated using a combination
  * of different feature-vectors. For e.g. azimDelta is generated using 
  * 'weighted doppler' and 'weighted azimuth', and is defined as the distance 
  * between the two inflection points on the 'weighted azimuth' feature vector 
  * centred around the point with the highest 'weighted doppler'. For the 
  * right-to-left swipe gesture, azimDelta output is high for a sustained 
  * period. 
  *
  * These additional features are computed in the function angleDelta().
  *
  * In general, the need for (and the actual perfomance with) augmented 
  * features is not clear. The ANN during training should have been able to  
  * make do without these augmented features, but in our experiments, inference 
  * accuracy seemed toimprove with these additional features. 
  * 
  * There are other augmented features that can be generated. For example, if 
  * one is interested in inferring a 'twirl' gesture (where the finger is 
  * rotated in front of, and perpendicular to, the plane of the radar), one can
  * generate the doppler-azimuth correlation vector - which is simply the 
  * correlation between 'weighted doppler' and 'weighted azimuth'. If the 
  * 'twirl' is in the clockwise direction, this correlation is continuously a
  * large positive value. If the 'twirl' is in the counter-clockwise direction, 
  * this output is continously negative. If any other gesture than a 'twirl' is 
  * performed the output would continously vary. 
  *
  * @subsection gest_recog Gesture recognition. 
  * 
  * Gesture recognition is achieved by passing the feature-vectors (computed in 
  * the DSS, collated and augmented in the MSS) through a three-layer ANN. The
  * implementation of the ANN is given in the function computeInference() from
  * mss/Inference.c. 
  * 
  * The non-linearity/activation function between the first and second layer, 
  * and the second and third layer is relu. The output of the third layer is 
  * passed through a soft-max function to generate the inference output - 
  * i.e probabilities of each gesture. 
  * 
  * These probabilites are logged and passed through a set of heuristics in the
  * function called postProcessing() in mss_main.c. These heuristics count 
  * the number of gesture probabilities that exceed a threshold (currently 0.9), 
  * they check if such events were 'roughly' contiguous, and whether sufficient
  * time has passed since the last gesture. If all such conditions were met, 
  * a valid gesture is declared. 
  *
  * @subsection presence_detection Presence detection. 
  *
  * The final piece of the demo is presence detection, which detects if there is
  * movement in front of the Radar. This is done in the function 
  * runPeopleDetection() by thresholding the energy in non-zero velocities. The
  *  region in front of the radar is divided into four parts, and the energy is 
  * computed for each of these four parts seperately. As such, the radar is  
  * able to ascertain (coarsely) at what range the movement is taking place. 
  * Some amount of hysterisis is also coded, so as to prevent the presence 
  * detection output from changing too rapidly.  
  *    
  */

  /**************************************************************************
   *************************** Include Files ********************************
   **************************************************************************/

   /* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/hsiheader/hsiheader.h>

/* Demo Include Files */
#include <mss_mmw.h>
#include <mss_data_path.h>
#include <mmw_config.h>
#include <mmw_output.h>
#include <mmw_messages.h>
#include <mmw_gesture_out.h>


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

 /**************************************************************************
  *************************** Global Definitions ***************************
  **************************************************************************/

  /**
   * @brief
   *  Global Variable for tracking information required by the mmw Demo
   */
MmwDemo_MCB    gMmwMssMCB;

float gestureProbabilityLog[NUM_NODES_LOG][NUM_NODES_THIRD_LAYER] = { 0.0f };
int32_t gestureTimeStamp[2] = { 0 };

#ifdef DETECT_2x_GESTURES
#define NUM_OUTPUTS  (2 * (NUM_NODES_THIRD_LAYER - 1))
#else
#define NUM_OUTPUTS (NUM_NODES_THIRD_LAYER - 1)
#endif

int32_t gestureCnt[NUM_OUTPUTS] = { 0 };
int32_t gestureCntPrev[NUM_OUTPUTS] = { 0 };
float fltGestureCnt[NUM_OUTPUTS] = {0.0f};
/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
 /* CLI Init function */
extern void MmwDemo_CLIInit(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

 /* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStart(void);
int32_t MmwDemo_mssDataPathStop(void);

/* mmwave library call back fundtions */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void MmwDemo_mssMmwaveCloseCallbackFxn(void);

void MmwDemo_mssMmwaveStopcallbackFxn(void);
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* MMW demo Task */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1);
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1);
void MmwDemo_gpioSwitchTask(UArg arg0, UArg arg1);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* DSS to MSS exception signalling ISR */
static void MmwDemo_installDss2MssExceptionSignallingISR(void);

/* Some additional features to augment the feature vector array. */
void angleDelta(float * ptrAzimDelta, float * ptrElevDelta, float dop[], float azim[], float elev[]);
float findDelta(float arr[], int32_t maxIdx, int32_t negate);



/* global flag to signal transfer of ANN output through UART to external world*/
volatile int8_t send_ANN_OP_to_UART = 0; // when set to 1, the ANN output will be sent to UART

/*Creating buffers to copy features from DSS */
#define HSRAM_BASE_ADDR_DSS  (0x21080000)
#define HSRAM_BASE_ADDR_MSS  (0x52080000)
#define L3RAM_BASE_ADDR_MSS  (0x51000000)
#define L3RAM_BASE_ADDR_DSS  (0x20000000)

float featureVecPosLog[(NUM_CLASSIFIER_FEATURES * FEATURE_LENGTH) + NUM_AUGMENTS ] = { 0 };
#ifdef DETECT_2x_GESTURES
float featureVecNegLog[(NUM_CLASSIFIER_FEATURES * FEATURE_LENGTH) + NUM_AUGMENTS ] = { 0 };
#endif
float energyLog[(NUM_RANGE_BUCKETS_FOR_PERSON_DETECTION * ENERGY_LOG_LENGTH) ] = { 0 };
uint16_t debug_cnt = 0;
uint32_t gesture_pktno = 0;
uint32_t frameNumber = 0;

ANN_struct_t gANN_struct_t = {
#include <ANN_params_60Ghz.h>
};
/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

 /**
  *  @b Description
  *  @n
  *      Registered event function which is invoked when an event from the
  *      BSS is received.
  *
  *  @param[in]  msgId
  *      Message Identifier
  *  @param[in]  sbId
  *      Subblock identifier
  *  @param[in]  sbLen
  *      Length of the subblock
  *  @param[in]  payload
  *      Pointer to the payload buffer
  *
  *  @retval
  *      Always returns 0 [Continue passing the event to the peer domain]
  */
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
	uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

#if 0
	System_printf("Debug: BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
		msgId, sbId, sbLen);
#endif

	/* Process the received message: */
	switch (msgId)
	{
	case RL_RF_ASYNC_EVENT_MSG:
	{
		/* Received Asychronous Message: */
		switch (asyncSB)
		{
		case RL_RF_AE_CPUFAULT_SB:
		{
			/* Post event to datapath task notify BSS events */
			Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
			break;
		}
		case RL_RF_AE_ESMFAULT_SB:
		{
			/* Post event to datapath task notify BSS events */
			Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
			break;
		}
		case RL_RF_AE_INITCALIBSTATUS_SB:
		{
			rlRfInitComplete_t*  ptrRFInitCompleteMessage;
			uint32_t            calibrationStatus;

			/* Get the RF-Init completion message: */
			ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
			calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

			/* Display the calibration status: */
			CLI_write("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
			break;
		}
		case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
		{
			/* This event is not handled on MSS */
			break;
		}
		case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
		{
			/* Increment the statistics for the number of failed reports */
			gMmwMssMCB.stats.numFailedTimingReports++;

			break;
		}
		case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
		{
			/* Increment the statistics for the number of received calibration reports */
			gMmwMssMCB.stats.numCalibrationReports++;

			break;
		}
		case RL_RF_AE_FRAME_END_SB:
		{
			/*Received Frame Stop async event from BSS.
			  No further action required on MSS as it will
			  wait for a message from DSS when it is done (MMWDEMO_DSS2MSS_STOPDONE)*/
			break;
		}
		default:
		{
			System_printf("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
			break;
		}
		}
		break;
	}
	default:
	{
		System_printf("Error: Asynchronous message %d is NOT handled\n", msgId);
		break;
	}
	}
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked after the configuration
 *      has been used to configure the mmWave link and the BSS. This is applicable only for
 *      the XWR16xx. The BSS can be configured only by the MSS *or* DSS. The callback API is
 *      triggered on the remote execution domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
	/* For mmw Demo, mmwave_config() will always be called from MSS,
	   due to the fact CLI is running on MSS, hence this callback won't be called */

	gMmwMssMCB.stats.datapathConfigEvt++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
	return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveCloseCallbackFxn(void)
{
	return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
	/* Post an event to main data path task.
	   This function in only called when mmwave_start() is called on DSS */
	gMmwMssMCB.stats.datapathStartEvt++;

	/* Post event to start is done */
	Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStopCallbackFxn(void)
{
	/* Possible sceanarios:
	   1. CLI sensorStop command triggers mmwave_stop() to be called from MSS
	   2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
	 */
	gMmwMssMCB.stats.datapathStopEvt++;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
	int32_t                  retVal = -1;

	retVal = Mailbox_write(gMmwMssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
	if (retVal == sizeof(MmwDemo_message))
	{
		retVal = 0;
	}
	return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from
 *      Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */

int32_t peopleDetectorState = -1;
float peopleDetectorStateFlt = -1;
int32_t peopleDetectorStatePrev = -1;

int32_t numDetectionsPerBucket[NUM_RANGE_BUCKETS_FOR_PERSON_DETECTION] = {0};
Gesture_mode_type g_mode_type = GESTURE_MODE_IDLE;
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
	MmwDemo_message      message;
	int32_t              retVal = 0;
	float * ptrFeatures;
	float gestureOut;


	/* wait for new message and process all the messages received from the peer */
	while (1)
	{
		Semaphore_pend(gMmwMssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

		/* Read the message from the peer mailbox: We are not trying to protect the read
		 * from the peer mailbox because this is only being invoked from a single thread */
		retVal = Mailbox_read(gMmwMssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
		if (retVal < 0)
		{
			/* Error: Unable to read the message. Setup the error code and return values */
			System_printf("Error: Mailbox read failed [Error code %d]\n", retVal);
		}
		else if (retVal == 0)
		{
			/* We are done: There are no messages available from the peer execution domain. */
			continue;
		}
		else
		{
			/* Flush out the contents of the mailbox to indicate that we are done with the message. This will
			 * allow us to receive another message in the mailbox while we process the received message. */
			Mailbox_readFlush(gMmwMssMCB.peerMailbox);

			/* Process the received message: */
			switch (message.type)
			{
			case MMWDEMO_DSS2MSS_DETOBJ_READY:
				/* Got feature vecctors. Perform inference and then  ship the results and the feature vector out through UART.*/

#if SEND_FEATURE_VECTORS_OUT_ON_UART
                /* Send the  feature vectors. */
				UART_writePolling(gMmwMssMCB.loggingUartHandle,
					(uint8_t*)SOC_translateAddress(message.body.detObj.tlv[0].address,
						SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL),
					message.body.detObj.tlv[0].length);
#endif
                /* collect the features (for the last 10 frames) in a
                 * buffer, and perform inference on them. */
				ptrFeatures = (float*)  SOC_translateAddress(message.body.detObj.tlv[0].address,
						SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);

				/* A. Log features. */
				/* We log the features in two different formats.
				 * 1. With the azimuth and elevation without modification. */
				logFeatures(ptrFeatures, featureVecPosLog, 0);
				/* 2. With the azimuth and elevation inverted. This allows us to re-use the
				 *   ANN trained to detect up-to-down and right-to-left gestures to do
				 *   down-to-up and left-to-right gestures. */
				#ifdef DETECT_2x_GESTURES
				logFeatures(ptrFeatures, featureVecNegLog, -1);
				#endif

				/* B. Log energy (used later for people detection). */
				logEnergy(ptrFeatures, energyLog);

				{
					/* Send a message to DSS to indicate successful transfer of features from
					 * the DSS. If this message is not sent before the start of the next frame
					 * the DSS will halt. */
					memset((void *)&message, 0, sizeof(MmwDemo_message));

					message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

					if (MmwDemo_mboxWrite(&message) != 0)
					{
						System_printf("Error: Mailbox send message id=%d failed \n", message.type);
					}
				}

				/* Pass the feature vector log to an ANN and post process the output to get an
				 * estimate of the current gesture. */
				#ifdef DETECT_2x_GESTURES
				gestureOut = gestureInferenceProcessing(featureVecPosLog, featureVecNegLog, gestureCnt, gestureCntPrev);
				#else
				gestureOut = gestureInferenceProcessing(featureVecPosLog, gestureCnt, gestureCntPrev);
				#endif

				/* People detection. */
				/* 1. Maintain a copy of the previous state so as to
				* transmit the state only if it changes. */
				peopleDetectorStatePrev = peopleDetectorState;
				peopleDetectorState = runPeopleDetection(peopleDetectorState, energyLog);

				peopleDetectorStateFlt = (float) peopleDetectorState;
				fltGestureCnt[UP_TO_DOWN_SWIPE_GESTURE] = (float)gestureCnt[UP_TO_DOWN_SWIPE_GESTURE];
				fltGestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE] = (float)gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE];


#if SEND_INFERENCE_INTERMEDIATE_RESULTS_OUT_ON_UART
				UART_writePolling(gMmwMssMCB.loggingUartHandle,
					(uint8_t*) (&featureVecPosLog[AZIM_DELTA_IDX]),
							2 * sizeof(float) ); // send AzimDelta, and ElevDelta out.

				UART_writePolling(gMmwMssMCB.loggingUartHandle,
					(uint8_t*) (&gestureOut),
							sizeof(int32_t) );
                UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                    (uint8_t*) (gANN_struct_t.prob),
                                            (NUM_NODES_THIRD_LAYER)*sizeof(float) );
                UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                    (uint8_t*) (fltGestureCnt),
                                            (NUM_NODES_THIRD_LAYER - 1)*sizeof(int32_t) );
				UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                    (uint8_t*) (&peopleDetectorStateFlt),
                                            sizeof(float) );
#else
                /* Some un-necessary code to satisfy the compiler. */
                if (gestureOut)
                {
                    gestureOut = 0;
                }
#endif

#if SEND_ONLY_INFERENCE_RESULTS_OUT_ON_UART
                {
#ifdef GESTURE_OUT_HMI
                    GestureDemo_gesture_report gesture_out_msg;



                    /*Construct the gesture report message */
                    memset((void *)&gesture_out_msg, 0, sizeof(GestureDemo_gesture_report));
                    /* Header: */
                    gesture_out_msg.magicWord[0] = 0x12;
                    gesture_out_msg.magicWord[1] = 0x34;
                    gesture_out_msg.magicWord[2] = 0x56;
                    gesture_out_msg.magicWord[3] = 0x78;
                    gesture_out_msg.mode_ind = GESTURE_MODE_IDLE;

                    /* Return the people detector state only if the detector state changes. */
                    if (peopleDetectorState != peopleDetectorStatePrev)
                    {
                        switch(peopleDetectorState)
                        {
                            case -1:
                                //out = 'N';
                                g_mode_type = GESTURE_MODE_IDLE;
                                gesture_out_msg.mode_ind = GESTURE_MODE_IDLE;
                                gesture_out_msg.gesture_type = GESTURE_TYPE_NONE;
                                break;
                            case 0:
                                //out = 'A';
                                g_mode_type = GESTURE_MODE_GESTURE;
                                gesture_out_msg.mode_ind = GESTURE_MODE_GESTURE;
                                break;
                            case 1:
                                //out = 'B';
                                g_mode_type = GESTURE_MODE_GESTURE;
                                gesture_out_msg.mode_ind = GESTURE_MODE_GESTURE;
                                break;
                        }

                    }


                    if(g_mode_type == GESTURE_MODE_GESTURE)
                    {
                        gesture_out_msg.mode_ind = GESTURE_MODE_GESTURE;

                        if (gestureCntPrev[UP_TO_DOWN_SWIPE_GESTURE] != gestureCnt[UP_TO_DOWN_SWIPE_GESTURE])
                        {
                            //gesture_out_msg.gesture_type = GESTURE_TYPE_UP_TO_DOWN;
                            gesture_out_msg.gesture_type = GESTURE_TYPE_RIGHT_TO_LEFT;
                        }
                        else if (gestureCntPrev[RIGHT_TO_LEFT_SWIPE_GESTURE] != gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE])
                        {
                            //gesture_out_msg.gesture_type = GESTURE_TYPE_RIGHT_TO_LEFT;
                            gesture_out_msg.gesture_type = GESTURE_TYPE_UP_TO_DOWN;
                        }
                        else
                        {
                            gesture_out_msg.gesture_type = GESTURE_TYPE_NONE;
                        }

                    }

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)&gesture_out_msg,
                            sizeof(GestureDemo_gesture_report));

#else
                    char out =' ';
                    if (gestureCntPrev[UP_TO_DOWN_SWIPE_GESTURE] != gestureCnt[UP_TO_DOWN_SWIPE_GESTURE])
                    {
                        out = 'U';
                        UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                (uint8_t*) (&out),
                                sizeof(char) );

                    }
                    else if (gestureCntPrev[RIGHT_TO_LEFT_SWIPE_GESTURE] != gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE])
                    {
                        out = 'R';
                        UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                (uint8_t*) (&out),
                                sizeof(char) );
                    }

                    /* Return the people detector state only if the detector state changes. */
                    if (peopleDetectorState != peopleDetectorStatePrev)
                    {
                        switch(peopleDetectorState)
                        {
                            case -1:
                                out = 'N';
                                break;
                            case 0:
                                out = 'A';
                                break;
                            case 1:
                                out = 'B';
                                break;
                            case 2:
                                out = 'C';
                                break;
                            case 3:
                                out = 'D';
                                break;	
                        }
                        UART_writePolling(gMmwMssMCB.loggingUartHandle,
                                (uint8_t*) (&out),
                                sizeof(char) );
                    }	
#endif

                }
#endif
				
				break;
			case MMWDEMO_DSS2MSS_STOPDONE:
				/* Post event that stop is done */
				gMmwMssMCB.stats.dssSensorStop++;
				Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
				break;
			case MMWDEMO_DSS2MSS_ASSERT_INFO:
				/* Send the received DSS assert info through CLI */
				CLI_write("DSS Exception: %s, line %d.\n", message.body.assertInfo.file,
					message.body.assertInfo.line);
				break;
			case MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS:
				gMmwMssMCB.dss2mssIsrInfoAddress =
					SOC_translateAddress(message.body.dss2mssISRinfoAddress,
						SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);
				MmwDemo_installDss2MssExceptionSignallingISR();
				break;
			case MMWDEMO_DSS2MSS_MEASUREMENT_INFO:
				/* Send the received DSS calibration info through CLI */
				CLI_write("compRangeBiasAndRxChanPhase");
				CLI_write(" %.7f", message.body.compRxChanCfg.rangeBias);
				int32_t i;
				for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
				{
					CLI_write(" %.5f", (float)message.body.compRxChanCfg.rxChPhaseComp[i].real / 32768.);
					CLI_write(" %.5f", (float)message.body.compRxChanCfg.rxChPhaseComp[i].imag / 32768.);
				}
				CLI_write("\n");
				break;
			default:
			{
				/* Message not support */
				System_printf("Error: unsupported Mailbox message id=%d\n", message.type);
				break;
			}
			}
		}
	}
}

/**
 *  @b Description
 *  @n
 *      This function is a callback function that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
	Mbox_Handle  handle,
	Mailbox_Type    peer
	)
{
	/* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
	 * the received message. */
	Semaphore_post(gMmwMssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *    Function that configures the BPM chirps based on the stored BPM CLI commands.
 *    The MMW demo supports only a fixed BPM scheme and this scheme is implemented
 *    by this function.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_bpmConfig(void)
{
	uint8_t                subframe;
	uint8_t                numberOfSubframes = 1;
	int32_t                errCode;
	rlBpmChirpCfg_t        bpmChirpCfg;
	MMWave_BpmChirpHandle  bpmChirpHandle;

	if (gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
	{
		/* BPM configuration for all valid sub-frames */
		numberOfSubframes = gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames;
	}

	for (subframe = 0; subframe < numberOfSubframes; subframe++)
	{
		/* Is BPM enabled*/
		if (gMmwMssMCB.cliCfg[subframe].bpmCfg.isEnabled)
		{
			/*configure chirp 0 (++)*/
			memset((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
			bpmChirpCfg.chirpStartIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp0Idx;
			bpmChirpCfg.chirpEndIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp0Idx;
			/* Phase configuration: TX0 positive, TX1 positive*/
			bpmChirpCfg.constBpmVal = 0U;

			bpmChirpHandle = MMWave_addBpmChirp(gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
			if (bpmChirpHandle == NULL)
			{
				System_printf("Error: Unable to add BPM cfg chirp 0. Subframe %d [Error code %d]\n", subframe, errCode);
				return -1;
			}

			/*configure chirp 1 (++)*/
			memset((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
			bpmChirpCfg.chirpStartIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp1Idx;
			bpmChirpCfg.chirpEndIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp1Idx;
			/* Phase configuration: TX0 positive, TX1 negative*/
			bpmChirpCfg.constBpmVal = 0xCU;

			bpmChirpHandle = MMWave_addBpmChirp(gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
			if (bpmChirpHandle == NULL)
			{
				System_printf("Error: Unable to add BPM cfg chirp 1. Subframe %d [Error code %d]\n", subframe, errCode);
				return -1;
			}
		}
		else
		{
			/*BPM is disabled.
			  Configure the range of chirps [chirp0Idx..chirp1Idx]
			  all to have zero phase.*/
			memset((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
			bpmChirpCfg.chirpStartIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp0Idx;
			bpmChirpCfg.chirpEndIdx = gMmwMssMCB.cliCfg[subframe].bpmCfg.chirp1Idx;
			/* Phase configuration: TX0 positive, TX1 positive*/
			bpmChirpCfg.constBpmVal = 0U;

			bpmChirpHandle = MMWave_addBpmChirp(gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
			if (bpmChirpHandle == NULL)
			{
				System_printf("Error: Unable to add BPM cfg for BPM disabled. Subframe %d [Error code %d]\n", subframe, errCode);
				return -1;
			}
		}
	}

	return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to Setup the HSI Clock. Required for LVDS streaming.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssSetHsiClk(void)
{
	rlDevHsiClk_t                           hsiClkgs;
	int32_t                                 retVal;

	/*************************************************************************************
	 * Setup the HSI Clock through the mmWave Link:
	 *************************************************************************************/
	memset((void*)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));

	/* Setup the HSI Clock as per the Radar Interface Document:
	 * - This is set to 600Mhz DDR Mode */
	hsiClkgs.hsiClk = 0x9;

	/* Setup the HSI in the radar link: */
	retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1, &hsiClkgs);
	if (retVal != RL_RET_CODE_OK)
	{
		/* Error: Unable to set the HSI clock */
		System_printf("Error: Setting up the HSI Clock Failed [Error %d]\n", retVal);
		return -1;
	}

	/*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
	  This is needed because the DCA1000EVM FPGA needs the delay to lock to the
	  bit clock before they can start capturing the data correctly. */
	Task_sleep(HSI_DCA_MIN_DELAY_MSEC);

	return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on MSS. After received Configuration from
 *    CLI, this function will start the system configuration process, including mmwaveLink, BSS
 *    and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathConfig(void)
{
	int32_t  errCode;

	/* Has the mmWave module been opened? */
	if (gMmwMssMCB.isMMWaveOpen == false)
	{
		/* Get the open configuration from the CLI mmWave Extension */
		CLI_getMMWaveExtensionOpenConfig(&gMmwMssMCB.cfg.openCfg);

		/* NO: Setup the calibration frequency: */
		gMmwMssMCB.cfg.openCfg.freqLimitLow = 0U;
		gMmwMssMCB.cfg.openCfg.freqLimitHigh = 0U;

		/* Disable the frame start async event so that small chirp times
		   are supported. If this event is enabled it will break real-time
		   for small chirp times and cause 1D processing to crash
		   due to lack of MIPS*/
		gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;

		/* Enable frame stop async event so that we know when BSS has stopped*/
		gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent = false;

                /* No custom calibration: */
                gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
                gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

		/* Open the mmWave module: */
		if (MMWave_open(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
		{
			System_printf("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
			return -1;
		}

		/* mmWave module has been opened. */
		gMmwMssMCB.isMMWaveOpen = true;

		/*Set up HSI clock*/
		if (MmwDemo_mssSetHsiClk() < 0)
		{
			System_printf("Error: MmwDemo_mssSetHsiClk failed.\n");
			return -1;
		}
	}
	else
	{
		/* openCfg related configurations like chCfg, lowPowerMode, adcCfg
		 * are only used on the first sensor start. If they are different
		 * on a subsequent sensor start, then generate a fatal error
		 * so the user does not think that the new (changed) configuration
		 * takes effect, the board needs to be reboot for the new
		 * configuration to be applied.
		 */
		MMWave_OpenCfg openCfg;

		CLI_getMMWaveExtensionOpenConfig(&openCfg);

		/* Initialize to same as in "if" part where open is done
		 * to allow memory compare of structures to be used.
		 * Note that even if structures may have holes, the memory
		 * compare is o.k because CLI always stores the configurations
		 * in the same global CLI structure which is copied over to the
		 * one supplied by the application through the
		 * CLI_getMMWaveExtensionOpenConfig API. Not using memcmp will
		 * require individual field comparisons which is probably
		 * more code size and cumbersome.
		 */
		openCfg.freqLimitLow = 0U;
		openCfg.freqLimitHigh = 0U;
		openCfg.disableFrameStartAsyncEvent = false;
		openCfg.disableFrameStopAsyncEvent = false;

		/* No custom calibration: */
		gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
		gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

		/* Compare openCfg */
		if (memcmp((void *)&gMmwMssMCB.cfg.openCfg, (void *)&openCfg,
			sizeof(MMWave_OpenCfg)) != 0)
		{
			/* Post event to CLI task that start failed so that CLI/GUI can be notified */
			Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);

			MmwDemo_mssAssert(0);
		}
	}

	/* Get the control configuration from the CLI mmWave Extension */
	CLI_getMMWaveExtensionConfig(&gMmwMssMCB.cfg.ctrlCfg);

	/* Prepare BPM configuration */
	if (MmwDemo_bpmConfig() < 0)
	{
		System_printf("Error: MMWDemoMSS mmWave BPM Configuration failed\n");
		return -1;
	}

	/* Configure the mmWave module: */
	if (MMWave_config(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
	{
		System_printf("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
		return -1;
	}

	return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path components including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStart(void)
{
	int32_t                 errCode;
	MMWave_CalibrationCfg   calibrationCfg;

	/* Initialize the calibration configuration: */
	memset((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

	/* Populate the calibration configuration: */
	calibrationCfg.dfeDataOutputMode =
		gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
	calibrationCfg.u.chirpCalibrationCfg.enableCalibration = true;
	calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = true;
	calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

	/* Start the mmWave module: The configuration has been applied successfully. */
	if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
	{
		/* Error: Unable to start the mmWave control */
		System_printf("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
		return -1;
	}
	System_printf("Debug: MMWDemoMSS mmWave Start succeeded \n");
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will
 *    start all data path components including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStop(void)
{
	int32_t             errCode;
	MMWave_ErrorLevel   errorLevel;
	int16_t             mmWaveErrorCode;
	int16_t             subsysErrorCode;
	int32_t             retVal = 0;

	/* Start the mmWave module: The configuration has been applied successfully. */
	if (MMWave_stop(gMmwMssMCB.ctrlHandle, &errCode) < 0)
	{
		/* Error/Warning: Unable to stop the mmWave module */
		MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
		if (errorLevel == MMWave_ErrorLevel_ERROR)
		{
			/* Error: Set the return value to indicate error: */
			System_printf("Error: MMWDemoMSS mmWave Stop failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
			retVal = -1;
		}
		else
		{
			System_printf("Warning: MMWDemoMSS mmWave Stop failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);

			/* Warning: This is treated as a successful stop. */
		}
	}
	return retVal;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
	int32_t errCode;

	while (1)
	{
		/* Execute the mmWave control module: */
		if (MMWave_execute(gMmwMssMCB.ctrlHandle, &errCode) < 0)
			System_printf("Error: mmWave control execution failed [Error code %d]\n", errCode);
	}
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStart(bool doReconfig)
{
	gMmwMssMCB.stats.cliSensorStartEvt++;

	if (doReconfig) {
		/* Post sensorStart event to start reconfig and then start the sensor */
		Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTART_EVT);
	}
	else
	{
		/* Post frameStart event to skip reconfig and just start the sensor */
		Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_FRAMESTART_EVT);
	}
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStop(void)
{
	gMmwMssMCB.stats.cliSensorStopEvt++;

	/* Post sensorSTOP event to notify sensor stop command */
	Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTOP_EVT);
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to
 *      pend for start complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
int32_t MmwDemo_waitSensorStartComplete(void)
{
	UInt          event;
	int32_t       retVal;
	/* Pend on the START NOTIFY event */
	event = Event_pend(gMmwMssMCB.eventHandleNotify,
		Event_Id_NONE,
		MMWDEMO_DSS_START_COMPLETED_EVT | MMWDEMO_DSS_START_FAILED_EVT,
		BIOS_WAIT_FOREVER);

	/************************************************************************
	 * DSS event:: START notification
	 ************************************************************************/
	if (event & MMWDEMO_DSS_START_COMPLETED_EVT)
	{
		/* Sensor has been started successfully */
		gMmwMssMCB.isSensorStarted = true;
		/* Turn on the LED */
		GPIO_write(SOC_XWR68XX_GPIO_2, 1U);
		retVal = 0;
	}
	else if (event & MMWDEMO_DSS_START_FAILED_EVT)
	{
		/* Sensor start failed */
		gMmwMssMCB.isSensorStarted = false;
		retVal = -1;
	}
	else
	{
		/* we should block forever till we get the events. If the desired event
		   didn't happen, then throw an assert */
		retVal = -1;
		MmwDemo_mssAssert(0);
	}
	return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to
 *      pend for stop complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_waitSensorStopComplete(void)
{
	UInt          event;
	/* Pend on the START NOTIFY event */
	event = Event_pend(gMmwMssMCB.eventHandleNotify,
		MMWDEMO_DSS_STOP_COMPLETED_EVT | MMWDEMO_MSS_STOP_COMPLETED_EVT, /* andMask */
		Event_Id_NONE,  /* orMask */
		BIOS_WAIT_FOREVER);

	/************************************************************************
	 * DSS event:: STOP notification
	 ************************************************************************/
	if (event & (MMWDEMO_DSS_STOP_COMPLETED_EVT | MMWDEMO_MSS_STOP_COMPLETED_EVT))
	{
		/* Sensor has been stopped successfully */
		gMmwMssMCB.isSensorStarted = false;

		/* Turn off the LED */
		GPIO_write(SOC_XWR68XX_GPIO_2, 0U);

		/* print for user */
		System_printf("Sensor has been stopped\n");
	}
	else {
		/* we should block forever till we get the event. If the desired event
		   didn't happen, then throw an assert */
		MmwDemo_mssAssert(0);
	}
}



/**
 *  @b Description
 *  @n
 *      Callback function invoked when the GPIO switch is pressed.
 *      This is invoked from interrupt context.
 *
 *  @param[in]  index
 *      GPIO index configured as input
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_switchPressFxn(unsigned int index)
{
	/* Post semaphore to process GPIO switch event */
	Semaphore_post(gMmwMssMCB.gpioSemHandle);
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1)
{
	UInt          event;

	/**********************************************************************
	 * Setup the PINMUX:
	 * - GPIO Input : Configure pin J13 as GPIO_1 input
	 * - GPIO Output: Configure pin K13 as GPIO_2 output
	 **********************************************************************/
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINJ13_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINJ13_PADAC, SOC_XWR68XX_PINJ13_PADAC_GPIO_1);
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);

	/**********************************************************************
	 * Setup the SW1 switch on the EVM connected to GPIO_1
	 * - This is used as an input
	 * - Enable interrupt to be notified on a switch press
	 **********************************************************************/
	GPIO_setConfig(SOC_XWR68XX_GPIO_1, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING | GPIO_CFG_IN_INT_LOW);
	GPIO_setCallback(SOC_XWR68XX_GPIO_1, MmwDemo_switchPressFxn);
	GPIO_enableInt(SOC_XWR68XX_GPIO_1);

	/**********************************************************************
	 * Setup the DS3 LED on the EVM connected to GPIO_2
	 **********************************************************************/
	GPIO_setConfig(SOC_XWR68XX_GPIO_2, GPIO_CFG_OUTPUT);

	/* Data Path management task Main loop */
	while (1)
	{
		event = Event_pend(gMmwMssMCB.eventHandle,
			Event_Id_NONE,
			MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS |
			MMWDEMO_DSS_EXCEPTION_EVENTS,
			BIOS_WAIT_FOREVER);

		/************************************************************************
		 * CLI event:: SensorStart
		 ************************************************************************/

		if (event & MMWDEMO_CLI_SENSORSTART_EVT)
		{
			System_printf("Debug: MMWDemoMSS Received CLI sensorStart Event\n");

			/* Setup the data path: */
			if (gMmwMssMCB.isSensorStarted == false)
			{
				if (MmwDemo_mssDataPathConfig() < 0) {
					/* Post start failed event; error printing is done in function above */
					Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);
					continue;
				}
				else
				{
					/* start complete event is posted via DSS */
				}
			}
			else
			{
				/* Post start complete event as this is just a duplicate command */
				Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
				continue;
			}
		}

		/************************************************************************
		 * CLI event:: SensorStop
		 ************************************************************************/
		if (event & MMWDEMO_CLI_SENSORSTOP_EVT)
		{
			if (gMmwMssMCB.isSensorStarted == true)
			{
				if (MmwDemo_mssDataPathStop() < 0) {
					/* Do we need stop fail event? */
					DebugP_assert(0);
				}
				else
				{
					/* DSS will post the stop completed event */
				}
			}
			else
			{
				/* Post data path stop complete event on behalf on Datapath in case sensor is already in stop state */
				Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
			}

			/* In all cases send MSS stop complete */
			Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_MSS_STOP_COMPLETED_EVT);
		}

		/************************************************************************
		 * CLI event:: Framestart
		 ************************************************************************/
		if (event & MMWDEMO_CLI_FRAMESTART_EVT)
		{
			/* error printing happens inside this function */
			if (gMmwMssMCB.isSensorStarted == false)
			{
				if (MmwDemo_mssDataPathStart() < 0) {
					/* Post start failed event; error printing is done in function above */
					Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);
					continue;
				}
			}

			/* Post event to start is done */
			Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
		}

		/************************************************************************
		 * BSS event:: CPU fault
		 ************************************************************************/
		if (event & MMWDEMO_BSS_CPUFAULT_EVT)
		{
			MmwDemo_mssAssert(0);
			break;
		}

		/************************************************************************
		 * BSS event:: ESM fault
		 ************************************************************************/
		if (event & MMWDEMO_BSS_ESMFAULT_EVT)
		{
			MmwDemo_mssAssert(0);
			break;
		}

		/************************************************************************
		 * BSS event:: Analog fault
		 ************************************************************************/
		if (event & MMWDEMO_BSS_ANALOGFAULT_EVT)
		{
			MmwDemo_mssAssert(0);
			break;
		}

		if (event & MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT)
		{
			CLI_write("DSS Chirp Processing Deadline Miss Exception.\n");
			DebugP_assert(0);
			break;
		}

		if (event & MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT)
		{
			CLI_write("DSS Frame Processing Deadline Miss Exception.\n");
			DebugP_assert(0);
			break;
		}
	}

	System_printf("Debug: MMWDemoDSS Data path exit\n");
}

/**
 *  @b Description
 *  @n
 *      The task is used to process events related
 *      to pressing the GPIO switch
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_gpioSwitchTask(UArg arg0, UArg arg1)
{
	/* wait for GPIO switch event */
	while (1)
	{
		Semaphore_pend(gMmwMssMCB.gpioSemHandle, BIOS_WAIT_FOREVER);
		/* Was the sensor started? */
		if (gMmwMssMCB.isSensorStarted == true)
		{
			/* YES: We need to stop the sensor now */
			MmwDemo_notifySensorStop();
			/* Pend for completion */
			MmwDemo_waitSensorStopComplete();
		}
		else
		{
			/* NO: We need to start the sensor now. */
			MmwDemo_notifySensorStart(true);
			/* Pend for completion */
			if (MmwDemo_waitSensorStartComplete() == -1)
			{
				/* Sensor start failed */
				MmwDemo_mssAssert(0);
			}
		}
	}
}

/**
 *  @b Description
 *  @n
 *      DSS to MSS ISR used for direct signalling of things like urgent exception
 *      events from DSS. Posts deadline miss events to @ref MmwDemo_mssCtrlPathTask.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_Dss2MssISR(uintptr_t arg)
{
	switch (*(uint8_t*)gMmwMssMCB.dss2mssIsrInfoAddress)
	{
	case MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION:
		Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT);
		break;

	case MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION:
		Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT);
		break;

	default:
		MmwDemo_mssAssert(0);
		break;
	}
}

/**
 *  @b Description
 *  @n
 *      Installs DSS to MSS Software Interrupt ISR for exception signalling.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_installDss2MssExceptionSignallingISR(void)
{
	HwiP_Params  hwiParams;
	volatile HwiP_Handle  hwiHandle;

	HwiP_Params_init(&hwiParams);
	hwiParams.name = "Dss2MssSwISR";
	hwiHandle = HwiP_create(MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS,
		MmwDemo_Dss2MssISR, &hwiParams);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1)
{
	int32_t             errCode;
	MMWave_InitCfg      initCfg;
	UART_Params         uartParams;
	Task_Params         taskParams;
	Semaphore_Params    semParams;
	Mailbox_Config      mboxCfg;
	Error_Block         eb;

	/* Debug Message: */
	System_printf("Debug: MMWDemoMSS Launched the Initialization Task\n");

	/*****************************************************************************
	 * Initialize the mmWave SDK components:
	 *****************************************************************************/
	 /* Pinmux setting */

	 /* Setup the PINMUX to bring out the UART-1 */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

	/* Setup the PINMUX to bring out the UART-3 */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

	/* Setup the PINMUX to bring out the DSS UART */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINP8_PADBM, SOC_XWR68XX_PINP8_PADBM_DSS_UART_TX);

	/* Initialize the UART */
	UART_init();

	/* Initialize the GPIO */
	GPIO_init();

	/* Initialize the Mailbox */
	Mailbox_init(MAILBOX_TYPE_MSS);

	/*****************************************************************************
	 * Open & configure the drivers:
	 *****************************************************************************/

	 /* Setup the default UART Parameters */
	UART_Params_init(&uartParams);
	uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
	uartParams.baudRate = gMmwMssMCB.cfg.commandBaudRate;
	uartParams.isPinMuxDone = 1U;

	/* Open the UART Instance */
	gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
	if (gMmwMssMCB.commandUartHandle == NULL)
	{
		System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
		return;
	}

	/* Setup the default UART Parameters */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
	uartParams.baudRate = gMmwMssMCB.cfg.loggingBaudRate;
	uartParams.isPinMuxDone = 1U;

	/* Open the Logging UART Instance: */
	gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
	if (gMmwMssMCB.loggingUartHandle == NULL)
	{
		System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
		return;
	}

	/* Create a binary semaphore which is used to handle GPIO switch interrupt. */
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	gMmwMssMCB.gpioSemHandle = Semaphore_create(0, &semParams, NULL);

	/*****************************************************************************
	 * Creating communication channel between MSS & DSS
	 *****************************************************************************/

	 /* Create a binary semaphore which is used to handle mailbox interrupt. */
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	gMmwMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

	/* Setup the default mailbox configuration */
	Mailbox_Config_init(&mboxCfg);

	/* Setup the configuration: */
	mboxCfg.chType = MAILBOX_CHTYPE_MULTI;
	mboxCfg.chId = MAILBOX_CH_ID_0;
	mboxCfg.writeMode = MAILBOX_MODE_BLOCKING;
	mboxCfg.readMode = MAILBOX_MODE_CALLBACK;
	mboxCfg.readCallback = &MmwDemo_mboxCallback;

	/* Initialization of Mailbox Virtual Channel  */
	gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
	if (gMmwMssMCB.peerMailbox == NULL)
	{
		/* Error: Unable to open the mailbox */
		System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
		return;
	}

	/* Create task to handle mailbox messges */
	Task_Params_init(&taskParams);
	taskParams.stackSize = 16 * 1024;
	Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

	/*****************************************************************************
	 * Create Event to handle mmwave callback and system datapath events
	 *****************************************************************************/
	 /* Default instance configuration params */
	Error_init(&eb);
	gMmwMssMCB.eventHandle = Event_create(NULL, &eb);
	if (gMmwMssMCB.eventHandle == NULL)
	{
		MmwDemo_mssAssert(0);
		return;
	}

	Error_init(&eb);
	gMmwMssMCB.eventHandleNotify = Event_create(NULL, &eb);
	if (gMmwMssMCB.eventHandleNotify == NULL)
	{
		MmwDemo_mssAssert(0);
		return;
	}

	/*****************************************************************************
	 * mmWave: Initialization of the high level module
	 *****************************************************************************/

	 /* Initialize the mmWave control init configuration */
	memset((void*)&initCfg, 0, sizeof(MMWave_InitCfg));

	/* Populate the init configuration for mmwave library: */
	initCfg.domain = MMWave_Domain_MSS;
	initCfg.socHandle = gMmwMssMCB.socHandle;
	initCfg.eventFxn = MmwDemo_mssMmwaveEventCallbackFxn;
	initCfg.linkCRCCfg.useCRCDriver = 1U;
	initCfg.linkCRCCfg.crcChannel = CRC_Channel_CH1;
	initCfg.cfgMode = MMWave_ConfigurationMode_FULL;
	initCfg.executionMode = MMWave_ExecutionMode_COOPERATIVE;
	initCfg.cooperativeModeCfg.cfgFxn = MmwDemo_mssMmwaveConfigCallbackFxn;
	initCfg.cooperativeModeCfg.openFxn = MmwDemo_mssMmwaveOpenCallbackFxn;
	initCfg.cooperativeModeCfg.closeFxn = MmwDemo_mssMmwaveCloseCallbackFxn;
	initCfg.cooperativeModeCfg.startFxn = MmwDemo_mssMmwaveStartCallbackFxn;
	initCfg.cooperativeModeCfg.stopFxn = MmwDemo_mssMmwaveStopCallbackFxn;

	/* Initialize and setup the mmWave Control module */
	gMmwMssMCB.ctrlHandle = MMWave_init(&initCfg, &errCode);
	if (gMmwMssMCB.ctrlHandle == NULL)
	{
		/* Error: Unable to initialize the mmWave control module */
		System_printf("Error: MMWDemoMSS mmWave Control Initialization failed [Error code %d]\n", errCode);
		return;
	}
	System_printf("Debug: MMWDemoMSS mmWave Control Initialization was successful\n");

	/* Synchronization: This will synchronize the execution of the control module
	 * between the domains. This is a prerequiste and always needs to be invoked. */
	while (1)
	{
		int32_t syncStatus;

		/* Get the synchronization status: */
		syncStatus = MMWave_sync(gMmwMssMCB.ctrlHandle, &errCode);
		if (syncStatus < 0)
		{
			/* Error: Unable to synchronize the mmWave control module */
			System_printf("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
			return;
		}
		if (syncStatus == 1)
		{
			/* Synchronization acheived: */
			break;
		}
		/* Sleep and poll again: */
		Task_sleep(1);
	}

	/*****************************************************************************
	 * Launch the mmWave control execution task
	 * - This should have a higher priroity than any other task which uses the
	 *   mmWave control API
	 *****************************************************************************/
	Task_Params_init(&taskParams);
	taskParams.priority = 6;
	taskParams.stackSize = 3 * 1024;
	Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

	/*****************************************************************************
	 * Create a data path management task to handle data Path events
	 *****************************************************************************/
	Task_Params_init(&taskParams);
	taskParams.priority = 2;
	taskParams.stackSize = 3 * 1024;
	Task_create(MmwDemo_mssCtrlPathTask, &taskParams, NULL);

	/*****************************************************************************
	 * Create a GPIO switch task to handle events related to pressing GPIO switch
	 *****************************************************************************/
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 1024;
	Task_create(MmwDemo_gpioSwitchTask, &taskParams, NULL);

	/*****************************************************************************
	 * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
	 * Start CLI to get configuration from user
	 *****************************************************************************/
	MmwDemo_CLIInit();

	/*****************************************************************************
	 * Benchmarking Count init
	 *****************************************************************************/
	 /* Configure banchmark counter */
	Pmu_configureCounter(0, 0x11, FALSE);
	Pmu_startCounter(0);

	return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
	/* issue WFI (Wait For Interrupt) instruction */
	asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Send MSS assert information through CLI.
 */
void _MmwDemo_mssAssert(int32_t expression, const char *file, int32_t line)
{
	if (!expression) {
		CLI_write("MSS Exception: %s, line %d.\n", file, line);
	}
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main(void)
{
	Task_Params     taskParams;
	int32_t         errCode;
	SOC_Cfg         socCfg;


	/* Initialize the ESM: */
	ESM_init(0U); //dont clear errors as TI RTOS does it

	/* Initialize and populate the demo MCB */
	memset((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

	/* Initialize the SOC confiugration: */
	memset((void *)&socCfg, 0, sizeof(SOC_Cfg));

	/* Populate the SOC configuration: */
	socCfg.clockCfg = SOC_SysClock_INIT;

	/* Initialize the SOC Module: This is done as soon as the application is started
	 * to ensure that the MPU is correctly configured. */
	gMmwMssMCB.socHandle = SOC_init(&socCfg, &errCode);
	if (gMmwMssMCB.socHandle == NULL)
	{
		System_printf("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
		return -1;
	}

	/* Initialize the DEMO configuration: */
	gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
	gMmwMssMCB.cfg.loggingBaudRate = 921600;
	gMmwMssMCB.cfg.commandBaudRate = 115200;

	/* Check if the SOC is a secure device */
	if (SOC_isSecureDevice(gMmwMssMCB.socHandle, &errCode))
	{
		/* Disable firewall for JTAG and LOGGER (UART) which is needed by the demo */
		SOC_controlSecureFirewall(gMmwMssMCB.socHandle,
			(uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER),
			SOC_SECURE_FIREWALL_DISABLE,
			&errCode);
	}

	/* Debug Message: */
	System_printf("**********************************************\n");
	System_printf("Debug: Launching the Millimeter Wave Demo\n");
	System_printf("**********************************************\n");

	/* Initialize the Task Parameters. */
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2 * 1024;
	Task_create(MmwDemo_mssInitTask, &taskParams, NULL);

	/* Start BIOS */
	BIOS_start();
	return 0;
}


/**
 *  @b Description
 *  @n
 *      Once the inference module has completed, this function copies them into a
 *      buffer that holds the last NUM_NODES_LOG probabilities.
 *      Each probability (corresponding to a gesture output) in the buffer is compared
 *      against a threshold (GESTURE_INFERENCE_THRESHOLD) and a decision is made.
 *
 *      A final heuristics gates whether a valid gesture has occurred. This heuristic
 *      merely counts how many contiguous gestures have been seen (of one type) and if
 *      said count has crossed a threshold and whether a period of time (given by
 *      INTER_GESTURE_MIN_SEPARATION_FRAMES frames) has passed between successive
 *      gestures.
 *
 *      The number of gestures that have occured since boot is also maintained.
 *
 *  @retval
 *      Not Applicable.
 */

int32_t postProcessing(float prob[], float prob_log[][NUM_NODES_THIRD_LAYER], int32_t gestureTimeStamp[], int32_t gestureCnt[], int32_t gestureCntPrev[], int32_t maxCheckFlag)
{
	int32_t logIdx, probIdx, n1, n2, gestureOut, n1_num_non_contiguous, n2_num_non_contiguous, interGestureThreshold;
	gestureTimeStamp[iCURR]++;

	/* Update the Note the previous count of gestures. The gestureCnt array is updated in the postProcessing
	 * function. */
	gestureCntPrev[UP_TO_DOWN_SWIPE_GESTURE] = gestureCnt[UP_TO_DOWN_SWIPE_GESTURE];
	gestureCntPrev[RIGHT_TO_LEFT_SWIPE_GESTURE] = gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE];

	/* 1. Some book-keeping. */
	if (gestureTimeStamp[iCURR] == INT32_MAX)
	{
		gestureTimeStamp[iCURR] = 0;
		gestureTimeStamp[iPREV] = 0;
	}

	if (gestureCnt[UP_TO_DOWN_SWIPE_GESTURE] == INT32_MAX)
	{
		gestureCnt[UP_TO_DOWN_SWIPE_GESTURE] = 0;
	}

	if (gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE] == INT32_MAX)
	{
		gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE] = 0;
	}

	/* 2. Shift out the oldest set of probabilities from the log. */
	for (logIdx = 1; logIdx < NUM_NODES_LOG; logIdx ++)
	{

		for (probIdx = 0; probIdx < NUM_NODES_THIRD_LAYER; probIdx++)
		{
			prob_log[logIdx - 1][probIdx] = prob_log[logIdx][probIdx];
		}
	}

	/* 3. Inserting the newest probability n the log. */
	for (probIdx = 0; probIdx < NUM_NODES_THIRD_LAYER; probIdx++)
	{
		prob_log[NUM_NODES_LOG - 1][probIdx] = prob[probIdx];
	}

	/* 4. Count the number of (semi-) contiguous gesture probablities
	 *    (of a single type) that exceed a threshold.
	 *    n1 and n2 correspond to counts of gestures 1, and 2.
	 *    nx_num_non_contiguous measure is used to check if
	 *    nx increments each for each frame. */
	n1 = 0;
	n2 = 0;
	n1_num_non_contiguous = 0;
	n2_num_non_contiguous = 0;
	for (logIdx = 0; logIdx < NUM_NODES_LOG; logIdx++)
	{
	    if (prob_log[logIdx][RIGHT_TO_LEFT_SWIPE_GESTURE] > GESTURE_INFERENCE_THRESHOLD)
		{
			n1++;n1_num_non_contiguous = 0;
		}

		if ((n1_num_non_contiguous > 2) && ( (n1 > 0) && (n1 < 2)) )
		{
			n1--;
		}

		if (prob_log[logIdx][UP_TO_DOWN_SWIPE_GESTURE] > GESTURE_INFERENCE_THRESHOLD)
		{
			n2++;n2_num_non_contiguous = 0;
		}

		if  ((n2_num_non_contiguous > 2) && ((n2 > 0) && (n2 < 2)) )
		{
			n2--;
		}

		n1_num_non_contiguous++;
		n2_num_non_contiguous++;

	}

	interGestureThreshold = INTER_GESTURE_MIN_SEPARATION_FRAMES;

	/* 4. Make a decision, based on the counts of each gesture. */
	gestureOut = 0;
	if ((n1 >= 2) && (n1 >= n2))
	{
		if (((gestureTimeStamp[iCURR] - gestureTimeStamp[iPREV]) > interGestureThreshold) && (maxCheckFlag))
		{
			gestureOut = 4;
			gestureCnt[RIGHT_TO_LEFT_SWIPE_GESTURE]++;
			gestureTimeStamp[iPREV] = gestureTimeStamp[iCURR];
		}

	}
	else if ((n2 >= 2) && (n2 > n1))
	{
		if (((gestureTimeStamp[iCURR] - gestureTimeStamp[iPREV]) > interGestureThreshold) && (maxCheckFlag))
		{
			gestureOut = 2;
		    gestureCnt[UP_TO_DOWN_SWIPE_GESTURE]++;
			gestureTimeStamp[iPREV] = gestureTimeStamp[iCURR];
		}
	}

	return gestureOut;
}



/**
 *  @b Description
 *  @n
 *     A function to perform three operations, and thereby extract a more meaningful
 *     'change in azimuth/elevation' estimate. -
 *       1. Find the location of the peak (call it maxIdx) in the doppler log.
 *       2. Find the nearest two 'inflection points' (on the left and the right) of
 *          maxIdx in both the azimuth/elevation log. 'Inflection points' are defined as
 *          those points where the sign of the slope changes.
 * 			Note that the 'inflection points' computation function require an assumption
 *          on the direction of motion - so, since we only seek to detect up-to-down and
 * 			right-to-left gestures, those directions are assumed.
 * 	     3. Compute the distance (in indices) between the two 'inflection points' for
 *          both elevation and azimuth.
 *
 *     The reasoning behind this computation is that if there is significant movement in
 *     one dimension (say up-to-down), then we should see a continuous 'monotonic' change
 *     in that dimension, hence the distance between the 'inflection points' would be
 *     large.
 *
 *  @retval
 *      Not Applicable.
 */
void angleDelta(float * ptrAzimDelta, float * ptrElevDelta, float dop[], float azim[], float elev[])
{
	int featureIdx;
	float maxVal = 0.0f, currAbsVal;
	int maxIdx = -1;

	float azim_delta = 0;
	float elev_delta = 0;

	for (featureIdx = 0; featureIdx < FEATURE_LENGTH; featureIdx++)
	{
		currAbsVal = fabsf(dop[featureIdx]);
		if (currAbsVal > maxVal)
		{
			maxVal = currAbsVal;
			maxIdx = featureIdx;
		}
	}

	if ((maxIdx <= ((FEATURE_LENGTH / 2) + 2 - 1)) &&
		(maxIdx >= ((FEATURE_LENGTH / 2) - 2 - 1)))
	{
		azim_delta = findDelta(azim, maxIdx, 0);
		elev_delta = findDelta(elev, maxIdx, 1); //TODO REVERT TO PREVIOUS
	}

	*ptrAzimDelta = azim_delta;
	*ptrElevDelta = elev_delta;

}

/**
 *  @b Description
 *  @n
 *     A function to compute the closest two 'inflection points' around a given
 *      and then compute the distance (in indices) between them.
 *
 *  @retval
 *      Not Applicable.
 */
float findDelta(float arr[], int32_t maxIdx, int32_t negate)
{
	float dLeft;
	int currIdx;
	float sumLeft = 0.0f;
	for (currIdx = maxIdx; currIdx > 0; currIdx--)
	{
		dLeft = arr[currIdx - 1] - arr[currIdx];
		if (negate)
		{
			dLeft = -dLeft;
		}

		if (dLeft > 0)
		{
			break;
		}
		sumLeft += dLeft;

	}

	float dRight;
	float sumRight = 0.0f;
	for (currIdx = maxIdx; currIdx < FEATURE_LENGTH - 1; currIdx++)
	{
		dRight = arr[currIdx + 1] - arr[currIdx];
		if (negate)
		{
			dRight = -dRight;
		}

		if (dRight < 0)
		{
			break;
		}
		sumRight += dRight;

	}

	return (sumLeft - sumRight);
}


/**
 *  @b Description
 *  @n
 *     A function to detect the presence of a person (or more honestly
 *     movement) using the energy in the non-zero velocity bins. (Although,
 *     since we've enabled clutter removal, all doppler bins correspond to
 * 		non-zero velocities).
 *
 *     Whether movement has been detected or not is held in a variable called
 *     'peopleDetectorState'. If the state is negative no one has been
 *     detected. If the state is non-negative, movement has been detected in a
 *     specific range around the device. The state then corresponds to the
 *     closest range where movement has been detected.
 *
 * 	   Given the 'peopleDetectorState', it is possible to compute the range
 *     where movement has been detected using the following formula
 *
 * 		if (peopleDetectorState >= 0)
 * 		{
 *	 		startdist = peopleDetectorState*(numRangeBins/numBuckets)*rangeRes
 *   		stopdist  = (peopleDetectorState+1)*(numRangeBins/numBuckets)*rangeRes
 * 		}
 *
 *
 *  @retval
 *      peopleDetectorState.
 */
int32_t runPeopleDetection(int32_t peopleDetectorState, float energyLog[])
{
	int32_t bucketIdx;
	int32_t maxNumDet = 0;
	int32_t maxNumBucketDetIdx = -1;
	int32_t offset, itemIdx;

	/* 2. From the given energy buckets (each of which correspond to the energy
	 * in non-zero velocity bins for a particular range) in front of the radar,
	 * check if any exceed a set threshold. */
	for (bucketIdx = 0; bucketIdx < NUM_RANGE_BUCKETS_FOR_PERSON_DETECTION; bucketIdx++)
	{
		float thresh = 16500.0f;

		offset = bucketIdx*ENERGY_LOG_LENGTH;
		numDetectionsPerBucket[bucketIdx] = 0;

		/* What is the number of buckets with significant energy?. */
		for (itemIdx = 0; itemIdx < ENERGY_LOG_LENGTH; itemIdx++)
		{
			if (energyLog[offset + itemIdx] > thresh)
			{
				numDetectionsPerBucket[bucketIdx] ++;
			}
		}

		/* What is the bucket with the most detections?. */
		if (numDetectionsPerBucket[bucketIdx] > maxNumDet)
		{
			maxNumDet = numDetectionsPerBucket[bucketIdx];
			maxNumBucketDetIdx = bucketIdx;
		}
	}

	/* 3. If there are no detections reset the state, else see if there are
	 * enough detection to maintain the current state. */
	if (maxNumBucketDetIdx == -1)
	{
		peopleDetectorState = -1;
	}
	else
	{
		if (peopleDetectorState > 0)
		{
			/* State maintenance.
			   Keep the current state, unless a closer state has more energy. */
			for (bucketIdx = 0; bucketIdx < peopleDetectorState; bucketIdx++)
			{
				if (numDetectionsPerBucket[bucketIdx] >= numDetectionsPerBucket[peopleDetectorState])
				{
					peopleDetectorState = bucketIdx;
				}
			}

			if (numDetectionsPerBucket[peopleDetectorState] >= NUM_DETECTIONS_FOR_MAINTENANCE )
			{
				/* Maintain state. */
			}
			else
			{
				/* Reset. */
				peopleDetectorState = -1;
			}
		}

		/* If the state is unpopulated, check if there are enough detections for a new initialisation. */
		if ((peopleDetectorState < 0) && (maxNumDet >= NUM_DETECTIONS_FOR_INIT))
		{
			peopleDetectorState = maxNumBucketDetIdx;
		}
	}

	return peopleDetectorState;
}

/**
 *  @b Description
 *  @n
 *     A function to copy the current frame's feature vectors into a log. This
 *     log is then passed on to the Inference engine.  The oldest set of
 *     feature vector is deleted from the log, simultaneously.
 *
 *  @retval
 *      Not Applicable.
 */
void logFeatures(float * ptrFeatures, float *featureVecLog, int32_t negateAngleMeasurements)
{
	/* Of the features send from the DSS, we are only interested in a subset.
	 * The array featureList[] holds that subset. */
	int32_t featureList[NUM_CLASSIFIER_FEATURES] = {DSS_FEATURE_IDX_WTDOPPLER,
													DSS_FEATURE_IDX_WTDOPPLERPOS,
													DSS_FEATURE_IDX_WTDOPPLERNEG,
													DSS_FEATURE_IDX_WTAZIM,
													DSS_FEATURE_IDX_WTELEV,
													DSS_FEATURE_IDX_NUMDET,
													DSS_FEATURE_IDX_WTRANGE};
	int32_t featureIdx, itemIdx, offset;
	for (featureIdx = 0; featureIdx < NUM_CLASSIFIER_FEATURES; featureIdx++)
	{
		// Shift out the oldest feature vector set.
		offset = featureIdx*FEATURE_LENGTH;
		for (itemIdx = 0; itemIdx < FEATURE_LENGTH - 1; itemIdx++)
		{
			featureVecLog[offset + itemIdx] = featureVecLog[offset + itemIdx + 1U];
		}
		// add in the newest feature vector set.
		// We optionally negate the azimuth and elevation angles in order to reuse
		if  ( (negateAngleMeasurements == -1) &&
			  ( (featureList[featureIdx] == DSS_FEATURE_IDX_WTAZIM) || (featureList[featureIdx] == DSS_FEATURE_IDX_WTAZIM) ) )
		{
			featureVecLog[offset + FEATURE_LENGTH - 1U] = -ptrFeatures[featureList[featureIdx] + FEATURE_START_IDX];
		}
		else
		{
			featureVecLog[offset + FEATURE_LENGTH - 1U] = ptrFeatures[featureList[featureIdx] + FEATURE_START_IDX];
		}
	}

}

/**
 *  @b Description
 *  @n
 *     A function to copy the current frame's energy measurements into a log. This
 *     log is then passed on to the 'people detection' engine.  The oldest set of
 *     energy measurements are also deleted from the log, simultaneously.
 *
 *     The energy measurements come as part of the feature vector array from the
 *     DSS.
 *
 *  @retval
 *      Not Applicable.
 */

void logEnergy(float * ptrFeatures, float * energyLog)
{
	int32_t idx, itemIdx, offset;
	for (idx = 0; idx < NUM_RANGE_BUCKETS_FOR_PERSON_DETECTION; idx++)
	{
		// Shift out the oldest set of energy measurements.
		offset = idx*ENERGY_LOG_LENGTH;
		for (itemIdx = 0; itemIdx < ENERGY_LOG_LENGTH - 1; itemIdx++)
		{
			energyLog[offset + itemIdx] = energyLog[offset + itemIdx + 1U];
		}
		// add in the newest energy measurements
		energyLog[offset + ENERGY_LOG_LENGTH - 1U] = ptrFeatures[idx + DSS_FEATURE_IDX_SUMABS_STARTIDX + FEATURE_START_IDX];
	}
}

/**
 *  @b Description
 *  @n
 *     The main gesture Inference function.
 *
 *     - The featureVector log is first augmented with two new features. These
 *     are expected to help the ANN in correct inference.
 *     - Next the featureVector is passed to the inference module which returns
 *     a set of probabilites.
 *     - Finally, the  augment the  to copy the current frame's energy
 *     measurements into a log. This log is then passed on to the 'people
 *     detection' engine.  The oldest set of energy measurements are also
 *     deleted from the log, simultaneously.
 *
 *     The energy measurements come as part of the feature vector array from the
 *     DSS.
 *
 *  @retval
 *      Not Applicable.
 */
float gestureInferenceProcessing (float * featureVecLog, int32_t gestureCnt[], int32_t gestureCntPrev[])
{
	float gestureOut;
	int32_t maxCheckFlag;
	/* Augment to the feature vectors with some filtered features. These new features
	 * note the change in angle (in both azimuth and elevation) around the peak doppler. */
	angleDelta(&featureVecLog[AZIM_DELTA_IDX], // AzimDelta, the first new feature.
			   &featureVecLog[ELEV_DELTA_IDX], // ElevDelta, the second new feature.
			   &featureVecLog[MSS_FEATURE_IDX_WTDOPPLER*FEATURE_LENGTH], // WtDoppler[]
			   &featureVecLog[MSS_FEATURE_IDX_WTAZIM*FEATURE_LENGTH],    // WtAzim[]
			   &featureVecLog[MSS_FEATURE_IDX_WTELEV*FEATURE_LENGTH]);   // WtElev[]

	maxCheckFlag = maxCheck(&featureVecLog[MSS_FEATURE_IDX_WTDOPPLER*FEATURE_LENGTH]);


	/* Perform inference. */
	Inference(&featureVecLog[0], &gANN_struct_t);


	/* Some additional heuristics on the inferred probabilities. */
	gestureOut = postProcessing(gANN_struct_t.prob, gestureProbabilityLog, gestureTimeStamp, gestureCnt, gestureCntPrev, maxCheckFlag);

	return gestureOut;
}



/**
 *  @b Description
 *  @n
 *     This function checks whether the max doppler (i.e the main part of the
 *   gesture) occurs at the middle of the feature vector. This prevents cases
 *   where the tailing end of a gesture triggers an inference event (i.e.
 *   gesture detection). Such an event can result in double counting of
 *   gestures.
 *
 *  @retval
 *      true/false.
 */
int32_t maxCheck(float dop[])
{
	int32_t idx, maxIdx = 0;
	float maxVal = -1.0f;
	float absVal;

	for (idx = 0; idx < FEATURE_LENGTH; idx ++)
	{
		absVal = fabsf(dop[idx]);
		if (absVal > maxVal)
		{
			maxVal = absVal;
			maxIdx = idx;
		}
	}

	/* If the feature length is 8, then we accept features that lie between
	 * indices 2 and 4 (i.e the 3rd and the 6th numbers). */
	if ((maxIdx > FEATURE_LENGTH/2 -2) &&
		(maxIdx < FEATURE_LENGTH/2 + 1 ))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
