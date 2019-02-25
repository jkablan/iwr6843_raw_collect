#include <gesture.h>
void addDopplerCompensation16(int32_t dopplerIdx,
								int32_t numDopplerBins,
								cmplx16ImRe_t *azimuthModCoefs,
								cmplx16ImRe_t *azimuthModCoefsHalfBin,
								cmplx16ReIm_t *azimuthIn,
								uint32_t numAnt);

void gestureInit(Features_t *pfeatures)
{

	pfeatures->rangebin_start = 1;
	pfeatures->rangebin_stop = 7;
	pfeatures->posdopplerbin_start = 5;
	pfeatures->posdopplerbin_stop = 60;
	pfeatures->negdopplerbin_start = -5;
	pfeatures->negdopplerbin_stop = -60;
}

volatile float dopplerdbg;
void Computefeatures_RDIBased(Features_t *pfeatures, uint32_t *preDetMatrix, uint32_t indx, uint16_t num_doppler_bins)
{
	uint32_t wt, numDetections = 0;
	uint32_t rangeIdx;
	int32_t dopplerIdx = 0;
	//int32_t  rindx, dindx=0;
	//uint32_t maxval=0U;
	float rangeAvg = 0, dopplerAvg = 0, wtSum = 0, wtSumPos = 0, wtSumNeg = 0, rangesqAvg = 0, dopplersqAvg = 0, dopplerAvgPos = 0, dopplerAvgNeg = 0;
	uint32_t *tempPtr = NULL;



	// Compute wtrange, wtdoppler, instantaneous energy, max cell indx
	for (rangeIdx = pfeatures->rangebin_start; rangeIdx <= pfeatures->rangebin_stop; rangeIdx++)
	{
		// set the temp pointer, this logic may be different on SDK code

		for (dopplerIdx = pfeatures->posdopplerbin_start; dopplerIdx <= pfeatures->posdopplerbin_stop; dopplerIdx++)
		{

			tempPtr = preDetMatrix + rangeIdx*num_doppler_bins + dopplerIdx;
			wt = *tempPtr;

			dopplerAvg += wt * ((float)(dopplerIdx)); // float promotion
			dopplerAvgPos += wt * ((float)(dopplerIdx)); // float promotion
			rangeAvg += wt * ((float)(rangeIdx)); // float promotion
			rangesqAvg += wt * ((float)(rangeIdx * rangeIdx)); // float promotion
			dopplersqAvg += wt * ((float)(dopplerIdx * dopplerIdx)); //float promotion
			wtSumPos += (float)wt;

			if (wt > THRESH_NUM_POINTS)
				numDetections++;


#if 0
			if (wt > maxval)
			{
				rindx = rangeIdx;
				dindx = dopplerIdx;
				maxval = wt;
			}
#endif

		}


	}

	for (rangeIdx = pfeatures->rangebin_start; rangeIdx <= pfeatures->rangebin_stop; rangeIdx++)
	{
		// set the temp pointer, this logic may be different on SDK code

		for (dopplerIdx = pfeatures->negdopplerbin_stop; dopplerIdx <= pfeatures->negdopplerbin_start; dopplerIdx++)
		{
			tempPtr = preDetMatrix + rangeIdx*num_doppler_bins + (dopplerIdx + num_doppler_bins); // map the negative doppler index b/w [0,Fs]
			wt = *tempPtr;
			dopplerAvg += wt * ((float)(dopplerIdx)); // float promotion
			dopplerAvgNeg += wt * ((float)(dopplerIdx)); // float promotion
			rangeAvg += wt * ((float)(rangeIdx)); // float promotion
			rangesqAvg += wt * ((float)(rangeIdx * rangeIdx)); // float promotion
			dopplersqAvg += wt * ((float)(dopplerIdx * dopplerIdx)); //float promotion
			wtSumNeg += (float)wt;

			if (wt > THRESH_NUM_POINTS)
				numDetections++;


#if 0
			if (wt > maxval)
			{
				rindx = rangeIdx;
				dindx = (uint32_t)(dopplerIdx + num_doppler_bins);
				maxval = wt;
			}
#endif
		}

	}
	wtSum = wtSumPos + wtSumNeg;
	dopplerdbg = dopplerAvg / wtSum;
	pfeatures->pWtdoppler = dopplerAvg / wtSum;
	pfeatures->pWtrange = rangeAvg / wtSum;
	pfeatures->pInstenergy = wtSum;
	pfeatures->pNumDetections = (float)numDetections;
	pfeatures->pWtdopplerNeg = dopplerAvgPos / wtSumPos; //swapped intentionally to conform with matlab scripts
	pfeatures->pWtdopplerPos = dopplerAvgNeg / wtSumNeg; //swapped intentionally
 
}



/********************DOA based features *******************************/

cmplx16ReIm_t gDOAIn[NUM_ANGLE_BINS_GESTURE];

cmplx16ReIm_t gDOAOut[NUM_ANGLE_BINS_GESTURE];
cmplx16ReIm_t gDOA2D[NUM_ANGLE_BINS_GESTURE][NUM_ANGLE_BINS_GESTURE];


#define NUM_USED_CHANNELS 8
void Computefeatures_DOABased(cmplx16ReIm_t * restrict DOAIn, cmplx16ReIm_t * restrict azimuthTwiddle16x16, uint16_t *restrict log2Abs, int16_t *azimIdx, int16_t *elevIdx)
{
	uint16_t row_idx, col_idx;
	int16_t azimIdxtemp, elevIdxtemp;
	uint32_t ind;
        uint32_t antIndx;

#if defined(MMW_6843_ODS)
        /* The 6843 ODS Antenna has RX1 and RX4 fed from the opposite side so all
         * corresponding virtual RXs channels need to be inverted by 180 degrees
         */
        int16_t rxPhaseRotVector[NUM_USED_CHANNELS] = {-1,1,1,-1, -1,1,1,-1};
#elif defined(MMW_6843_AOP)
        /* The 6843 AOP Antenna has RX1 and RX3 fed from the opposite side so all
         * corresponding virtual RXs channels need to be inverted by 180 degrees
         */
        int16_t rxPhaseRotVector[NUM_USED_CHANNELS] = {-1,1,-1,1, -1,1,-1,1};
#endif

	/* auto-scale the input so as to prevent overflow in the 32-bit FFT. */
	AutoScale((int16_t*)DOAIn);
        
        /* Rotate the RX channels according to the ODS RX phase rotation vector */
        for (antIndx = 0; antIndx < NUM_USED_CHANNELS; antIndx++)
        {
            DOAIn[antIndx].real *= rxPhaseRotVector[antIndx];
            DOAIn[antIndx].imag *= rxPhaseRotVector[antIndx];
        } 


#if defined(MMW_6843_ODS)      
        // Arrange the DOAIn in 2D array format (below is based on 6843 ODS antenna)
        /* Channels are arranged as (because the chirps are sent in this order: TX1, TX2)
         * ch-0   ch-3   ch-4   ch-7
         * ch-1   ch-2   ch-5   ch-6 
         *               ch-8   ch-11 ---> (TX-3 row: Not used)
         *               ch-9   ch-10 ---> (TX-3 row: Not used)
         */
        memset((void *)&gDOA2D[0][0], 0, NUM_ANGLE_BINS_GESTURE * NUM_ANGLE_BINS_GESTURE *sizeof(cmplx16ReIm_t));

	gDOA2D[0][0] = DOAIn[0];
	gDOA2D[0][1] = DOAIn[3];
	gDOA2D[0][2] = DOAIn[4];
	gDOA2D[0][3] = DOAIn[7];


	gDOA2D[1][0] = DOAIn[1];
	gDOA2D[1][1] = DOAIn[2];
	gDOA2D[1][2] = DOAIn[5];
	gDOA2D[1][3] = DOAIn[6];

#elif defined(MMW_6843_AOP)
        // Arrange the DOAIn in 2D array format (below is based on 6843 AOP antenna)
        /* Channels are arranged as (because the chirps are sent in this order: TX1, TX2)
         *               
         *               
         *               ch-8    ch-9  ---> (TX-3 row: Not used)
         *               ch-10   ch-11 ---> (TX-3 row: Not used)
         * ch-0   ch-1   ch-4    ch-5
         * ch-2   ch-3   ch-6    ch-7
         */
	memset((void *)&gDOA2D[0][0], 0, NUM_ANGLE_BINS_GESTURE * NUM_ANGLE_BINS_GESTURE *sizeof(cmplx16ReIm_t));

	gDOA2D[0][0] = DOAIn[0];
	gDOA2D[0][1] = DOAIn[1];
	gDOA2D[0][2] = DOAIn[4];
	gDOA2D[0][3] = DOAIn[5];


	gDOA2D[1][0] = DOAIn[2];
	gDOA2D[1][1] = DOAIn[3];
	gDOA2D[1][2] = DOAIn[6];
	gDOA2D[1][3] = DOAIn[7];
#endif


	/* 1D FFT on the azimuth array of virtual antennas*/
	for (row_idx = 0; row_idx < 2U; row_idx++)
	{
		for (col_idx = 0; col_idx < NUM_ANGLE_BINS_GESTURE; col_idx++)
		{
			gDOAIn[col_idx] = gDOA2D[row_idx][col_idx];

		}

		DSP_fft16x16(
			(int16_t *)azimuthTwiddle16x16,
			NUM_ANGLE_BINS_GESTURE,
			(int16_t *)gDOAIn,
			(int16_t *)&gDOA2D[row_idx][0]);

	} // 1D FFT for angle ends

	/* 2D FFT on the elevation array of virtual antennas*/
	for (col_idx = 0; col_idx < NUM_ANGLE_BINS_GESTURE; col_idx++)
	{
		for (row_idx = 0; row_idx < NUM_ANGLE_BINS_GESTURE; row_idx++)
		{
			gDOAIn[row_idx] = gDOA2D[row_idx][col_idx];
		}

		DSP_fft16x16(
			(int16_t *)azimuthTwiddle16x16,
			NUM_ANGLE_BINS_GESTURE,
			(int16_t *)gDOAIn,
			(int16_t *)gDOAOut);

		for (row_idx = 0; row_idx < NUM_ANGLE_BINS_GESTURE; row_idx++)
		{
			gDOA2D[row_idx][col_idx] = gDOAOut[row_idx];
		}

	} // 2D FFT for angle ends

	/* Compute logabs of  2Dangle-FFT: InputDOA*/
	mmwavelib_log2Abs16(
		(int16_t *)gDOA2D,
		log2Abs,
		NUM_ANGLE_BINS_GESTURE*NUM_ANGLE_BINS_GESTURE);

	ind = find_max16(log2Abs, NUM_ANGLE_BINS_GESTURE*NUM_ANGLE_BINS_GESTURE); // find the index of the maximum in a vector
	//map ind to row and columns
	azimIdxtemp = ind / NUM_ANGLE_BINS_GESTURE;
	elevIdxtemp = ind - (azimIdxtemp)*NUM_ANGLE_BINS_GESTURE;

	if (azimIdxtemp > NUM_ANGLE_BINS_GESTURE / 2 - 1)
		azimIdxtemp -= NUM_ANGLE_BINS_GESTURE;

	if (elevIdxtemp > NUM_ANGLE_BINS_GESTURE / 2 - 1)
		elevIdxtemp -= NUM_ANGLE_BINS_GESTURE;

	*azimIdx = azimIdxtemp;
	*elevIdx = elevIdxtemp;

}


//REVISIT : type for rstart etc.
//obj->log2abs
//uint32_t timeLog1[NUM_SORTED_VALUES],timeLog2[NUM_SORTED_VALUES];
uint16_t maxIdxLog[2 * NUM_SORTED_VALUES], azimelevLog[2 * NUM_SORTED_VALUES];
void ComputeAngleStats(MmwDemo_DSS_DataPathObj *obj, Features_t * gfeatures, uint16_t indx)
{
	uint32_t maxIdx, k, len, j;
	//	uint32_t start_time,end_time;
	cmplx16ReIm_t DOAIn[8], *inptr;
	float wtMeanAzim = 0, wtMeanElev = 0, wtSum = 0, wtAzimSq = 0, wtElevSq = 0, currwt;
	uint32_t *detMatrix, *tempptr;
	volatile int32_t dopplerIdx, rangeIdx;
	int16_t azimIdx, elevIdx;

	detMatrix = obj->predetMatrixGesture;

	//zero out the doppler bins around zero

	for (k = 0; k < (gfeatures->rangebin_stop) + 1; k++)
	{
		tempptr = detMatrix + k*(obj->numDopplerBins);
		for (j = 0; j < (gfeatures->posdopplerbin_start); j++)
		{
			tempptr[j] = 0;
		}

		for (j = obj->numDopplerBins - 1; j >= (obj->numDopplerBins) + (gfeatures->negdopplerbin_start) - 1; j--)
		{
			tempptr[j] = 0;
		}

	}


	len = ((gfeatures->rangebin_stop) + 1)*obj->numDopplerBins;
	for (k = 0; k < NUM_SORTED_VALUES; k++)
	{
		//		start_time=TSCL;
		maxIdx = find_max(detMatrix, len);
		rangeIdx = maxIdx / (obj->numDopplerBins);
		dopplerIdx = maxIdx - rangeIdx*(obj->numDopplerBins);
		//		end_time=TSCL;
				//timeLog1[k]=end_time-start_time;

		maxIdxLog[2 * k] = rangeIdx;
		maxIdxLog[2 * k + 1] = dopplerIdx;

		currwt = (float)(*(detMatrix + rangeIdx*obj->numDopplerBins + dopplerIdx)); // fetch the weight from Rx1 heatmap

		detMatrix[maxIdx] = 0; //SR_CAUTION: this corrupts the detMatrix. Cannot be used further

		//SR_CAUTION: fill DOAIn. May change based on arrangement of data in radar cube
		inptr = (cmplx16ReIm_t*)(obj->radarCube + (obj->numDopplerBins) * 8 * rangeIdx + dopplerIdx);//alligned to arrangement of data in sdk2.0 (after 16-bit 2d-fft related optimization)

		for (j = 0; j < 8; j++)
		{
			DOAIn[j] = *inptr;
			inptr = inptr + (obj->numDopplerBins);
		}
		
		/* Doppler compensation */
		addDopplerCompensation16(dopplerIdx,
                                    (int32_t) obj->numDopplerBins,
                                    obj->azimuthModCoefs,
                                    &obj->azimuthModCoefsHalfBin,
                                    &DOAIn[obj->numRxAntennas],
                                    4);

		//	start_time=TSCL;
		Computefeatures_DOABased(DOAIn, obj->twiddle16x16_DOA, obj->log2Abs, &azimIdx, &elevIdx);
		//	end_time=TSCL;
			//timeLog2[k]=end_time-start_time;

		azimelevLog[2 * k] = azimIdx;
		azimelevLog[2 * k + 1] = elevIdx;

		wtMeanAzim += azimIdx*currwt;
		wtMeanElev += elevIdx*currwt;
		wtAzimSq += azimIdx*azimIdx*currwt;
		wtElevSq += elevIdx*elevIdx*currwt;
		wtSum += currwt;

	}

	wtMeanAzim /= wtSum;
	wtMeanElev /= wtSum;
	wtAzimSq /= wtSum;
	wtElevSq /= wtSum;

	gfeatures->pwtaz_mean = wtMeanElev;//swapped intentionally to conform with matlab scripts
	gfeatures->pwtel_mean = wtMeanAzim;//swapped intentionally
	gfeatures->pwtaz_std = wtElevSq - (wtMeanElev*wtMeanElev);//swapped intentionally
	gfeatures->pwtel_std = wtAzimSq - (wtMeanAzim*wtMeanAzim);//swapped intentionally

}
/***********END*********DOA based features *******************************/


/*************START : UTILS**************************/
unsigned  int find_max16(const unsigned short  x[restrict], unsigned int len)
{
	int k, j;
	unsigned int skip_idx;
	unsigned short max_val[2];
	unsigned int max_idx_temp[2], max_idx;
	max_val[0] = 0;
	max_val[1] = 0;

	skip_idx = len / 2;
	for (k = 0; k < skip_idx; k++)
	{

#pragma UNROLL(2)
		for (j = 0; j<2; j++)
		{
			if (x[k + j * 512]>max_val[j])
			{
				max_val[j] = x[k + j * 512];
				max_idx_temp[j] = k + j * 512;
			}

		}
	}

	if (max_val[0] > max_val[1])
		max_idx = max_idx_temp[0];
	else
		max_idx = max_idx_temp[1];

	return(max_idx);

}


void AutoScale(int16_t x[restrict])
{

	int32_t idx, maxval2 = 0, maxval, *ptr32, shiftval = 0;;
	int16_t *ptr16_out, *ptr16_in;
	ptr32 = (int32_t*)x;
	for (idx = 0; idx < 8; idx++)
	{
		maxval2 = _max2(_abs2(*ptr32++), maxval2);
	}

	maxval = (_max2(_packlh2(maxval2, maxval2), maxval2)) & 0xFFFF;

	shiftval = _lmbd(1, maxval) - 18;

	ptr16_out = ptr16_in = x;
	if (shiftval > 0)
	{
		for (idx = 0; idx < 16; idx++)
		{
			*ptr16_out++ = ((*ptr16_in++) << shiftval);
		}
	}
	else if (shiftval < 0)
	{


	}

}


unsigned  int find_max(const unsigned int  x[restrict], unsigned int len)
{
	int k, j;
	unsigned int skip_idx;
	unsigned int max_val[2], max_idx_temp[2], max_idx;
	max_val[0] = 1;
	max_val[1] = 0;

	skip_idx = len / 2;
	for (k = 0; k < skip_idx; k++)
	{

#pragma UNROLL(2)
		for (j = 0; j<2; j++)
		{
			if (x[k + j * 512]>max_val[j])
			{
				max_val[j] = x[k + j * 512];
				max_idx_temp[j] = k + j * 512;
			}

		}
	}

	if (max_val[0] > max_val[1])
		max_idx = max_idx_temp[0];
	else
		max_idx = max_idx_temp[1];

	return(max_idx);

}

/* possible reasons for c->matlab deviations

1) range-fft window
2) num detection threshold
3) acct for 2d fft scaling

*/


/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    addDopplerCompensation16
 *
 * \par
 * <b>Description</b>  : Compensation of Doppler phase shift in the virtual antennas,
 *                       (corresponding to second Tx antenna chirps). Symbols
 *                       corresponding to virtual antennas, are rotated by half
 *                       of the Doppler phase shift measured by Doppler FFT.
 *                       The phase shift read from the table using half of the
 *                       object Doppler index  value. If the Doppler index is
 *                       odd, an extra half of the bin phase shift is added.
 *
 * @param[in]               dopplerIdx     : Doppler index of the object
 *
 * @param[in]               numDopplerBins : Number of Doppler bins
 *
 * @param[in]               azimuthModCoefs: Table with cos/sin values SIN in even position, COS in odd position
 *                                           exp(1j*2*pi*k/N) for k=0,...,N-1 where N is number of Doppler bins.
 *
 * @param[out]              azimuthModCoefsHalfBin :  exp(1j*2*pi* 0.5 /N)
 *
 * @param[in,out]           azimuthIn        :Pointer to antenna symbols to be Doppler compensated
 *
 * @param[in]              numAnt       : Number of antenna symbols to be Doppler compensated
 *
 * @return                  void
 *
 *******************************************************************************************************************
 */
void addDopplerCompensation16(int32_t dopplerIdx,
	int32_t numDopplerBins,
	cmplx16ImRe_t *azimuthModCoefs,
	cmplx16ImRe_t *azimuthModCoefsHalfBin,
	cmplx16ReIm_t *azimuthIn,
	uint32_t numAnt)
{
	cmplx16ImRe_t expDoppComp;
	uint32_t * ptExpDoppComp = (uint32_t *)&expDoppComp;
	uint32_t * ptAzimuthModCoefsHalfBin = (uint32_t *)azimuthModCoefsHalfBin;
	uint32_t * azimuthInTmp = (uint32_t *)azimuthIn;
	int32_t dopplerCompensationIdx = dopplerIdx;
	uint32_t antIndx;

	/*Divide Doppler index by 2*/
	if (dopplerCompensationIdx >= numDopplerBins / 2)
	{
		dopplerCompensationIdx -= (int32_t)numDopplerBins;
	}
	dopplerCompensationIdx = dopplerCompensationIdx / 2;
	if (dopplerCompensationIdx < 0)
	{
		dopplerCompensationIdx += (int32_t)numDopplerBins;
	}
	expDoppComp = azimuthModCoefs[dopplerCompensationIdx];
	/* Add half bin rotation if Doppler index was odd */
	if (dopplerIdx & 0x1)
	{
		*ptExpDoppComp = _cmpyr1((*ptExpDoppComp), (*ptAzimuthModCoefsHalfBin));
	}

	/* Rotate symbols */
	#pragma MUST_ITERATE(4,,4)
	for (antIndx = 0; antIndx < numAnt; antIndx++)
	{
		int32_t tmp;
		/* In order to make use of the cmpyr1 function, the inputs must be in 
		 * the im,re order, whereas the azimuthIn input is in the re,im order. 
		 * Hence the swap. */
		tmp = _swap2(azimuthInTmp[antIndx]);
		tmp = _cmpyr1(*ptExpDoppComp, tmp);
		azimuthInTmp[antIndx] = _swap2(tmp);
	}

}

