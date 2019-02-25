#ifndef GESTURE_H
#define GESTURE_H


#include <ti/sysbios/knl/Semaphore.h>

#include <ti/common/sys_common.h>

#include <c6x.h>
/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <dss_data_path.h>
#include <DSP_fft16x16.h>
#include <ti/alg/mmwavelib/mmwavelib.h>

typedef struct Features_t {
    /* Note: These will point to L3 heap */
    float pWtrange;
    float pWtdoppler;
    float pWtdopplerPos;
    float pWtdopplerNeg;
    float pInstenergy;
    float pAzdoppcorr;
    float pwtaz_mean;
    float pwtel_mean;
    float pwtaz_std;
    float pwtel_std;
    float pWtrange_disp;
    float pWtdoppler_disp;
    float pNumDetections;
    float numCycles;
    uint16_t rangebin_start;
    uint16_t rangebin_stop; //SR_CAUTION : num valid range bins is rangebin_stop+1
    uint16_t posdopplerbin_start; //Q:  Why are these in both structs?
    uint16_t posdopplerbin_stop;
    int16_t  negdopplerbin_start;
    int16_t  negdopplerbin_stop;
    float    detthresh;

} Features_t;

extern Features_t gfeatures;
#define NUM_ANGLE_BINS_GESTURE 32
#define NUM_SORTED_VALUES 50
#define THRESH_NUM_POINTS 8000

extern void ComputeAngleStats(MmwDemo_DSS_DataPathObj *obj,Features_t * gfeatures,uint16_t indx);


extern void Computefeatures_RDIBased(Features_t *pfeatures, uint32_t *preDetMatrix, uint32_t indx, uint16_t num_doppler_bins);

extern void gestureInit(Features_t *pfeatures);

extern void AutoScale(int16_t x[restrict]);
extern unsigned  int find_max16(const unsigned short  x[restrict],  unsigned int len);
extern unsigned  int find_max(const unsigned int  x[restrict],  unsigned int len);



#endif
