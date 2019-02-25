#ifndef MSS_DATA_PATH_H
#define MSS_DATA_PATH_H

#define FEATURE_LENGTH (8) 	 		// the length of the log of each feature.
#define NUM_FEATURES   (8)   		// total number of features extracted
#define NUM_CLASSIFIER_FEATURES (7) // The number of features that will be fed to inference engine.
#define NUM_NODES_LOG 10 			// The length of the recording of the number of frames.


#define iCURR 0
#define iPREV 1

#define FEATURE_START_IDX 2			// In the message from the DSS, the extracted features start from FEATURE_START_IDX

// Indices of the list of features as send by DSS.
#define DSS_FEATURE_IDX_WTDOPPLER 		0
#define DSS_FEATURE_IDX_WTDOPPLERPOS 	1
#define DSS_FEATURE_IDX_WTDOPPLERNEG 	2
#define DSS_FEATURE_IDX_WTAZIM 			3
#define DSS_FEATURE_IDX_WTELEV 			4
#define DSS_FEATURE_IDX_NUMDET 			5
#define DSS_FEATURE_IDX_WTAZIMSTD 		6
#define DSS_FEATURE_IDX_WTELEVSTD 		7
#define DSS_FEATURE_IDX_WTRANGE 		8
#define DSS_FEATURE_IDX_NUMCYCLES 		9
// These 4 features (non-zero doppler energy measurements for different ranges) are used for people detection.
#define DSS_FEATURE_IDX_SUMABS_IDX1 	10
#define DSS_FEATURE_IDX_SUMABS_IDX2 	11
#define DSS_FEATURE_IDX_SUMABS_IDX3 	12
#define DSS_FEATURE_IDX_SUMABS_IDX4 	13
// The start and end indices for the non-zero doppler energy measurements for different ranges used for people detection. .
#define DSS_FEATURE_IDX_SUMABS_STARTIDX 10
#define DSS_FEATURE_IDX_SUMABS_STOPIDX  13
// The length of the energy log.
#define ENERGY_LOG_LENGTH (32U)
// The number of buckets into which the arc-region in front the radar is divided into.
#define NUM_RANGE_BUCKETS_FOR_PERSON_DETECTION (DSS_FEATURE_IDX_SUMABS_STOPIDX - DSS_FEATURE_IDX_SUMABS_STARTIDX + 1)
// The number of detections (within one bucket) for the continued maintenance of 'people detection' state.
#define NUM_DETECTIONS_FOR_MAINTENANCE (ENERGY_LOG_LENGTH/8)
// The number of detections (within one bucket) for initialisation of 'people detection' state.
#define NUM_DETECTIONS_FOR_INIT (ENERGY_LOG_LENGTH/4)

// Indices of the list of features as used by MSS and the inference engine.
#define MSS_FEATURE_IDX_WTDOPPLER 		0
#define MSS_FEATURE_IDX_WTDOPPLERPOS 	1
#define MSS_FEATURE_IDX_WTDOPPLERNEG 	2
#define MSS_FEATURE_IDX_WTAZIM 			3
#define MSS_FEATURE_IDX_WTELEV 			4
#define MSS_FEATURE_IDX_NUMDET 			5
#define MSS_FEATURE_IDX_WTRANGE 		6

// The probability threshold beyond which the inference output is declared 'significant'.
#define GESTURE_INFERENCE_THRESHOLD (0.8999f)
// What should be the distance between successive gestures.
#define INTER_GESTURE_MIN_SEPARATION_FRAMES 15

// The total number of augmented features added to the base-feature vector.
#define NUM_AUGMENTS 2
// Indices of the augmented feature vectors (azimDelta and elevDelta) in the MSS's featureVecLog.
#define AZIM_DELTA_IDX (NUM_CLASSIFIER_FEATURES*FEATURE_LENGTH)
#define ELEV_DELTA_IDX (AZIM_DELTA_IDX + 1)

/* Enum of the list of gestures. */
#define UP_TO_DOWN_SWIPE_GESTURE 0
#define RIGHT_TO_LEFT_SWIPE_GESTURE 1
#define DOWN_TO_UP_SWIPE_GESTURE 2
#define LEFT_TO_RIGHT_SWIPE_GESTURE 3
#define NO_GESTURE 2

/* A set of options on what needs to be send out over the UART.
 * Set SEND_ONLY_INFERENCE_RESULTS_OUT_ON_UART to send only the
 * the inference results on UART (extremely low throughput).
 * Set SEND_FEATURE_VECTORS_OUT_ON_UART and
 * SEND_INFERENCE_INTERMEDIATE_RESULTS_OUT_ON_UART to send the
 * the featureVectors and the intermediate results out on UART
 * to aid either understanding or debug. */
#define	SEND_ONLY_INFERENCE_RESULTS_OUT_ON_UART 1
#define SEND_FEATURE_VECTORS_OUT_ON_UART 0
#define SEND_INFERENCE_INTERMEDIATE_RESULTS_OUT_ON_UART 0

/* The dimensions of the different layers of the ANN. */
#define NUM_NODES_FIRST_LAYER (30)
#define NUM_NODES_SECOND_LAYER (20)
#define NUM_NODES_THIRD_LAYER (3)
#define INP_DIM ((NUM_CLASSIFIER_FEATURES* FEATURE_LENGTH) + NUM_AUGMENTS) //input layer.

/**
 * @brief
 *  Millimeter Wave Demo Data Path ANN based detection wieghts, biases, internal/final ops.
 *
 * @details
 *  The structure is used to hold all the relevant information for ANN based detection in
 *  the data path.
 */
typedef struct ANN_struct_t_
{
	float b_1[NUM_NODES_FIRST_LAYER];
	float b_2[NUM_NODES_SECOND_LAYER];
	float b_3[NUM_NODES_THIRD_LAYER];
	float W_1[NUM_NODES_FIRST_LAYER][INP_DIM];
	float W_2[NUM_NODES_SECOND_LAYER][NUM_NODES_FIRST_LAYER];
	float W_3[NUM_NODES_THIRD_LAYER][NUM_NODES_SECOND_LAYER];
	float op_layer1[NUM_NODES_FIRST_LAYER];
	float op_layer2[NUM_NODES_SECOND_LAYER];
	float op_layer3[NUM_NODES_THIRD_LAYER];
	float prob[NUM_NODES_THIRD_LAYER];

} ANN_struct_t;


/**
 *  @b Description
 *  @n
 *      Preprocess the inputs to ANN detector, centers the input vector b/w [-1,1]
 *  @retval
 *     None
 */
void PreprocessANNInputs(float featureVec[], float FeaturevectorMSS[]);

/**
 *  @b Description
 *  @n
 *      Compute the inference on the current input
 *  @retval
 *     None
 */
void Inference(float *input, ANN_struct_t* pANN_struct_t);

/**
 *  @b Description
 *  @n
 *      Helper function which computes inner product between two float vectors
 *  @retval
 *     float type for the inner product
 */
float compute_innerprod(float x1[], float x2[], unsigned const int len);

/**
 *  @b Description
 *  @n
 *      Perform Relu activation (max(0,x))
 *  @retval
 *     None
 */
void relu_activation(float inp[], unsigned const int len);

/**
 *  @b Description
 *  @n
 *      Perform numerically stable implementation of SOFTMAX
 *  @retval
 *     None
 */
void soft_max(float inp[], unsigned const int len, float prob[]);
void normalize_feature_zero_mean_unit_sigma(float inp[], unsigned const int len, float op[]);
int32_t postProcessing(float prob[], float prob_log[][NUM_NODES_THIRD_LAYER], int32_t gestureTimeStamp[], int32_t gestureCnt[], int32_t gestureCntPrev[], int32_t maxCheckFlag);
float gestureInferenceProcessing (float * featureVecLog, int32_t gestureCnt[], int32_t gestureCntPrev[]);
void logEnergy(float * ptrFeatures, float * energyLog);
void logFeatures(float * ptrFeatures, float *featureVecLog, int32_t negateAngleMeasurements);
int32_t runPeopleDetection(int32_t peopleDetectorState, float energyLog[]);
int32_t maxCheck(float *dop);



#endif /* MSS_DATA_PATH_H */
