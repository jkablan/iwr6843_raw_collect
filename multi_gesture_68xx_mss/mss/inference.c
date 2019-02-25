/*
 * Inference.c
 *
 *  Created on: Jan 17, 2018
 *      Author: a0393451
 */
#include <stdint.h>
#include <mss_data_path.h>
void Inference(float *input, ANN_struct_t* pANN_struct_t)
{

    float prod;
    uint32_t i=0;

    // compute Layer 1 output
    for(i=0; i<NUM_NODES_FIRST_LAYER; i++)
    {
        pANN_struct_t->op_layer1[i] = pANN_struct_t->b_1[i]; // initialize with bias value
        prod = compute_innerprod(input, &pANN_struct_t->W_1[i][0], INP_DIM);
        pANN_struct_t->op_layer1[i] += prod;
    }
    // activation layer
    relu_activation(pANN_struct_t->op_layer1, NUM_NODES_FIRST_LAYER);

    // compute Layer2 output
    for(i=0; i<NUM_NODES_SECOND_LAYER; i++)
    {
        pANN_struct_t->op_layer2[i] = pANN_struct_t->b_2[i]; // initialize with bias value
        prod = compute_innerprod(pANN_struct_t->op_layer1, &pANN_struct_t->W_2[i][0], NUM_NODES_FIRST_LAYER);
        pANN_struct_t->op_layer2[i] += prod;
    }
    // activation layer
    relu_activation(pANN_struct_t->op_layer2, NUM_NODES_SECOND_LAYER);

    // compute Layer3 output
    for(i=0; i<NUM_NODES_THIRD_LAYER; i++)
    {
        pANN_struct_t->op_layer3[i] = pANN_struct_t->b_3[i]; // initialize with bias value
        prod = compute_innerprod(pANN_struct_t->op_layer2, &pANN_struct_t->W_3[i][0], NUM_NODES_SECOND_LAYER);
        pANN_struct_t->op_layer3[i] += prod;
    }

    // soft-max layer
    soft_max(pANN_struct_t->op_layer3, NUM_NODES_THIRD_LAYER, pANN_struct_t->prob);


}

