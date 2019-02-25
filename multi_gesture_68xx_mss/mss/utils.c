/*
 * Utils.c
 *
 *  Created on: Jan 17, 2018
 *      Author: a0393451
 */
#include <stdint.h>
#include <math.h>
 /*
 @Brief: x1 is the pointer to first input whose length is len
 @Brief: x2 is the pointer to second input whose length is len
 This function computes the inner product between two vectors
 */
float compute_innerprod(float x1[], float x2[], unsigned const int len)
{
	float prod = 0;
	uint32_t i = 0;

	for (i = 0; i < len; i++)
	{
		prod += x1[i] * x2[i];
	}
	return prod;
}

/*
This function computes the relu activation max(0,inp)
*/
void relu_activation(float inp[], unsigned const int len)
{
	uint32_t i = 0;
	for (i = 0; i < len; i++)
	{
		if (inp[i] < 0.0f)
		{
			inp[i] = 0.0f;
		}
	}

}

/*
This is stable implementation of SOFT-MAX
*/
void soft_max(float inp[], unsigned const int len, float prob[])
{
	// find the max
	float maxval = inp[0];
	float sum = 0.0f;
	uint32_t i;
	for (i = 1; i < len; i++)
	{
		if (inp[i] > maxval)
			maxval = inp[i];
	}
	// subtract max from all entries in input
	for (i = 0; i < len; i++)
	{
		inp[i] = (float)exp((double)(inp[i] - maxval)); // exp() expects double type as input and return double type as output
		sum += inp[i];
	}
	//
	for (i = 0; i < len; i++)
	{
		prob[i] = inp[i] / sum;
	}
}

/*
This function normalizes the input feature to have zero mean and unit standard deviation
*/
void normalize_feature_zero_mean_unit_sigma(float inp[], unsigned const int len, float op[])
{
	uint32_t i;
	float sum = 0, sumsq = 0, eps = 1e-16, mean = 0, sigma = 0;

	for (i = 0; i < len; i++)
	{
		sum += inp[i];
		sumsq += inp[i] * inp[i];
	}
	mean = sum / len;
	sigma = (float)sqrt((double)(sumsq / len - mean*mean)) + eps;

	for (i = 0; i < len; i++)
	{
		op[i] = (inp[i] - mean) / sigma;
	}

}

