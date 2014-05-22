/*******************************************************
 *
 * Author: Shinsaku Hiura, Hirokazu Kato
 *
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 2.1
 * Date: 99/07/16
 *
*******************************************************/
#include <stdio.h>
#include <math.h>
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif
#include <AR/matrix.h>

ARMat *arMatrixAllocMul(ARMat *a, ARMat *b)
{
	ARMat *dest;

	dest = arMatrixAlloc(a->row, b->clm);
	if( dest == NULL ) return NULL;

	if( arMatrixMul(dest, a, b) < 0 ) {
		arMatrixFree(dest);
		return NULL;
	}

	return dest;
}
