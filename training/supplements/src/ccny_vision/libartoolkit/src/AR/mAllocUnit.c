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

ARMat *arMatrixAllocUnit(int dim)
{
	ARMat *m;

	m = arMatrixAlloc(dim, dim);
	if( m == NULL ) return NULL;

	if( arMatrixUnit(m) < 0 ) {
		arMatrixFree(m);
		return NULL;
	}

	return m;
}
