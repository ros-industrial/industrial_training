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

ARMat *arMatrixAllocDup(ARMat *source)
{
	ARMat *dest;

	dest = arMatrixAlloc(source->row, source->clm);
        if( dest == NULL ) return NULL;

	if( arMatrixDup(dest, source) < 0 ) {
		arMatrixFree(dest);
		return NULL;
	}

	return dest;
}
