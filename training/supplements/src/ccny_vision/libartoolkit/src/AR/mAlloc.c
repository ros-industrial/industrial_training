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

ARMat *arMatrixAlloc(int row, int clm)
{
	ARMat *m;

	m = (ARMat *)malloc(sizeof(ARMat));
	if( m == NULL ) return NULL;

	m->m = (double *)malloc(sizeof(double) * row * clm);
	if(m->m == NULL) {
		free(m);
		return NULL;
	}
	else {
		m->row = row;
		m->clm = clm;
	}

	return m;
}
