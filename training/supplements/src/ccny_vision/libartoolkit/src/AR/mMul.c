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
#include <AR/matrix.h>

int arMatrixMul(ARMat *dest, ARMat *a, ARMat *b)
{
	int r, c, i;

	if(a->clm != b->row || dest->row != a->row || dest->clm != b->clm) return -1;

	for(r = 0; r < dest->row; r++) {
		for(c = 0; c < dest->clm; c++) {
			ARELEM0(dest, r, c) = 0.0;
			for(i = 0; i < a->clm; i++) {
				ARELEM0(dest, r, c) += ARELEM0(a, r, i) * ARELEM0(b, i, c);
			}
		}
	}

	return 0;
}
