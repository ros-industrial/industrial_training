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

int arMatrixTrans(ARMat *dest, ARMat *source)
{
	int r, c;

	if(dest->row != source->clm || dest->clm != source->row) return -1;

	for(r = 0; r < dest->row; r++) {
		for(c = 0; c < dest->clm; c++) {
			ARELEM0(dest, r, c) = ARELEM0(source, c, r);
		}
	}

	return 0;
}
