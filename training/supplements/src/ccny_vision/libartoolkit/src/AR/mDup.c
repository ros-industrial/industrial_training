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

int arMatrixDup(ARMat *dest, ARMat *source)
{
	int r,c;

	if(dest->row != source->row || dest->clm != source->clm) {
		return -1;
	}
	for(r = 0; r < source->row; r++) {
		for(c = 0; c < source->clm; c++) {
			ARELEM0(dest, r, c) = ARELEM0(source, r, c);
		}
	}
	return 0;
}
