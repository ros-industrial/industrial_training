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

int arMatrixUnit(ARMat *unit)
{
	int r, c;

	if(unit->row != unit->clm) return -1;

	for(r = 0; r < unit->row; r++) {
		for(c = 0; c < unit->clm; c++) {
			if(r == c) {
				ARELEM0(unit, r, c) = 1.0;
			}
			else {
				ARELEM0(unit, r, c) = 0.0;
			}
		}
	}

	return 0;
}
