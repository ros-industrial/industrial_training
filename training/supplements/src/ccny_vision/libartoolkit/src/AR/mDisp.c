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

int arMatrixDisp(ARMat *m)
{
	int r, c;

	printf(" === matrix (%d,%d) ===\n", m->row, m->clm);
	for(r = 0; r < m->row; r++) {
		printf(" |");
		for(c = 0; c < m->clm; c++) {
			printf(" %10g", ARELEM0(m, r, c));
		}
		printf(" |\n");
	}
	printf(" ======================\n");

	return 0;
}
