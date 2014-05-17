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

int arMatrixFree(ARMat *m)
{
	free(m->m);
	free(m);

	return 0;
}
