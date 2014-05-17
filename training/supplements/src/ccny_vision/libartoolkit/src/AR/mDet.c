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

#define MATRIX(name,x,y,width)  ( *(name + (width) * (x) + (y)) )

static double mdet( double *ap, int dimen, int rowa );

double arMatrixDet(ARMat *m)
{

	if(m->row != m->clm) return 0.0;

	return mdet(m->m, m->row, m->row);
}



static double mdet(double *ap, int dimen, int rowa)
/*  double  *ap;          input matrix */
/*  int     dimen;        Dimension of linre and row, those must be equal,
                          that is square matrix.       */
/*  int     rowa;         ROW Dimension of matrix A    */
{
    double det = 1.0;
    double work;
    int    is = 0;
    int    mmax;
    int    i, j, k;

    for(k = 0; k < dimen - 1; k++) {
        mmax = k;
        for(i = k + 1; i < dimen; i++)
            if (fabs(MATRIX(ap, i, k, rowa)) > fabs(MATRIX(ap, mmax, k, rowa)))
                mmax = i;
        if(mmax != k) {
            for (j = k; j < dimen; j++) {
                work = MATRIX(ap, k, j, rowa);
                MATRIX(ap, k, j, rowa) = MATRIX(ap, mmax, j, rowa);
                MATRIX(ap, mmax, j, rowa) = work;
            }
            is++;
        }
        for(i = k + 1; i < dimen; i++) {
            work = MATRIX(ap, i, k, rowa) / MATRIX(ap, k, k, rowa);
            for (j = k + 1; j < dimen; j++)
                MATRIX(ap, i, j, rowa) -= work * MATRIX(ap, k, j, rowa);
        }
    }
    for(i = 0; i < dimen; i++)
        det *= MATRIX(ap, i, i, rowa);
    for(i = 0; i < is; i++) 
        det *= -1.0;
    return(det);
}
