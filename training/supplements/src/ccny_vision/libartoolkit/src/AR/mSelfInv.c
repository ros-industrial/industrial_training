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

static double *minv( double *ap, int dimen, int rowa );

int arMatrixSelfInv(ARMat *m)
{
	if(minv(m->m, m->row, m->row) == NULL) return -1;

	return 0;
}

	
/********************************/
/*                              */
/*    MATRIX inverse function   */
/*                              */
/********************************/
static double *minv( double *ap, int dimen, int rowa )
{
        double *wap, *wcp, *wbp;/* work pointer                 */
        int i,j,n,ip,nwork;
        int nos[50];
        double epsl;
        double p,pbuf,work;
        double  fabs();

        epsl = 1.0e-10;         /* Threshold value      */

        switch (dimen) {
                case (0): return(NULL);                 /* check size */
                case (1): *ap = 1.0 / (*ap);
                          return(ap);                   /* 1 dimension */
        }

        for(n = 0; n < dimen ; n++)
                nos[n] = n;

        for(n = 0; n < dimen ; n++) {
                wcp = ap + n * rowa;

                for(i = n, wap = wcp, p = 0.0; i < dimen ; i++, wap += rowa)
                        if( p < ( pbuf = fabs(*wap)) ) {
                                p = pbuf;
                                ip = i;
                        }
                if (p <= epsl)
                        return(NULL);

                nwork = nos[ip];
                nos[ip] = nos[n];
                nos[n] = nwork;

                for(j = 0, wap = ap + ip * rowa, wbp = wcp; j < dimen ; j++) {
                        work = *wap;
                        *wap++ = *wbp;
                        *wbp++ = work;
                }

                for(j = 1, wap = wcp, work = *wcp; j < dimen ; j++, wap++)
                        *wap = *(wap + 1) / work;
                *wap = 1.0 / work;

                for(i = 0; i < dimen ; i++) {
                        if(i != n) {
                                wap = ap + i * rowa;
                                for(j = 1, wbp = wcp, work = *wap;
                                                j < dimen ; j++, wap++, wbp++)
                                        *wap = *(wap + 1) - work * (*wbp);
                                *wap = -work * (*wbp);
                        }
                }
        }

        for(n = 0; n < dimen ; n++) {
                for(j = n; j < dimen ; j++)
                        if( nos[j] == n) break;
                nos[j] = nos[n];
                for(i = 0, wap = ap + j, wbp = ap + n; i < dimen ;
                                        i++, wap += rowa, wbp += rowa) {
                        work = *wap;
                        *wap = *wbp;
                        *wbp = work;
                }
        }
        return(ap);
}
