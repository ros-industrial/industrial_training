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

int arVecDisp( ARVec *v )
{
    int    c;
    
    if( v == NULL ) return -1;

    printf(" === vector (%d) ===\n", v->clm);
    printf(" |");
    for( c = 0; c < v->clm; c++ ){
	printf( " %10g", v->v[c] );
    }
    printf(" |\n");
    printf(" ===================\n");

    return 0;
}
