/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 1.0
 * Date: 01/09/05
 *
*******************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <AR/ar.h>
#include <AR/matrix.h>
#include <AR/arMulti.h>

static char *get_buff( char *buf, int n, FILE *fp );

ARMultiMarkerInfoT *arMultiReadConfigFile( const char *filename )
{
    FILE                   *fp;
    ARMultiEachMarkerInfoT *marker;
    ARMultiMarkerInfoT     *marker_info;
    double                 wpos3d[4][2];
    char                   buf[256], buf1[256];
    int                    num;
    int                    i, j;

    if( (fp=fopen(filename,"r")) == NULL ) return NULL;

    get_buff(buf, 256, fp);
    if( sscanf(buf, "%d", &num) != 1 ) {fclose(fp); return NULL;}

    arMalloc(marker,ARMultiEachMarkerInfoT,num);

    for( i = 0; i < num; i++ ) {
        get_buff(buf, 256, fp);
        if( sscanf(buf, "%s", buf1) != 1 ) {
            fclose(fp); free(marker); return NULL;
        }
        if( (marker[i].patt_id = arLoadPatt(buf1)) < 0 ) {
            fclose(fp); free(marker); return NULL;
        }

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%lf", &marker[i].width) != 1 ) {
            fclose(fp); free(marker); return NULL;
        }

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%lf %lf", &marker[i].center[0], &marker[i].center[1]) != 2 ) {
            fclose(fp); free(marker); return NULL;
        }

        for( j = 0; j < 3; j++ ) {
            get_buff(buf, 256, fp);
            if( sscanf(buf, "%lf %lf %lf %lf", &marker[i].trans[j][0],
                       &marker[i].trans[j][1], &marker[i].trans[j][2],
                       &marker[i].trans[j][3]) != 4 ) {
                fclose(fp); free(marker); return NULL;
            }
        }
        arUtilMatInv( marker[i].trans, marker[i].itrans );

        wpos3d[0][0] = marker[i].center[0] - marker[i].width/2.0;
        wpos3d[0][1] = marker[i].center[1] + marker[i].width/2.0;
        wpos3d[1][0] = marker[i].center[0] + marker[i].width/2.0;
        wpos3d[1][1] = marker[i].center[1] + marker[i].width/2.0;
        wpos3d[2][0] = marker[i].center[0] + marker[i].width/2.0;
        wpos3d[2][1] = marker[i].center[1] - marker[i].width/2.0;
        wpos3d[3][0] = marker[i].center[0] - marker[i].width/2.0;
        wpos3d[3][1] = marker[i].center[1] - marker[i].width/2.0;
        for( j = 0; j < 4; j++ ) {
            marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0]
                                  + marker[i].trans[0][1] * wpos3d[j][1]
                                  + marker[i].trans[0][3];
            marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0]
                                  + marker[i].trans[1][1] * wpos3d[j][1]
                                  + marker[i].trans[1][3];
            marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0]
                                  + marker[i].trans[2][1] * wpos3d[j][1]
                                  + marker[i].trans[2][3];
        }
    }

    fclose(fp);

    marker_info = (ARMultiMarkerInfoT *)malloc( sizeof(ARMultiMarkerInfoT) );
    if( marker_info == NULL ) {free(marker); return NULL;}
    marker_info->marker     = marker;
    marker_info->marker_num = num;
    marker_info->prevF      = 0;

    return marker_info;
}

static char *get_buff( char *buf, int n, FILE *fp )
{
    char *ret;

    for(;;) {
        ret = fgets( buf, n, fp );
        if( ret == NULL ) return(NULL);
        if( buf[0] != '\n' && buf[0] != '#' ) return(ret);
    }
}
