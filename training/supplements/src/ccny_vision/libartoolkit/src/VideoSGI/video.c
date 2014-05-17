/*******************************************************
 *
 * Author: Hirokazu Kato, Atsishi Nakazawa
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *         nakazawa@inolab.sys.es.osaka-u.ac.jp
 *
 * Revision: 5.1
 * Date: 00/05/06
 *
*******************************************************/

#include <string.h>
#include <AR/config.h>
#include <AR/video.h>


static int             init      = 0;             /* initialize flag */
static AR2VideoParamT  *gVid = NULL;


int arVideoDispOption( void )
{
    return  ar2VideoDispOption();
}

int arVideoOpen( char *config )
{
    if( gVid != NULL ) {
        printf("Device has been opened!!\n");
        return -1;
    }
    gVid = ar2VideoOpen( config );
    if( gVid == NULL ) return -1;

    return 0;
}

int arVideoClose( void )
{
	int result;
	
    if( gVid == NULL ) return -1;

	result = ar2VideoClose(gVid);
	gVid = NULL;
    return (result);
}

int arVideoInqSize( int *x, int *y )
{
    if( gVid == NULL ) return -1;

    return ar2VideoInqSize( gVid, x, y );
}

ARUint8 *arVideoGetImage( void )
{
    if( gVid == NULL ) return NULL;

    return ar2VideoGetImage( gVid );
}

int arVideoCapStart( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapStart( gVid );
}

int arVideoCapStop( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapStop( gVid );
}

int arVideoCapNext( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapNext( gVid );
}

/*-------------------------------------------*/


int ar2VideoDispOption( void )
{
    printf("ARVideo may be configured using one or more of the following options,\n");
    printf("separated by a space:\n\n");
    printf(" -size=[FULL/HALF]\n");
    printf("    specifies size of image.\n");
    printf(" -device=N\n");
    printf("    specifies device number.\n");
    printf(" -bufsize=N\n");
    printf("    specifies video buffer size.\n");
    printf("\n");

    return 0;
}

AR2VideoParamT *ar2VideoOpen( char *config )
{
    AR2VideoParamT    *vid;
    char              *a, line[256];
    int               i;

    arMalloc( vid, AR2VideoParamT, 1 );
    vid->did       = 0;
    vid->format    = AR_VIDEO_INTERLEAVED;
    vid->packing   = AR_VIDEO_RGB_8;
    if( DEFAULT_VIDEO_SIZE == VIDEO_FULL ) {
        vid->zoom      = AR_VIDEO_1_P_1;
    }
    else {
        vid->zoom      = AR_VIDEO_1_P_2;
    }
    vid->buf_size  = -1;
    
    a = config;
    if( a != NULL) {
        for(;;) {
            while( *a == ' ' || *a == '\t' ) a++;
            if( *a == '\0' ) break;

            if( strncmp( a, "-size=", 6 ) == 0 ) {
                if( strncmp( &a[6], "FULL", 4 ) == 0 )       vid->zoom = AR_VIDEO_1_P_1;
                else if( strncmp( &a[6], "HALF", 4 ) == 0 )  vid->zoom = AR_VIDEO_1_P_2;
                else {
                    arVideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-device=", 8 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[8], "%d", &vid->did ) == 0 ) {
                    arVideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-bufsize=", 9 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[9], "%d", &vid->buf_size ) == 0 ) {
                    arVideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else {
                arVideoDispOption();
                free( vid );
                return 0;
            }

            while( *a != ' ' && *a != '\t' && *a != '\0') a++;
        }
    }

    if( init  == 0 ) {
        if( arVideoOpen2() < 0 ) {
            free( vid );
            return 0;
        }
        init = 1;
    }

    if( arVideoSetupDevice2(vid->did, vid->format, vid->packing, vid->zoom) < 0 ) {
        free( vid );
        return 0;
    }
    if( vid->buf_size > 0 ) {
        arVideoSetBufferSize2( vid->did, vid->buf_size );
    }

    return vid;
}

int ar2VideoClose( AR2VideoParamT *vid )
{
    if(arVideoCleanupDevice2(vid->did) < 0) return -1;
    free( vid );

    return 0;
}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x,int *y)
{
    return arVideoInqSize2( vid->did, x, y );
}

int ar2VideoCapStart( AR2VideoParamT *vid )
{
    return arVideoStart2( vid->did );
}

int ar2VideoCapStop( AR2VideoParamT *vid )
{
    return arVideoStop2( vid->did );
}

int ar2VideoCapNext( AR2VideoParamT *vid )
{
    return 0;
}

ARUint8 *ar2VideoGetImage( AR2VideoParamT *vid )
{
    return arVideoGetImage2( vid->did );
}
