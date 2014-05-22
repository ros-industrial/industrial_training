#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libraw1394/raw1394.h>
#include <libdv/dv.h>
#include <AR/config.h>
#include <AR/ar.h>
#include <AR/video.h>

#define VIDEO_MODE_PAL             0
#define VIDEO_MODE_NTSC            1
#define DEFAULT_VIDEO_MODE         VIDEO_MODE_NTSC

#define ARV_BUF_FRAME_DATA    150000
#define ARV_NTSC_FRAME_SIZE   120000
#define ARV_PAL_FRAME_SIZE    144000

#define ARV_PACKET_NUM_NTSC      250
#define ARV_PACKET_NUM_PAL       300

#define ARV_BIG_PACKET_SIZE      492
#define ARV_PACKET_DATA_SIZE     480



static AR2VideoParamT   *gVid = NULL;

static void ar2VideoCapture(AR2VideoParamT *vid);
static int ar2VideoRawISOHandler(raw1394handle_t handle, int channel, size_t length, quadlet_t *data);
static int ar2VideoBusResetHandler(raw1394handle_t handle, unsigned int generation);
static int ar2VideoBufferInit(AR2VideoBufferT *buffer, int size);
static int ar2VideoBufferClose(AR2VideoBufferT *buffer);
static int ar2VideoBufferRead(AR2VideoBufferT *buffer, ARUint8 *dest, int size, int flag);
static int ar2VideoBufferWrite(AR2VideoBufferT *buffer, ARUint8 *src, int size, int flag);
static ARUint8 *ar2VideoBufferReadDV(AR2VideoParamT *vid);

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
    printf(" -mode=[PAL|NTSC]\n");
    printf("    specifies TV signal mode.\n");
    printf("\n");

    return 0;
}

AR2VideoParamT *ar2VideoOpen( char *config_in )
{
    struct raw1394_portinfo    g_pinf[16];
    AR2VideoParamT            *vid;
    char                      *config, *a, line[256];
    int                        numcards;
    int                        i;

    arMalloc( vid, AR2VideoParamT, 1 );
    vid->mode       = DEFAULT_VIDEO_MODE;
    vid->debug      = 0;
    vid->status     = 0;

	/* If no config string is supplied, we should use the environment variable, otherwise set a sane default */
	if (!config_in || !(config_in[0])) {
		/* None suppplied, lets see if the user supplied one from the shell */
		char *envconf = getenv ("ARTOOLKIT_CONFIG");
		if (envconf && envconf[0]) {
			config = envconf;
			printf ("Using config string from environment [%s].\n", envconf);
		} else {
			config = NULL;
			printf ("No video config string supplied, using defaults.\n");
		}
	} else {
		config = config_in;
		printf ("Using supplied video config string [%s].\n", config_in);
	}
	
    a = config;
    if( a != NULL) {
        for(;;) {
            while( *a == ' ' || *a == '\t' ) a++;
            if( *a == '\0' ) break;

            if( strncmp( a, "-mode=", 6 ) == 0 ) {
                if( strncmp( &a[6], "PAL", 3 ) == 0 )        vid->mode = VIDEO_MODE_PAL;
                else if( strncmp( &a[6], "NTSC", 4 ) == 0 )  vid->mode = VIDEO_MODE_NTSC;
                else {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            } else if( strncmp( a, "-debug", 6 ) == 0 ) {
                vid->debug = 1;
            } else {
                ar2VideoDispOption();
                free( vid );
                return 0;
            }

            while( *a != ' ' && *a != '\t' && *a != '\0') a++;
        }
    }


    if ((vid->handle = raw1394_new_handle()) == NULL) {
        free( vid );
        perror("raw1394 - couldn't get handle");
        return NULL;
    }
    
    if ((numcards = raw1394_get_port_info(vid->handle, g_pinf, 16)) < 0) {
        free( vid );
        perror("raw1394 - couldn't get card info");
        return NULL;
    }
    else {
        if( vid->debug ) {
            printf("NUMCARDS = %d\n", numcards);
            for (i = 0; i < numcards; i++) { 
                printf("%2d: %s\n", g_pinf[i].nodes, g_pinf[i].name);
            }
        }
    }

    if (raw1394_set_port(vid->handle, 0) < 0) {
        free(vid);
        perror("raw1394 - couldn't set port");
        return NULL;
    }

    if ((vid->dv_decoder = dv_decoder_new((vid->mode == VIDEO_MODE_NTSC), FALSE, FALSE)) == 0) {
		free(vid);
        return NULL;
    }
    vid->dv_decoder->quality = 5;
	if (vid->mode == VIDEO_MODE_NTSC) {
		vid->dv_decoder->height = 480;
		vid->dv_decoder->arg_video_system = 1; // video standard: 0=autoselect [default], 1=525/60 4:1:1 (NTSC), 2=625/50 4:2:0 (PAL,IEC 61834 DV), 3=625/50 4:1:1 (PAL,SMPTE 314M DV).
	} else { // (vid->mode == VIDEO_MODE_PAL)
		vid->dv_decoder->height = 576;
		vid->dv_decoder->arg_video_system = 2;
	}
    dv_init(FALSE, FALSE);

    arMalloc( vid->buffer, AR2VideoBufferT, 1 );
    vid->buffer->init = 0;
    ar2VideoBufferInit( vid->buffer, ARV_BUF_FRAME_DATA );

    arMalloc( vid->image, ARUint8, 720*576*4 ); // Make buffer big enough for PAL BGRA images.

    return vid;
}

int ar2VideoCapStart( AR2VideoParamT *vid )
{
    if( vid->status != 0 ) return -1;
    vid->status = 1;
    vid->packet_num = 0;

    pthread_create(&(vid->capture), NULL, (void * (*)(void *))ar2VideoCapture, vid);

    return 0;
}

int ar2VideoCapStop( AR2VideoParamT *vid )
{
    if( vid->status != 1 ) return -1;
    vid->status = 2;

    return 0;
}

int ar2VideoCapNext( AR2VideoParamT *vid )
{
    return 0;
}

int ar2VideoClose( AR2VideoParamT *vid )
{
    if( vid->status == 1 || vid->status == 2 ) vid->status = 0;
     else return -1;

    pthread_join( vid->capture, NULL );

    ar2VideoBufferClose(vid->buffer);
    free( vid->buffer );
    free( vid->image );

    raw1394_stop_fcp_listen(vid->handle);
    raw1394_destroy_handle(vid->handle);

    free( vid );

    return 0;
} 


  
static void ar2VideoCapture(AR2VideoParamT *vid)
{ 
    raw1394_set_userdata(vid->handle, vid);
    raw1394_set_bus_reset_handler(vid->handle, ar2VideoBusResetHandler);
    raw1394_set_iso_handler(vid->handle, 63, ar2VideoRawISOHandler);
    if( raw1394_start_iso_rcv(vid->handle, 63) < 0 ) {
        perror("raw1394 - couldn't start iso receive");
        exit(-1);
    }
  
    for(;;) {
        if( vid->status == 1 ) {
            while ( vid->status == 1 ) {
                raw1394_loop_iterate(vid->handle);
            } 
            raw1394_stop_iso_rcv(vid->handle, 63);
        }
        else if( vid->status == 2 ) usleep(10);
        else break;
    }

    return;
}

static int ar2VideoRawISOHandler(raw1394handle_t handle, int channel, size_t length, quadlet_t *data)
{
    AR2VideoParamT  *vid = (AR2VideoParamT *)raw1394_get_userdata(handle);
    ARUint8         *packet = (ARUint8 *)data;
    int              len = 0;

    if(length == ARV_BIG_PACKET_SIZE) {
        if(vid->packet_num == 0) {
            if(packet[12] == 0x1f && packet[13] == 0x07) {
                if( vid->debug ) printf("Receiving...\n");
            }
            else return 0;
        }

        if (packet[12] == 0x1f && packet[13] == 0x07) {
            if(vid->packet_num == 0) {
                vid->packet_num++;
                len = ar2VideoBufferWrite(vid->buffer, (ARUint8 *)(data+3), ARV_PACKET_DATA_SIZE, 0);
            }
            else {
                vid->packet_num = 0;
                vid->packet_num++;
                len = ar2VideoBufferWrite(vid->buffer, (ARUint8 *)(data+3), ARV_PACKET_DATA_SIZE, 2);
            }
        }
        else {
            vid->packet_num++;
            if( (vid->mode == VIDEO_MODE_NTSC && vid->packet_num == ARV_PACKET_NUM_NTSC)
             || (vid->mode == VIDEO_MODE_PAL  && vid->packet_num == ARV_PACKET_NUM_PAL) ) {
                len = ar2VideoBufferWrite(vid->buffer, (ARUint8 *)(data+3), ARV_PACKET_DATA_SIZE, 1);
                vid->packet_num = 0;
            }
            else {
                len = ar2VideoBufferWrite(vid->buffer, (ARUint8 *)(data+3), ARV_PACKET_DATA_SIZE, 0);
            }
        }
    }

    return len;
}

static int ar2VideoBusResetHandler(raw1394handle_t handle, unsigned int generation)
{
    static int       i = 0;
    AR2VideoParamT  *vid = (AR2VideoParamT *)raw1394_get_userdata(handle);

    fprintf(stderr,"reset %d\n", i++);
    if (i == 10) {
        vid->status = 0;
    }

    return 0;
}

ARUint8 *ar2VideoGetImage( AR2VideoParamT *vid )
{
    return ar2VideoBufferReadDV( vid );
}

static ARUint8 *ar2VideoBufferReadDV(AR2VideoParamT *vid)
{
    static int     f = 1;
    ARUint8       *tmp;
    int            read_size;
    int            pitches[3];
    unsigned char *pixels[3];

    if( vid->buffer->init == 0 ) return NULL;

    pthread_mutex_lock(&(vid->buffer->mutex));
    tmp = vid->buffer->buff_wait;
    vid->buffer->buff_wait = vid->buffer->buff_out;
    vid->buffer->buff_out = tmp;

    vid->buffer->fill_size_out = vid->buffer->fill_size_wait;
    vid->buffer->read_size = 0;
    vid->buffer->fill_size_wait = 0;
    pthread_mutex_unlock(&(vid->buffer->mutex));
 
    if( vid->mode == VIDEO_MODE_NTSC ) {
        if( vid->buffer->fill_size_out != ARV_NTSC_FRAME_SIZE ) return NULL;
    }
    else if( vid->mode == VIDEO_MODE_PAL ) {
        if( vid->buffer->fill_size_out != ARV_PAL_FRAME_SIZE ) return NULL;
    }
    if( f ) {
        dv_parse_header(vid->dv_decoder, vid->image);
        if( vid->mode == VIDEO_MODE_NTSC ) {
            if( vid->dv_decoder->width != 720 || vid->dv_decoder->height != 480 ) {
                printf("Image format is not correct.\n");
                return NULL;
            }
        }
        else if( vid->mode == VIDEO_MODE_PAL ) {
            if( vid->dv_decoder->width != 720 || vid->dv_decoder->height != 576 ) {
                printf("Image format is not correct.\n");
                return NULL;
            }
        }
        f = 0;
    }

    pitches[0] = 720*AR_PIX_SIZE_DEFAULT;
    pixels[0] =  vid->image;
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
    dv_decode_full_frame(vid->dv_decoder, vid->buffer->buff_out, e_dv_color_rgb, pixels, pitches );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
    dv_decode_full_frame(vid->dv_decoder, vid->buffer->buff_out, e_dv_color_bgr0, pixels, pitches );
#else
#  error Unsupported pixel format defined in <AR/config.h>.
#endif

    return vid->image;
}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x,int *y)
{
    if( vid->mode == VIDEO_MODE_NTSC ) {
        *x = 720;
        *y = 480;
    }
    else if( vid->mode == VIDEO_MODE_PAL ) {
        *x = 720;
        *y = 576;
    }
    else return -1;
  
    return 0;
}

static int ar2VideoBufferInit(AR2VideoBufferT *buffer, int size)
{
    if( buffer->init ) return -1;

    buffer->size = size;

    arMalloc( buffer->buff_in,   ARUint8, size );
    arMalloc( buffer->buff_wait, ARUint8, size );
    arMalloc( buffer->buff_out,  ARUint8, size );
    buffer->fill_size_in   = 0;
    buffer->fill_size_wait = 0;
    buffer->fill_size_out  = 0;
    buffer->read_size      = 0;

    pthread_mutex_init(&(buffer->mutex), NULL);

    buffer->init = 1;

    return 0;
}

static int ar2VideoBufferClose(AR2VideoBufferT *buffer)
{
    if( buffer->init == 0 ) return -1;

    pthread_mutex_lock(&(buffer->mutex));
    free( buffer->buff_in   );
    free( buffer->buff_wait );
    free( buffer->buff_out  );
    pthread_mutex_unlock(&(buffer->mutex));
    pthread_mutex_destroy(&(buffer->mutex));

    buffer->init = 0;

    return 0;
}

static int ar2VideoBufferRead(AR2VideoBufferT *buffer, ARUint8 *dest, int size, int flag)
{
    ARUint8   *tmp;
    int        read_size;

    if( buffer->init == 0 ) return -1;

    if( flag ) {
        pthread_mutex_lock(&(buffer->mutex));
        tmp = buffer->buff_wait;
        buffer->buff_wait = buffer->buff_out;
        buffer->buff_out = tmp;

        buffer->fill_size_out = buffer->fill_size_wait;
        buffer->read_size = 0;
        buffer->fill_size_wait = 0;
        pthread_mutex_unlock(&(buffer->mutex));
    }
 
    if( buffer->fill_size_out - buffer->read_size >= size ) read_size = size;
     else                                                   read_size = buffer->fill_size_out - buffer->read_size;
    memcpy(dest, buffer->buff_out + buffer->read_size, read_size);
    buffer->read_size += read_size;

    return read_size;
}

static int ar2VideoBufferWrite(AR2VideoBufferT *buffer, ARUint8 *src, int size, int flag)
{
    ARUint8   *tmp;
    int        write_size;

    if( buffer->init == 0 ) return -1;

    if( flag == 2 ) {
        buffer->fill_size_in = 0;
    }

    if( buffer->size - buffer->fill_size_in > size ) write_size = size;
     else                                            write_size = buffer->size - buffer->fill_size_in;
    memcpy(buffer->buff_in + buffer->fill_size_in, src, write_size);
    buffer->fill_size_in += write_size;

    if( flag == 1 ) {
        pthread_mutex_lock(&(buffer->mutex));
        tmp = buffer->buff_wait;
        buffer->buff_wait = buffer->buff_in;
        buffer->buff_in = tmp;

        buffer->fill_size_wait = buffer->fill_size_in;
        buffer->fill_size_in = 0;
        pthread_mutex_unlock(&(buffer->mutex));
    }

    return write_size;
}
