/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 1.0
 * Date: 01/12/08
 *
*******************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <AR/ar.h>
#include <AR/matrix.h>
#include <AR/arMulti.h>

int arMultiActivate(ARMultiMarkerInfoT *config)
{
    int    i;

    config->prevF = 0;

    for(i = 0; i < config->marker_num; i++) {
        if (arActivatePatt(config->marker[i].patt_id) != 1) return (-1);
    }

    return 0;
}

int arMultiDeactivate(ARMultiMarkerInfoT *config)
{
    int    i;

    config->prevF = 0;

    for(i = 0; i < config->marker_num; i++) {
        if (arDeactivatePatt(config->marker[i].patt_id)) return (-1);
    }

    return 0;
}

int arMultiFreeConfig(ARMultiMarkerInfoT *config)
{
    int    i;

    for(i = 0; i < config->marker_num; i++) {
        if (arFreePatt(config->marker[i].patt_id) != 1) return (-1);
    }
    free(config->marker);
    free(config);
    config = NULL;

    return 0;
}
