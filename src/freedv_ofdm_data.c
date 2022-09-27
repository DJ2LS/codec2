#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "fsk.h"
#include "fmfsk.h"
#include "codec2.h"
#include "codec2_fdmdv.h"
#include "varicode.h"
#include "freedv_api.h"
#include "freedv_api_internal.h"
#include "comp_prim.h"

#include "codec2_ofdm.h"
#include "ofdm_internal.h"
#include "mpdecode_core.h"
#include "gp_interleaver.h"
#include "ldpc_codes.h"
#include "interldpc.h"
#include "debug_alloc.h"
#include "filter.h"


// open function for OFDM advanced data modes
void freedv_ofdm_data_adv_open(struct freedv *f, struct freedv_ofdm_advanced *adv) {
    assert(adv != NULL);


    //ofdm_config = (struct OFDM *) CALLOC(1, sizeof (struct OFDM));

    struct OFDM_CONFIG ofdm_config;
    char mode[32];
    if (f->mode == FREEDV_MODE_OFDM_ADV) strcpy(mode, "ofdm_adv");




    // lets do an ofdm init and create an example config
    ofdm_init_mode(mode, &ofdm_config);
    
    //strcpy(ofdm_config.codename, "H_128_256_5");
    //ofdm_config.codename = "H_128_256_5";
    //ofdm_config.nc = 3;
    //ofdm_config.np = 12;
    

    //printf("\ncodename-param0: %s", *adv->codename);
    //printf("\n ---codename-param1: %s \n", adv->codename);
    //printf("\n ---tx_uw: %s \n", adv->tx_uw);


    // now override settings    
    ofdm_config.nc = adv->nc;                            
    ofdm_config.np = adv->np;
    ofdm_config.ns = adv->ns;
    ofdm_config.bps = adv->bps;                             
    ofdm_config.nuwbits = adv->nuwbits;          
    ofdm_config.bad_uw_errors = adv->bad_uw_errors;
    ofdm_config.ftwindowwidth = adv->ftwindowwidth;
    ofdm_config.edge_pilots = adv->edge_pilots;
    ofdm_config.ts = adv->ts;
    ofdm_config.tcp = adv->tcp;                        
    ofdm_config.timing_mx_thresh = adv->timing_mx_thresh;
    ofdm_config.tx_bpf_en = adv->tx_bpf_en;
    //ofdm_config.amp_scale = adv->amp_scale;    
    ofdm_config.clip_gain1 = adv->clip_gain1;
    ofdm_config.clip_gain2 = adv->clip_gain2;
    ofdm_config.clip_en = adv->clip_en;
    ofdm_config.txtbits = adv->txtbits;
    ofdm_config.state_machine = adv->state_machine;
    ofdm_config.data_mode = adv->data_mode;
    ofdm_config.codename = adv->codename; 
    ofdm_config.amp_est_mode = adv->amp_est_mode; 


    //int i;
    //for(i = 0; i < sizeof(ofdm_config.tx_uw); i++){
    //    printf("%i", ofdm_config.tx_uw[i]);
    //}
    printf("\n length1: %li\n", sizeof(adv->tx_uw));
    printf("\n length2: %li\n", sizeof(adv->tx_uw[0]));
    printf("\n ---- \n");
    printf("0:%i\n", adv->tx_uw[0]);
    printf("1:%i\n", adv->tx_uw[1]);
    printf("63:%i\n", adv->tx_uw[63]);
    printf("64:%i\n", adv->tx_uw[64]);
    printf("\n ---- \n");

    
    int p;
    for(p = -4; p < 60; p++){
        printf("%i", adv->tx_uw[p]);
    }

    printf("\n ---- \n");
    memcpy(ofdm_config.tx_uw, adv->tx_uw, sizeof(adv->tx_uw));



    int i;
    for(i = 0; i < 32; i++){
        printf("%i", ofdm_config.tx_uw[i]);
    }

    //uint8_t uw[] = {1,1,0,0, 1,0,1,0,  1,1,1,1, 0,0,0,0};
    //uint8_t uw[] = adv->tx_uw;


        
    //ofdm_config.tx_centre = adv->tx_centre;                
    //ofdm_config.rx_centre = adv->rx_centre;                
    ////ofdm_config.fs = adv->fs;                       
    ////ofdm_config.bps = adv->bps;                            
    ////ofdm_config.foff_limiter = adv->foff_limiter;
    
    
    
    
    
    
    // after setting our own settings, create ofdm modem instance
    f->ofdm = ofdm_create(&ofdm_config);

    assert(f->ofdm != NULL);
    

    // LDPC set up
    f->ldpc = (struct LDPC*)MALLOC(sizeof(struct LDPC));
    assert(f->ldpc != NULL);
    ldpc_codes_setup(f->ldpc, f->ofdm->codename);
    
#ifdef __EMBEDDED__
    f->ldpc->max_iter = 10; /* limit LDPC decoder iterations to limit CPU load */
#endif

    // useful constants
    f->ofdm_bitsperpacket = ofdm_get_bits_per_packet(f->ofdm);
    f->ofdm_bitsperframe = ofdm_get_bits_per_frame(f->ofdm);
    f->ofdm_nuwbits = ofdm_config.nuwbits;
    f->ofdm_ntxtbits = ofdm_config.txtbits;

    /* payload bits per FreeDV API "frame".  In OFDM modem nomenclature this is the number of
       payload data bits per packet, or the number of data bits in a LDPC codeword */
    f->bits_per_modem_frame = f->ldpc->data_bits_per_frame;
    
    // buffers for received symbols for one packet/LDPC codeword - may span many OFDM modem frames
    int Nsymsperpacket = ofdm_get_bits_per_packet(f->ofdm) / f->ofdm->bps;
    f->rx_syms = (COMP*)MALLOC(sizeof(COMP) * Nsymsperpacket);
    assert(f->rx_syms != NULL);
    f->rx_amps = (float*)MALLOC(sizeof(float) * Nsymsperpacket);
    assert(f->rx_amps != NULL);
    for(int i=0; i<Nsymsperpacket; i++) {
        f->rx_syms[i].real = f->rx_syms[i].imag = 0.0;
        f->rx_amps[i]= 0.0;
    }

    f->nin = f->nin_prev = ofdm_get_nin(f->ofdm);
    f->n_nat_modem_samples = ofdm_get_samples_per_packet(f->ofdm);
    f->n_nom_modem_samples = ofdm_get_samples_per_frame(f->ofdm);
    /* in burst mode we might jump a preamble frame */
    f->n_max_modem_samples = 2*ofdm_get_max_samples_per_frame(f->ofdm);
    f->modem_sample_rate = f->ofdm->config.fs;
    f->sz_error_pattern = f->ofdm_bitsperpacket;

    // Note inconsistency: freedv API modem "frame" is a OFDM modem packet
    f->tx_payload_bits = (unsigned char*)MALLOC(f->bits_per_modem_frame);
    assert(f->tx_payload_bits != NULL);
    f->rx_payload_bits = (unsigned char*)MALLOC(f->bits_per_modem_frame);
    assert(f->rx_payload_bits != NULL);
    
}

