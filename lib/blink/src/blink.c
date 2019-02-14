/*
 * Copyright 2018, Decawave Limited, All Rights Reserved
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/**
 * @file dw1000_blink.c
 * @author paul kettle
 * @date 2018
 * @brief clock calibration packets
 *
 * @details This is the blink base class which utilises the functions to enable/disable the configurations related to blink.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>
#include <bsp/bsp.h>

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(BLINK_ENABLED)
#include <blink/blink.h>
#endif
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static struct _dw1000_blink_status_t dw1000_blink_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
static void blink_master_timer_ev_cb(struct os_event *ev);

/**
 * API to initiate timer for blink.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
static void
blink_timer_init(struct _dw1000_dev_instance_t * inst, dw1000_blink_role_t role) {

    dw1000_blink_instance_t * blink = inst->blink;
    blink->status.timer_enabled = true;

    if (role == BLINK_ROLE_MASTER)
        os_callout_init(&blink->event_cb, os_eventq_dflt_get(), blink_master_timer_ev_cb, (void *) inst);
    os_callout_reset(&blink->event_cb, OS_TICKS_PER_SEC/MYNEWT_VAL(BLINK_RATE));
}

dw1000_blink_instance_t *
dw1000_blink_init(struct _dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t uuid){
    assert(inst);
    assert(nframes > 1);

    if (inst->blink == NULL ) {
        dw1000_blink_instance_t * blink = (dw1000_blink_instance_t *) malloc(sizeof(dw1000_blink_instance_t) + nframes * sizeof(blink_frame_t *));
        assert(blink);
        memset(blink, 0, sizeof(dw1000_blink_instance_t));
        blink->status.selfmalloc = 1;
        blink->nframes = nframes;
        blink_frame_t blink_default = {
            .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
            .seq_num = 0xFF
        };

        for (uint16_t i = 0; i < blink->nframes; i++){
            blink->frames[i] = (blink_frame_t *) malloc(sizeof(blink_frame_t));
            assert(blink->frames[i]);
            memcpy(blink->frames[i], &blink_default, sizeof(blink_frame_t));
            blink->frames[i]->seq_num = 0;
        }

        blink->parent = inst;
        inst->blink = blink;
        blink->task_prio = inst->task_prio - 0x4;

    }else{
        assert(inst->blink->nframes == nframes);
    }
    inst->blink->config = (dw1000_blink_config_t){
        .postprocess = false,
        .tx_holdoff_dly = 0x300,
        .tx_guard_dly = 0x10,
    };
    inst->blink->uuid = uuid;

    os_error_t err = os_sem_init(&inst->blink->sem, 0x1);
    assert(err == OS_OK);
    inst->blink->status.valid = true;

    inst->blink->cbs = (dw1000_mac_interface_t){
        .id = DW1000_BLINK,
        .tx_complete_cb = blink_tx_complete_cb,
        .rx_complete_cb = rx_complete_cb,
        .rx_timeout_cb = blink_rx_timeout_cb,
        .rx_error_cb = blink_rx_error_cb,
        .tx_error_cb = blink_tx_error_cb,
        .reset_cb = blink_reset_cb
    };
    dw1000_mac_append_interface(inst, &inst->blink->cbs);
    inst->blink->os_epoch = os_cputime_get32();
    inst->blink->status.initialized = 1;
    return inst->blink;
}

/**
 * Deconstructor
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
void
dw1000_blink_free(dw1000_blink_instance_t * inst){
    assert(inst);
    os_sem_release(&inst->sem);

    if (inst->status.selfmalloc){
        for (uint16_t i = 0; i < inst->nframes; i++)
            free(inst->frames[i]);
        free(inst);
    }
    else
        inst->status.initialized = 0;
}

/**
 * API to initialise the package, only one blink service required in the system.
 *
 * @return void
 */

void blink_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"blink_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_blink_init(hal_dw1000_inst(0), 2, MYNEWT_VAL(UUID_BLINK_MASTER));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_blink_init(hal_dw1000_inst(1), 2, MYNEWT_VAL(UUID_BLINK_MASTER));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_blink_init(hal_dw1000_inst(2), 2, MYNEWT_VAL(UUID_BLINK_MASTER));
#endif

}

/**
 * API that overrides the default post-processing behaviors, replacing the JSON stream with an alternative
 * or an advanced timescale processing algorithm.
 *
 * @param inst              Pointer to dw1000_dev_instance_t.
 * @param blink_postprocess   Pointer to os_events.
 * @return void
 */
void
dw1000_blink_set_postprocess(dw1000_blink_instance_t * inst, os_event_fn * postprocess){
    os_callout_init(&inst->callout_postprocess, os_eventq_dflt_get(), postprocess, (void *) inst);
//    os_callout_init(&inst->callout_postprocess, &inst->eventq,  postprocess, (void *) inst);
    inst->config.postprocess = true;
}


/**
 * Precise timing is achieved using the reception_timestamp and tracking intervals along with
 * the correction factor. For timescale processing, a postprocessing  callback is placed in the eventq.
 * This callback with FS_XTALT_AUTOTUNE_ENABLED set, uses the RX_TTCKO_ID register to compensate for crystal offset and drift. This is an
 * adaptive loop with a time constant of minutes. By aligning the crystals within the network RF TX power is optimum.
 * Note: Precise RTLS timing still relies on timescale algorithm.
 *
 * The fs_xtalt adjustments align crystals to 1us (1PPM) while timescale processing resolves timestamps to sub 1ns.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */

uint8_t blink_time[5];
uint8_t blink_seq_number = 0;
uint8_t blink_rx = 0;
uint16_t blink_firstPath = 0;
uint64_t tag_address;
static bool
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(os_sem_get_count(&inst->blink->sem) == 0){
            dw1000_set_rx_timeout(inst, (uint16_t) 0xffff);
            return true;
        }
        return false;
    }
    DIAGMSG("{\"utime\": %lu,\"msg\": \"blink:rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    printf("b-rx\n");
    dw1000_blink_instance_t * blink = inst->blink;
    blink_frame_t * frame = blink->frames[(blink->idx+1)%blink->nframes];  // speculative frame advance

    if (inst->frame_len >= sizeof(blink_blink_frame_t) && inst->frame_len <= sizeof(frame->array))
        memcpy(frame->array, inst->rxbuf, sizeof(blink_blink_frame_t));
    else
        return false;
    if (inst->status.lde_error)
        return false;

    /* A good blink packet has been received, stop the receiver */
    dw1000_stop_rx(inst); //Prevent timeout event

    blink->idx++; // confirmed frame advance

    memcpy(blink_time, &inst->rxtimestamp, 5);
    blink_seq_number = frame->seq_num;
    blink_firstPath = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 2);
    tag_address = frame->long_address;
    blink_rx = 1;
    /* Compensate if not receiving the master blink packet directly */
    //os_sem_release(&inst->blink->sem);
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
    //printf("rx-cb %d\n",seq_number);
    return false;
}

/**
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. This static
 * function is called on successful transmission of a BLINK packet, and this advances the frame index point. Circular addressing is used
 * for the frame addressing. The next os_event is scheduled to occur in (MYNEWT_VAL(BLINK_PERIOD) - MYNEWT_VAL(BLINK_OS_LATENCY)) usec
 * from now. This provided a context switch guard zone. The assumption is that the underlying OS will have sufficient time start
 * the call dw1000_set_delay_start within dw1000_blink_blink.
 *
 * @param inst    Pointer to dw1000_dev_instance_t
 * @return bool
 */
static bool
blink_tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64)
        return false;

    dw1000_blink_instance_t * blink = inst->blink;
    hal_gpio_toggle(LED_3);
    printf("blink-tx-cb\n");

    if (blink->status.timer_enabled){
        os_callout_reset(&blink->event_cb, OS_TICKS_PER_SEC/MYNEWT_VAL(BLINK_RATE));
    }
    return false;
}

/**
 * API for rx_error_cb of blink.
 *
 * @param inst   pointer to dw1000_dev_instance_t.
 * @return void
 */
static bool
blink_rx_error_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_blink_instance_t * blink = inst->blink;

    if (blink->config.role == BLINK_ROLE_MASTER)
        return false;

    // Release semaphore if rxauto enable is not set.
    if(inst->config.rxauto_enable)
        return false;

    else if(os_sem_get_count(&inst->blink->sem) == 0){
        os_error_t err = os_sem_release(&inst->blink->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}

/**
 * API for tx_error_cb of blink.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return bool
 */
static bool
blink_tx_error_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_blink_instance_t * blink = inst->blink;
    if (blink->config.role == BLINK_ROLE_SLAVE)
        return false;

    if(inst->fctrl_array[0] == FCNTL_IEEE_BLINK_TAG_64){
        if(os_sem_get_count(&inst->blink->sem) == 0){
            os_error_t err = os_sem_release(&inst->blink->sem);
            assert(err == OS_OK);
        return true;
        }
    }
    return false;
}

/**
 * API for rx_timeout_cb of blink.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
static bool
blink_rx_timeout_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_blink_instance_t * blink = inst->blink;
    if (blink->config.role == BLINK_ROLE_MASTER)
        return false;

    if (os_sem_get_count(&blink->sem) == 0){
        os_error_t err = os_sem_release(&blink->sem);
        assert(err == OS_OK);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"blink:rx_timeout_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

        return true;
    }
    return false;
}


/**
 * API for blink_reset_cb of blink.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
static bool
blink_reset_cb(struct _dw1000_dev_instance_t * inst,  dw1000_mac_interface_t * cbs){
    /* Place holder */
    if(os_sem_get_count(&inst->blink->sem) == 0){
        os_error_t err = os_sem_release(&inst->blink->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;   // BLINK is an observer and should not return true
}


/**
 * @fn dw1000_blink_send(dw1000_dev_instance_t * inst, dw1000_blink_modes_t mode)
 * API that start clock calibration packets (BLINK) blinks  with a pulse repetition period of MYNEWT_VAL(BLINK_PERIOD).
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame.
 * This removes the need to explicitly read the systime register and the assiciated non-deterministic latencies.
 * This function is static function for internl use. It will force a Half Period Delay Warning is called at
 * out of sequence.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param mode   dw1000_blink_modes_t for BLINK_BLOCKING, BLINK_NON_BLOCKING modes.
 * @return dw1000_blink_status_t
 */
static dw1000_blink_status_t
dw1000_blink_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
{

    dw1000_blink_instance_t * blink = inst->blink;
    os_error_t err = os_sem_pend(&blink->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    hal_gpio_toggle(LED_3);
    blink_frame_t * frame = blink->frames[(blink->idx+1)%blink->nframes];

    frame->seq_num = frame->seq_num + 1;
    printf("blink-seq == %d\n",frame->seq_num);
    frame->long_address = inst->my_long_address;

    dw1000_write_tx(inst, frame->array, 0, sizeof(blink_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(blink_blink_frame_t), 0, true);
    dw1000_set_wait4resp(inst, false);
    blink->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (blink->status.start_tx_error ){
        printf("tx-error\n");
        blink->idx++;
        err =  os_sem_release(&blink->sem);
        assert(err == OS_OK);

    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&blink->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        assert(err == OS_OK);
        err =  os_sem_release(&blink->sem);
        assert(err == OS_OK);
    }
    return blink->status;
}

/*!
 * @fn dw1000_blink_receive(dw1000_dev_instance_t * inst, dw1000_blink_modes_t mode)
 *
 * @brief Explicit entry function for reveicing a blink frame.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t *
 * @param mode - dw1000_blink_modes_t for BLINK_BLOCKING, BLINK_NONBLOCKING modes.
 *
 * output parameters
 *
 * returns dw1000_blink_status_t
 */
void
dw1000_blink_start(struct _dw1000_dev_instance_t * inst, dw1000_blink_role_t role){
    // Initialise frame timestamp to current time
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_blink_start\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_blink_instance_t * blink = inst->blink;
    blink->idx = 0x0;
    blink->status.valid = false;
    blink->config.role = role;
    blink_timer_init(inst, role);
}

/**
 * API to stop clock calibration packets (BLINK) blinks.
 *
 * @param inst   Pointer to  dw1000_dev_instance_t.
 * @return void
 */
void
dw1000_blink_stop(dw1000_dev_instance_t * inst){
    dw1000_blink_instance_t * blink = inst->blink;
    os_callout_stop(&blink->event_cb);
}

static void
blink_master_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_blink_instance_t * blink = inst->blink;
    os_sem_release(&blink->sem);
    if (dw1000_blink_send(inst, DWT_BLOCKING).start_tx_error){
        printf("blink-tx-error\n");
        os_callout_reset(&blink->event_cb, OS_TICKS_PER_SEC/MYNEWT_VAL(BLINK_RATE));
    }
}
