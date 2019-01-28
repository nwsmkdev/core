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
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)

/*    %Lowpass design
 Fs = 1
 Fc = 0.2
 [z,p,k] = cheby2(6,80,Fc/Fs);
 [sos]=zp2sos(z,p,k);
 fvtool(sos);
 info = stepinfo(zp2tf(p,z,k));
 sprintf("#define FS_XTALT_SETTLINGTIME %d", 2*ceil(info.SettlingTime))
 sos2c(sos,'g_fs_xtalt')
*/
#define FS_XTALT_SETTLINGTIME 17
static float g_fs_xtalt_b[] ={
         2.160326e-04, 9.661246e-05, 2.160326e-04,
         1.000000e+00, -1.302658e+00, 1.000000e+00,
         1.000000e+00, -1.593398e+00, 1.000000e+00,
         };
static float g_fs_xtalt_a[] ={
         1.000000e+00, -1.555858e+00, 6.083635e-01,
         1.000000e+00, -1.661260e+00, 7.136943e-01,
         1.000000e+00, -1.836731e+00, 8.911796e-01,
         };
/*
% From Figure 29 PPM vs Crystal Trim
p=polyfit([30,20,0,-18],[0,5,17,30],2)
mat2c(p,'g_fs_xtalt_poly')
*/
static float g_fs_xtalt_poly[] ={
         3.252948e-03, -6.641957e-01, 1.699287e+01,
         };
#endif


STATS_NAME_START(blink_stat_section)
    STATS_NAME(blink_stat_section, master_cnt)
    STATS_NAME(blink_stat_section, slave_cnt)
    STATS_NAME(blink_stat_section, send)
    STATS_NAME(blink_stat_section, listen)
    STATS_NAME(blink_stat_section, slave_cnt)
    STATS_NAME(blink_stat_section, tx_complete)
    STATS_NAME(blink_stat_section, rx_complete)
    STATS_NAME(blink_stat_section, rx_relayed)
    STATS_NAME(blink_stat_section, rx_unsolicited)
    STATS_NAME(blink_stat_section, rx_error)
    STATS_NAME(blink_stat_section, tx_start_error)
    STATS_NAME(blink_stat_section, tx_relay_error)
    STATS_NAME(blink_stat_section, tx_relay_ok)
    STATS_NAME(blink_stat_section, rx_timeout)
    STATS_NAME(blink_stat_section, reset)
STATS_NAME_END(blink_stat_section)


static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool blink_reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static struct _dw1000_blink_status_t dw1000_blink_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
static struct _dw1000_blink_status_t dw1000_blink_listen(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);

//static void blink_tasks_init(struct _dw1000_blink_instance_t * inst);
static void blink_timer_irq(void * arg);
static void blink_master_timer_ev_cb(struct os_event *ev);
static void blink_slave_timer_ev_cb(struct os_event *ev);

#if !MYNEWT_VAL(WCS_ENABLED)
static void blink_postprocess(struct os_event * ev);
#endif

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

    os_cputime_timer_init(&blink->timer, blink_timer_irq, (void *) inst);

    if (role == BLINK_ROLE_MASTER)
        os_callout_init(&blink->event_cb, &blink->eventq, blink_master_timer_ev_cb, (void *) inst);
    else
        os_callout_init(&blink->event_cb, &blink->eventq, blink_slave_timer_ev_cb, (void *) inst);

    os_cputime_timer_relative(&blink->timer, 0);
}

/**
 * blink_timer_event is in the interrupr context and schedules and tasks on the event queue
 *
 * @param arg   Pointer to  dw1000_dev_instance_t.
 * @return void
 */
static void
blink_timer_irq(void * arg){
   assert(arg);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *) arg;
    dw1000_blink_instance_t * blink = inst->blink;
//    os_eventq_put(&blink->eventq, &blink->event_cb.c_ev);
    if (blink->config.role == BLINK_ROLE_MASTER)
       blink_master_timer_ev_cb(&blink->event_cb.c_ev);
    else
        blink_slave_timer_ev_cb(&blink->event_cb.c_ev);
}



/**
 * The OS scheduler is not accurate enough for the timing requirement of an RTLS system.
 * Instead, the OS is used to schedule events in advance of the actual event.
 * The DW1000 delay start mechanism then takes care of the actual event. This removes the non-deterministic
 * latencies of the OS implementation.
 *
 * @param ev  Pointer to os_events.
 * @return void
 */
static void
blink_master_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_blink_instance_t * blink = inst->blink;

    STATS_INC(inst->blink->stat, master_cnt);

    if (dw1000_blink_send(inst, DWT_BLOCKING).start_tx_error){
        hal_timer_start_at(&blink->timer, blink->os_epoch
            + os_cputime_usecs_to_ticks((uint32_t)dw1000_dwt_usecs_to_usecs(blink->period) << 1)
        );
    }
#if 0
    else{
        hal_timer_start_at(&blink->timer, blink->os_epoch
            + os_cputime_usecs_to_ticks((uint32_t)dw1000_dwt_usecs_to_usecs(blink->period))
        );
    }
#endif
}

/**
 * Help function to calculate the delay between cascading blink relays
 *
 * @param inst Pointer to dw1000_dev_instance_t *
 * @param rx_slot 0 for master, and increasing
 * @param my_slot my_slot should be inst->slot_id - 1, master having slot_id=1 usually
 * @return void
 */
static uint32_t
usecs_to_response(dw1000_dev_instance_t * inst, int rx_slot, int my_slot)
{
    uint32_t blink_duration = dw1000_phy_frame_duration(&inst->attrib, sizeof(blink_blink_frame_t));
    uint32_t ret = ((uint32_t)inst->blink->config.tx_guard_dly + blink_duration);
    /* Master has slot 0 */
    if (rx_slot == 0) {
        ret *= my_slot - 1;
        ret += inst->blink->config.tx_holdoff_dly;
    } else {
        ret *= my_slot - rx_slot;
    }
    return ret;
}


/**
 * The OS scheduler is not accurate enough for the timing requirement of an RTLS system.
 * Instead, the OS is used to schedule events in advance of the actual event.
 * The DW1000 delay start mechanism then takes care of the actual event. This removes the non-deterministic
 * latencies of the OS implementation.
 *
 * @param ev  Pointer to os_events.
 * @return void
 */
static void
blink_slave_timer_ev_cb(struct os_event *ev) {

    DIAGMSG("{\"utime\": %lu,\"msg\": \"blink_slave_timer_ev_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_blink_instance_t * blink = inst->blink;

    STATS_INC(inst->blink->stat, slave_cnt);

    uint64_t dx_time = blink->epoch
            + ((uint64_t)inst->blink->period << 16)
            - ((uint64_t)ceilf(dw1000_usecs_to_dwt_usecs(dw1000_phy_SHR_duration(&inst->attrib))) << 16);

    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(blink_blink_frame_t))
                        + MYNEWT_VAL(XTALT_GUARD);

#if MYNEWT_VAL(BLINK_NUM_RELAYING_ANCHORS) != 0
    /* Adjust timeout if we're using cascading blink in anchors */
    timeout += usecs_to_response(inst, 0, MYNEWT_VAL(BLINK_NUM_RELAYING_ANCHORS));
#endif
    dw1000_set_rx_timeout(inst, timeout);
    dw1000_set_delay_start(inst, dx_time);
    dw1000_blink_status_t status = dw1000_blink_listen(inst, DWT_BLOCKING);
    if(status.start_rx_error){
        /* Sync lost, set a long rx timeout */
        dw1000_set_rx_timeout(inst, (uint16_t) 0xffff);
        dw1000_blink_listen(inst, DWT_BLOCKING);
    }
    // Schedule event
    hal_timer_start_at(&blink->timer, blink->os_epoch
        + os_cputime_usecs_to_ticks(
            - MYNEWT_VAL(OS_LATENCY)
            + (uint32_t)dw1000_dwt_usecs_to_usecs(blink->period)
            - dw1000_phy_frame_duration(&inst->attrib, sizeof(blink_blink_frame_t))
            )
        );
}
#if 0
static void
blink_task(void *arg)
{
    dw1000_blink_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->eventq);
    }
}
#endif
/**
 * The default eventq is used.
 *
 * @param inst Pointer to dw1000_dev_instance_t *
 * @return void
 */
#if 0
static void
blink_tasks_init(struct _dw1000_blink_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for tdma events */
        os_eventq_init(&inst->eventq);
        os_task_init(&inst->task_str, "dw1000_blink",
                     blink_task,
                     (void *) inst,
                     inst->task_prio + 3, OS_WAIT_FOREVER,
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
    }
}
#endif

/**
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame.
 * This approach solves the non-deterministic latencies caused by the OS. The OS, however, is used to schedule
 * the next transmission event, but the DW1000 controls the actual next transmission time using the dw1000_set_delay_start.
 * This function allocates all the required resources. In the case of a large scale deployment multiple instances
 * can be uses to track multiple clock domains.
 *
 * @param inst     Pointer to dw1000_dev_instance_t
 * @param nframes  Nominally set to 2 frames for the simple use case. But depending on the interpolation
 * algorithm this should be set accordingly. For example, four frames are required or bicubic interpolation.
 * @param clock_master  UUID address of the system clock_master all other masters are rejected.
 *
 * @return dw1000_blink_instance_t *
 */
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
    inst->blink->period = MYNEWT_VAL(BLINK_PERIOD);
    inst->blink->config = (dw1000_blink_config_t){
        .postprocess = false,
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
        .fs_xtalt_autotune = true,
#endif
        .tx_holdoff_dly = 0x300,
        .tx_guard_dly = 0x10,
    };
    inst->blink->uuid = uuid;

    os_error_t err = os_sem_init(&inst->blink->sem, 0x1);
    assert(err == OS_OK);
    inst->blink->status.valid = true;
#if MYNEWT_VAL(WCS_ENABLED)
    inst->blink->wcs = wcs_init(NULL, inst->blink);                 // Using wcs process
    dw1000_blink_set_postprocess(inst->blink, &wcs_update_cb);      // Using default process
#else
    dw1000_blink_set_postprocess(inst->blink, &blink_postprocess);    // Using default process
#endif

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

    //blink_tasks_init(inst->blink);
    inst->blink->os_epoch = os_cputime_get32();

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    inst->blink->xtalt_sos = sosfilt_init(NULL, sizeof(g_fs_xtalt_b)/sizeof(float)/BIQUAD_N);
#endif
    inst->blink->status.initialized = 1;

    int rc = stats_init(
                STATS_HDR(inst->blink->stat),
                STATS_SIZE_INIT_PARMS(inst->blink->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(blink_stat_section)
            );
    assert(rc == 0);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc = stats_register("blink", STATS_HDR(inst->blink->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        rc |= stats_register("blink0", STATS_HDR(inst->blink->stat));
    else
        rc |= stats_register("blink1", STATS_HDR(inst->blink->stat));
#endif
    assert(rc == 0);

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

#if MYNEWT_VAL(WCS_ENABLED)
    wcs_free(inst->wcs);
#endif
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    sosfilt_free(inst->xtalt_sos);
#endif
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

#if !MYNEWT_VAL(WCS_ENABLED)
/**
 * API that serves as a place holder for timescale processing and by default is creates json string for the event.
 *
 * @param ev   pointer to os_events.
 * @return void
 */
static void
blink_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_blink_instance_t * blink = (dw1000_blink_instance_t *)ev->ev_arg;

    blink_frame_t * previous_frame = blink->frames[(uint16_t)(blink->idx-1)%blink->nframes];
    blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];
    uint64_t delta = 0;

    if (blink->config.role == BLINK_ROLE_MASTER){
        delta = (frame->transmission_timestamp - previous_frame->transmission_timestamp);
    } else {
        delta = (frame->reception_timestamp - previous_frame->reception_timestamp);
    }
    delta = delta & ((uint64_t)1<<63)?delta & 0xFFFFFFFFFF :delta;

#if MYNEWT_VAL(BLINK_VERBOSE)
    float clock_offset = dw1000_calc_clock_offset_ratio(blink->parent, frame->carrier_integrator);
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        frame->transmission_timestamp,
        delta,
        *(uint32_t *)&clock_offset,
        frame->seq_num
    );
#endif
}
#endif


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
#if 0
    if(os_sem_get_count(&inst->blink->sem) != 0){
        //unsolicited inbound
        STATS_INC(inst->blink->stat, rx_unsolicited);
        return false;
    }
#endif

    if (inst->blink->config.role == BLINK_ROLE_MASTER) {
        return true;
    }
    DIAGMSG("{\"utime\": %lu,\"msg\": \"blink:rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_blink_instance_t * blink = inst->blink;
    blink_frame_t * frame = blink->frames[(blink->idx+1)%blink->nframes];  // speculative frame advance

    if (inst->frame_len >= sizeof(blink_blink_frame_t) && inst->frame_len <= sizeof(frame->array))
        memcpy(frame->array, inst->rxbuf, sizeof(blink_blink_frame_t));
    else
        return false;
#if 0
    /* Mask off the last 8 bits and compare to our blink->uuid master id */
    if((inst->blink->uuid & 0xffffffffffffff00UL) != (frame->long_address & 0xffffffffffffff00UL)) {
        return false;
    }
#endif
    if (inst->status.lde_error)
        return false;

    /* A good blink packet has been received, stop the receiver */
    dw1000_stop_rx(inst); //Prevent timeout event

    blink->idx++; // confirmed frame advance

    blink->os_epoch = os_cputime_get32();
    STATS_INC(inst->blink->stat, rx_complete);

    blink->epoch_master = frame->transmission_timestamp;
    blink->epoch = frame->reception_timestamp = inst->rxtimestamp;
    blink->period = frame->transmission_interval;
    frame->carrier_integrator = inst->carrier_integrator;
    blink->status.valid |= blink->idx > 1;

    memcpy(blink_time, &frame->reception_timestamp, 5);
    blink_seq_number = frame->seq_num;
    blink_firstPath = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 2);
    tag_address = frame->long_address;
    blink_rx = 1;

    /* Compensate if not receiving the master blink packet directly */
    int rx_slot = (frame->long_address & 0xff);
    if (rx_slot != 0x00) {
        STATS_INC(inst->blink->stat, rx_relayed);
        /* Assume blink intervals are a multiple of 0x10000 us */
        uint32_t master_interval = ((frame->transmission_interval/0x10000+1)*0x10000);
        blink->epoch_master -= (master_interval - frame->transmission_interval) << 16;
        blink->epoch -= (master_interval - frame->transmission_interval) << 16;
        blink->os_epoch -= os_cputime_usecs_to_ticks(master_interval - frame->transmission_interval);
    }

    /* Cascade relay of blink packet */
    if (blink->config.role == BLINK_ROLE_RELAY && blink->status.valid &&
        rx_slot < (inst->slot_id-1) && inst->slot_id != 0xffff) {
        blink_frame_t tx_frame;
        memcpy(tx_frame.array, frame->array, sizeof(blink_frame_t));
        uint64_t tx_timestamp = frame->reception_timestamp + (((uint64_t)usecs_to_response(inst, rx_slot, inst->slot_id-1))<<16);
        tx_timestamp &= 0x0FFFFFFFFFFUL;
        dw1000_set_delay_start(inst, tx_timestamp);

        /* Need to add antenna delay and tof compensation */
        tx_timestamp += inst->tx_antenna_delay + inst->blink->config.tof_compensation;

#if MYNEWT_VAL(WCS_ENABLED)
        tx_frame.transmission_timestamp = wcs_local_to_master(inst, tx_timestamp);
#else
        tx_frame.transmission_timestamp = frame->transmission_timestamp + tx_timestamp - frame->reception_timestamp;
#endif

        tx_frame.long_address = inst->blink->uuid | (inst->slot_id-1);
        tx_frame.transmission_interval = frame->transmission_interval - ((tx_frame.transmission_timestamp - frame->transmission_timestamp)>>16);

        dw1000_write_tx(inst, tx_frame.array, 0, sizeof(blink_blink_frame_t));
        dw1000_write_tx_fctrl(inst, sizeof(blink_blink_frame_t), 0, true);
        blink->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
        if (blink->status.start_tx_error){
            STATS_INC(inst->blink->stat, tx_relay_error);
        } else {
            STATS_INC(inst->blink->stat, tx_relay_ok);
        }
    }
#if 0
    if (blink->config.postprocess && blink->status.valid)
         os_eventq_put(&blink->eventq, &blink->callout_postprocess.c_ev);
        //os_eventq_put(os_eventq_dflt_get(), &blink->callout_postprocess.c_ev);
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    if (blink->config.fs_xtalt_autotune && blink->status.valid){
//        float fs_xtalt_offset = sosfilt(blink->xtalt_sos,  1e6 * ((float)tracking_offset) / tracking_interval, g_fs_xtalt_b, g_fs_xtalt_a);
        float fs_xtalt_offset = sosfilt(blink->xtalt_sos,  -1e6 * blink->wcs->skew, g_fs_xtalt_b, g_fs_xtalt_a);
        if(blink->xtalt_sos->clk % FS_XTALT_SETTLINGTIME == 0){
            int8_t reg = dw1000_read_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET, sizeof(uint8_t)) & FS_XTALT_MASK;
            int8_t trim_code = (int8_t) roundf(polyval(g_fs_xtalt_poly, fs_xtalt_offset, sizeof(g_fs_xtalt_poly)/sizeof(float))
                                - polyval(g_fs_xtalt_poly, 0, sizeof(g_fs_xtalt_poly)/sizeof(float)));
            if(reg - trim_code < 0)
                reg = 0;
            else if(reg - trim_code > FS_XTALT_MASK)
                reg = FS_XTALT_MASK;
            else
                reg = ((reg - trim_code) & FS_XTALT_MASK);
            dw1000_write_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET,  (3 << 5) | reg, sizeof(uint8_t));
        }
    }
#endif
   // os_sem_release(&inst->blink->sem);
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

    STATS_INC(inst->blink->stat, tx_complete);

//    if (inst->blink->config.role != BLINK_ROLE_MASTER)
//        return false;

    dw1000_blink_instance_t * blink = inst->blink;
    hal_gpio_toggle(LED_3);
    printf("blink-tx-cb\n");
    blink_frame_t * frame = blink->frames[(++blink->idx)%blink->nframes];

    blink->os_epoch = os_cputime_get32();
    blink->epoch = frame->transmission_timestamp = dw1000_read_txrawst(inst);
    blink->epoch_master = frame->transmission_timestamp;
    blink->period = frame->transmission_interval;

    if (blink->status.timer_enabled){
#if 1
      hal_timer_start_at(&blink->timer, blink->os_epoch
            - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY))
            + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(blink->period))
        );
#endif

    //os_cputime_timer_relative(&blink->timer, 1000000);
    }
    blink->status.valid |= blink->idx > 1;
    // Postprocess for tx_complete is used to generate tdma events on the clock master node.
    if (blink->config.postprocess && blink->status.valid)
        //os_eventq_put(&blink->eventq, &blink->callout_postprocess.c_ev);
        os_eventq_put(os_eventq_dflt_get(), &blink->callout_postprocess.c_ev);

    if(os_sem_get_count(&inst->blink->sem) == 0){
        os_error_t err = os_sem_release(&inst->blink->sem);
        assert(err == OS_OK);
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
    STATS_INC(inst->blink->stat, rx_error);

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
        STATS_INC(inst->blink->stat, tx_start_error);
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

        STATS_INC(inst->blink->stat, rx_timeout);
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
        STATS_INC(inst->blink->stat, reset);
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

    STATS_INC(inst->blink->stat,send);
    dw1000_blink_instance_t * blink = inst->blink;
    os_error_t err = os_sem_pend(&blink->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    hal_gpio_toggle(LED_3);
    blink_frame_t * previous_frame = blink->frames[(uint16_t)(blink->idx)%blink->nframes];
    blink_frame_t * frame = blink->frames[(blink->idx+1)%blink->nframes];

    frame->transmission_timestamp = (previous_frame->transmission_timestamp
                                    + ((uint64_t)inst->blink->period << 16)
                                    ) & 0x0FFFFFFFFFFUL;
    dw1000_set_delay_start(inst, frame->transmission_timestamp);
    frame->transmission_timestamp += inst->tx_antenna_delay;

    frame->seq_num = previous_frame->seq_num + 1;
    //printf("seq--%d\n",seq_number);
    //frame->long_address = inst->blink->uuid;
    frame->long_address = inst->my_long_address;
    frame->transmission_interval = inst->blink->period;

    dw1000_write_tx(inst, frame->array, 0, sizeof(blink_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(blink_blink_frame_t), 0, true);
    dw1000_set_wait4resp(inst, false);
    blink->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (blink->status.start_tx_error ){
        printf("tx-error\n");
        STATS_INC(inst->blink->stat, tx_start_error);
        previous_frame->transmission_timestamp = (frame->transmission_timestamp + ((uint64_t)inst->blink->period << 16)) & 0x0FFFFFFFFFFUL;
        blink->idx++;
        err =  os_sem_release(&blink->sem);
        assert(err == OS_OK);

    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&blink->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        //assert(err == OS_OK);
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
static dw1000_blink_status_t
dw1000_blink_listen(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
{
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_blink_listen\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_blink_instance_t * blink = inst->blink;
    os_error_t err = os_sem_pend(&blink->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(inst->blink->stat,listen);

    blink->status = (dw1000_blink_status_t){
        .rx_timeout_error = 0,
        .start_rx_error = 0
    };
    blink->status.start_rx_error = dw1000_start_rx(inst).start_rx_error;
    if (blink->status.start_rx_error){
        err = os_sem_release(&blink->sem);
        assert(err == OS_OK);
    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&blink->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        assert(err == OS_OK);
        err = os_sem_release(&blink->sem);
        assert(err == OS_OK);
    }
    return blink->status;
}



/**
 * API to start clock calibration packets (BLINK) blinks.
 * With a pulse repetition period of MYNEWT_VAL(BLINK_PERIOD).
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
void
dw1000_blink_start(struct _dw1000_dev_instance_t * inst, dw1000_blink_role_t role){
    // Initialise frame timestamp to current time
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_blink_start\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_blink_instance_t * blink = inst->blink;
    blink->idx = 0x0;
    blink->status.valid = false;
    blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];
    blink->config.role = role;

    if (blink->config.role == BLINK_ROLE_MASTER)
        blink->epoch = frame->transmission_timestamp = dw1000_read_systime(inst);
    else {
        blink->epoch = frame->reception_timestamp = dw1000_read_systime(inst);
        /* Temporarily override period to start listening for the first
         * blink packet sooner */
        blink->period = 5000;
    }

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
    os_cputime_timer_stop(&blink->timer);
}
