#include <stdio.h>
#include <math.h>
#include <string.h>
#include "DWM_comm.h"
#include "matrix.h"
#include "servo.h"

// Used to monitor interrupt sequence from DWM. Since two events can cause an interrupt and have no timing gaurantee,
//      a current and previous counter are used to monitor the interrupt state. Under effectively every environment
//      this is not needed, but with known interrupt order, counters prevent race conditions in the FSMs.
volatile uint32_t         DWM_IRQ_SEQ     = 0U;
         uint32_t         DWM_IRQ_PREV    = 0U;
volatile uint8_t          TX_WATCHDOG_EXP = FALSE; // expired timer
// Communication vars
#if (CONFIG == TAG)
TAG_METADATA    TAG_DATA    = {0};
#endif

#if (CONFIG == TAG)
/* TAG_Network():
FUNCTION:
    This function pings every node in the network to obtain the distance from the tag node.
    This is done with multiple samples with an applied median + averaging filter. The coordinate
    is calculated based on these distances, and is passed along the the relevant nodes (GIM/SCN).
    Additional filters can be elected (RBF, simple)
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    dest_coord: The final coordinate of the tag node (only used during INIT transformation tag location recording)
*/
void TAG_Network(SPI_HandleTypeDef* hspi, COORD* dest_coord) {
    // reset to first node for distance ping as gimbal node
    uint16_t dest_addr = GIM_ADDR;
    DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
    
    // samples must be power of 2 due to merging algorithm used for samples
    uint16_t samples = 2; // number of median filtered samples (~24 Hz coordinate update rate at 4; 2 would be ~60 Hz; 1 would be ~100 Hz)
    static float sample_set[128][5];
    if (dest_coord != NULL) { // init phase (recording tag location to transform anchor geometry)
        samples = 128;
    }
    for (int i = 0; i < samples; i++) {
        TAG_DATA.verified_nodes = GIM_UNVERIFIED;
        // collect distances to all anchor nodes
        while (TAG_DATA.verified_nodes != ALL_NODES_VERIFIED) {
            // Filter noisy/faulty data with a 3 sample redundant filter. Even though the
            //  target is moving, the samples happen fast enough that movement fits within typical noise profile.
            static float batch[3];
            // collect 3 batch samples per node
            for(int batch_sample = 0; batch_sample < 3; batch_sample++) {
                batch[batch_sample] = rangingMaster_stateMachine(hspi);;
            }
            // find median of 3 samples
            uint8_t median_idx = argMedian(batch[0], batch[1], batch[2]);
            // find delta to median of the 2 other samples
            float diff_med1  = batch[(median_idx + 1) % 3] - batch[median_idx];
            float diff_med2  = batch[(median_idx + 2) % 3] - batch[median_idx];
            // compute average of batch, discarding outside of delta tolerance samples
            float batch_avg = 0;
            uint32_t div    = 1;
            if (fabsf(diff_med1) < MAX_SAMPLE_DELTA) {
                batch_avg += diff_med1;
                div++;
            }
            if (fabsf(diff_med2) < MAX_SAMPLE_DELTA) {
                batch_avg += diff_med2;
                div++;
            }
            batch_avg /= div;
            batch_avg += batch[median_idx];
            
            if (batch_avg < 2E4) { // if more than 200m (physically unlikely), it was a faulty set of samples
                // add batch average to dist buffer
                TAG_DATA.node_dists[TAG_DATA.verified_nodes++] = batch_avg;
            }
            // move onto next anchor for ranging samples; only increments to next addr if previous batch was not faulty
            uint16_t dest_addr = GIM_ADDR + ((TAG_DATA.verified_nodes == ALL_NODES_VERIFIED) ? 0 : TAG_DATA.verified_nodes); // shortcut
            DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
        }
        for (int j = 0; j < 5; j++) {
            sample_set[i][j] = TAG_DATA.node_dists[j] / samples;
        }
    }
    // logN merge of the sample set. This is done this way to preserve exponents across each merge layer
    //      and reduce float sum error.
    int jump = 1;
    for (int layer = 0; layer < (int)log2(samples); layer++) {
        for(int idx = 0; idx < samples; idx += jump*2) {
            for (int dist = 0; dist < 5; dist++) {
                sample_set[idx][dist] += sample_set[idx+jump][dist];
            }
        }
        jump = jump << 1;
    }
    for (int dist = 0; dist < 5; dist++) {
        TAG_DATA.node_dists[dist] = sample_set[0][dist];
    }
    // If all nodes have been verified, calculate the coordinate and move the gimbal
    COORD tag_coord;
    char fail = TAG_calcCoordinate(&tag_coord, TAG_DATA.node_dists);
    if (fail) return; // skip due to singular 4x4
    if (dest_coord != NULL) {
        printf("\r[% 7.2f, % 7.2f, % 7.2f]\n", ((float*)tag_coord)[0], ((float*)tag_coord)[1], ((float*)tag_coord)[2]);
        memcpy(dest_coord, &tag_coord, sizeof(COORD));
    } else {
        simple_Kalhman_filter(tag_coord);
        TAG_moveGimbal(hspi, &TAG_DATA.historic_coord, FALSE);
        TAG_sendDataSCN(hspi, (uint8_t*)&TAG_DATA.historic_coord, sizeof(float)*3U, SCN_UPDATE_TAGLOC);
        TAG_sendDataSCN(hspi, (uint8_t*)&tag_coord, sizeof(float)*3U, SCN_UPDATE_TAGLOC);
    }
}

#else
/* ANC_Network():
FUNCTION:
    This function checks if a new message is present from the DWM1000. If one is, then it processes
    the message and proceeds with an action based on that message 
    (respond to ranging, initialize geometry, update screen, update gimbal direction).
    The message is guaranteed to be directed at the received node due to frame filter configured in DWM_init()
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void ANC_Network(SPI_HandleTypeDef* hspi) {
    #if (CONFIG == GIM)
    static ServoState pan_servo = {90, 90, 0.5, 0};
    static ServoState tilt_servo = {90, 90, 0.5, 0};
    #endif
    if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
        DWM_IRQ_PREV++;
        if (DWM_checkFrameStatus(hspi) == HAL_OK) {
            MESSAGE msg;
            DWM_readReceive(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE);
            switch (msg) {
                case START_RANGE:
                    // set dest address to source of start_range message
                    uint16_t source_addr;
                    DWM_readReceive(hspi, (uint8_t*)&source_addr, 2U, S_ADDR_OFFSET);
                    DWM_initTransmitBuffer(hspi, (uint8_t*)&source_addr, 2U, D_ADDR_OFFSET);
                    rangingSlave_stateMachine(hspi);
                    // Optimization: Separate messages for init phase vs tag tracking to save on redundant RW?
                    break;
                #if (CONFIG != AN5)
                case TAG2ANC_INIT:
                    uint16_t dest_addr = TAG_ADDR;
                    DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
                    ANC_respondACK(hspi, TRUE);
                    ANC_initGeometry(hspi)
                    break;
                #endif
                #if (CONFIG == SCN)
                case SCN_UPDATE_ANCHORS:
                    float anchors[5U][3U];
                    DWM_readReceive(hspi, (uint8_t*)anchors, sizeof(float)*5U*3U, BASE_FRAME_SIZE + 1U);
                    DWM_enableRX(hspi);
                    char cmd = 'I'; // I for init (since this is init of anchor coords)
                    fwrite(&cmd, 1U, 1U, stdout);
                    fwrite(anchors, 4U, 5U*3U, stdout);
                    fflush(stdout);
                    break;
                case SCN_UPDATE_TAGLOC:
                    float tagloc[3U];
                    DWM_readReceive(hspi, (uint8_t*)tagloc, sizeof(float)*3U, BASE_FRAME_SIZE + 1U);
                    DWM_enableRX(hspi);
                    char cmd = 'U'; // U for update
                    fwrite(&cmd, 1U, 1U, stdout);
                    fwrite(tagloc, 4U, 3U, stdout);
                    static uint8_t buffer_count = 0;
                    if ((buffer_count++ % 4) == 0) { // used instead of setvbuf(stdout, NULL, _IOFBF, 52)
                        fflush(stdout);              // since anchors update message is of different size
                    }
                    break;
                #endif
                #if (CONFIG == GIM)
                case MOVE_GIMBAL: case MOVE_GIMBAL_INIT:
                    COORD move_coord;
                    // move gimbal has ACK protocol since within a certain time, its better to confirm reception
                    //  then move on to the next coordinate calculation.
                    ANC_respondACK(hspi, FALSE); // RX is off at this point
                    if (msg == MOVE_GIMBAL)
                    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, (msg == MOVE_GIMBAL) ? GPIO_PIN_RESET : GPIO_PIN_SET);
                    DWM_readReceive(hspi, (uint8_t*)move_coord, sizeof(float)*3U, BASE_FRAME_SIZE + 1U);
                    point_gimbal(&pan_servo, &tilt_servo, ((float*)move_coord)[0], ((float*)move_coord)[1], ((float*)move_coord)[2]);
                    DWM_enableRX(hspi); // set receiver ON
                    break;
                #endif
            }
        } else {
            DWM_enableRX(hspi);
        }
    }
}
#endif

/* rangingMaster_stateMachine():
FUNCTION:
This function is the master state machine for a double-sided two-way ranging protocol. It initiates the ranging
transaction and monitors for errors/transmission loss. The 4 hops of the protocol include:
init ranging -> anchor ack -> master ack -> anchor sends local ranging data to master.
A TX Watchdog is used to retry ranging in the event the anchor does not respond.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
float rangingMaster_stateMachine(SPI_HandleTypeDef* hspi) {
    // fsm vars
    enum {
        START_RANGING,
        WAIT_TXPHS,
        PROCESS_TX,
        WAIT_RXDFR,
        START_RESPONSE,
        RANGING_FINISH
    } MST_STATE = START_RANGING;
    static struct { // static to limit stack allocation overhead
        TIME        TX;
        TIME        RX;
        uint32_t    T_ROUND;
        uint32_t    T_REPLY;
        uint8_t     second_range;
    } RANGE_DATA;
    MESSAGE msg;
    // RUN FSM
    DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + 1U);
    while(1) {
        switch(MST_STATE) {
        case START_RANGING:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur
            MST_STATE = WAIT_TXPHS;
            RANGE_DATA.second_range = FALSE;
            msg = START_RANGE;
            // transmit MAC frame, message data attached
            DWM_initTransmitBuffer(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE);
            SEND(hspi);
            break;
        case WAIT_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                MST_STATE = PROCESS_TX;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                MST_STATE = START_RANGING;
            }
            break;
        case PROCESS_TX:
            // record TX time/Treply2, goto next state
            MST_STATE = WAIT_RXDFR;
            RANGE_DATA.TX = DWM_getTXTimestamp(hspi);
            if (RANGE_DATA.second_range) {
                RANGE_DATA.T_REPLY = time_sub(RANGE_DATA.TX, RANGE_DATA.RX).BOT4;
            }
            break;
        case WAIT_RXDFR:
             // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                reset_TX_WATCHDOG();
                if (DWM_checkFrameStatus(hspi) != HAL_OK) {
                    MST_STATE = START_RANGING;
                } else if (RANGE_DATA.second_range) {
                    MST_STATE = RANGING_FINISH;
                } else {
                    MST_STATE = START_RESPONSE;
                }
            } else if (TX_WATCHDOG_EXP) {
                MST_STATE = START_RANGING;
            }
            break;
        case START_RESPONSE:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur. This is often not needed, but sometimes RX fails to turn-off automatically
            // respond to slave with second ranging message
            msg = MSTRESP_RANGE;
            DWM_initTransmitBuffer(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE);
            SEND(hspi);
            // acquire data from previous RX. getRX must take < ~64uS to be safe to come after SEND(). (preamble min. time)
            RANGE_DATA.RX = DWM_getRXTimestamp(hspi);
            RANGE_DATA.T_ROUND = time_sub(RANGE_DATA.RX, RANGE_DATA.TX).BOT4;
            MST_STATE = WAIT_TXPHS;
            RANGE_DATA.second_range = TRUE;
            break;
        case RANGING_FINISH:
            if (DWM_checkFrameStatus(hspi) != HAL_OK) {
              MST_STATE = START_RANGING;
            } else {
                uint8_t ranging_data[8U] __attribute__((aligned(4))); // for 2 TIME.BOT4 types; this has implied pruning of the top 8/40 bits
                DWM_readReceive(hspi, ranging_data, 8U, BASE_FRAME_SIZE);
                uint32_t Treply1 = *(uint32_t*) ranging_data;         // Reply time is usually ~340uS with 256 preamble. ~150uS with 64 preamble.
                uint32_t Tround2 = *(uint32_t*)(ranging_data + 4);

                uint64_t Tround_prod = (uint64_t)RANGE_DATA.T_ROUND * Tround2; // relies on compiler 64-bit multiplication on 32-bit architecture
                uint64_t Treply_prod = (uint64_t)RANGE_DATA.T_REPLY * Treply1; // relies on compiler 64-bit multiplication on 32-bit architecture
                uint64_t numerator   = Tround_prod - Treply_prod;
                uint32_t denominator = RANGE_DATA.T_ROUND + Tround2 + RANGE_DATA.T_REPLY + Treply1; // total transaction time is << 67ms, so 32-bits is sufficient
                
                // operation below takes ~ 6uS
                float ToF_est = (float)(numerator / denominator);
                float dist_cm = ToF_est * ADJ_SPEED_OF_LIGHT - 425.37584f; // additive is experimentally set
                return dist_cm;
            }
            break;
        }
    }
}

#if (CONFIG != TAG)
/* rangingSlave_stateMachine():
FUNCTION:
    This function is the slave state machine for a double-sided two-way ranging protocol. It responds
    to a ranging transaction and waits for master commands. The 4 hops of the protocol include:
    init ranging -> anchor ack -> master ack -> anchor sends local ranging data to master.
    A TX Watchdog is used to monitor faulty TX, in the event send fails and the FSM must be reset.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void rangingSlave_stateMachine(SPI_HandleTypeDef* hspi) {
    // fsm vars
    enum {
        START_RESPONSE,
        WAIT_TXPHS,
        PROCESS_TX,
        WAIT_RXDFR,
        RANGING_FINISH,
        FINISH_TXPHS
    } SLV_STATE = START_RESPONSE;
    static struct { // static to limit stack allocation overhead
        TIME TX;
        TIME RX;
        uint32_t T_REPLY;
    } RANGE_DATA;
    // RUN FSM
    while(1) {
        switch(SLV_STATE) {
        case START_RESPONSE:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur. This is often not needed, but sometimes RX fails to turn-off automatically
            // Check for faulty RX timestamp read. There seems to be an occasional
            //      bug where the DWM1000 doesn't report the top 3 bytes correctly
            //      for RX timestamp. While the timestamp (0xFFFFFFXXXX) is possible
            //      regularly, noticing this pattern discards the transaction under the
            //      assumption it is faulty.
            // LDEDONE, in testing, is always set by this point. RX timestamp must be valid here. SPI produces no errors when this occurs.
            // if ((RANGE_DATA.RX.TOP1 == 0xFFU) &&
            //     ((RANGE_DATA.RX.BOT4 & 0xFFFF0000) == 0xFFFF0000)) {
            //     SLV_STATE = CMD; // fixme
            //     DWM_enableRX(hspi);
            //     break;
            // } // temporarily commented out since why do we not need this in all other RX timestamp recordings? Maybe one-off DWM, so make generalized filter? Maybe macro?
            SLV_STATE = WAIT_TXPHS;
            DWM_initTransmitSize(hspi, BASE_FRAME_SIZE);
            MAX_TX_WATCHDOG();
            SEND(hspi);
            // record RX, send reply to target node. getRX must take < ~64uS to be safe to come after SEND(). (preamble min. time)
            RANGE_DATA.RX = DWM_getRXTimestamp(hspi);
            break;
        case WAIT_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                SLV_STATE = PROCESS_TX;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                DWM_enableRX(hspi);
                return;
            }
            break;
        case PROCESS_TX:
            // record TX time/Treply1, goto next state
            SLV_STATE = WAIT_RXDFR;
            RANGE_DATA.TX = DWM_getTXTimestamp(hspi);
            RANGE_DATA.T_REPLY = time_sub(RANGE_DATA.TX, RANGE_DATA.RX).BOT4;
            break;
        case WAIT_RXDFR:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                TX_WATCHDOG->ARR = TX_TIMEOUT;
                reset_TX_WATCHDOG();
                // Check frame condition
                if (DWM_checkFrameStatus(hspi) == HAL_OK) {
                    SLV_STATE = RANGING_FINISH;
                } else {
                    DWM_enableRX(hspi);
                    return;
                }
                // Verify Master is conducting second ranging
                uint8_t msg;
                DWM_readReceive(hspi, &msg, 1U, BASE_FRAME_SIZE);
                if (msg != MSTRESP_RANGE) {
                    SLV_STATE = START_RESPONSE;
                }
            } else if (TX_WATCHDOG_EXP) { // missed for ~64ms, so assume anchor/tag died and a new master will transmit
                TX_WATCHDOG->ARR = TX_TIMEOUT;
                TX_WATCHDOG->CNT = TX_TIMEOUT;
                TX_WATCHDOG->EGR = TIM_EGR_UG;
                __DSB();
                return;
            }
            break;
        case RANGING_FINISH:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur. This is often not needed, but sometimes RX fails to turn-off automatically
            SLV_STATE = FINISH_TXPHS;
            // Get RX timestamp and calculate round trip time on second ranging
            RANGE_DATA.RX = DWM_getRXTimestamp(hspi);
            uint32_t Tround2 = time_sub(RANGE_DATA.RX, RANGE_DATA.TX).BOT4;
            // package and send to master node
            uint8_t ranging_data[8U] __attribute__((aligned(4))); // for 2 TIME.BOT4 types; this has implied pruning of the top 8/40 bits
            *(uint32_t*)ranging_data       = RANGE_DATA.T_REPLY;
            *(uint32_t*)(ranging_data + 4) = Tround2;
            DWM_initTransmitBuffer(hspi, ranging_data, 8U, BASE_FRAME_SIZE);
            DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + 8U);
            SEND(hspi);
            break;
        case FINISH_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                reset_TX_WATCHDOG();
                return;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                DWM_enableRX(hspi);
                return;
            }
            break;
        }
    }
}
#endif

#if (CONFIG == TAG)
/* TAG_initGeometry():
FUNCTION:
    This function handles the initialization of the UWB network geometry by handing off master control
    to each of the anchor nodes for a temporary time to collect distance data. This data is used to make
    an initial coordinate estimate based only on the geometry of the anchors. After-the-fact, the tag
    is used to create a transformation for this predicted geometry so it matches positioning in reality.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void TAG_initGeometry(SPI_HandleTypeDef* hspi) {
    /*localize barrier*/ {
    // squared distance matrix
    float S_dist[5][5];
    S_dist[4][4] = 0.0f; // not reached in INIT for-loop
    // INIT each anchor
    for (int anchor = 0; anchor < 4; anchor++) { // anchor 5 does not need go through init phase
        uint16_t dest_addr = GIM_ADDR + anchor; // shortcut
        DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
        
        float node_dists[4];
        TAG_demandINIT(hspi, node_dists);
        printf("\r[%5.2f, %5.2f, %5.2f, %5.2f]\n", node_dists[0], node_dists[1], node_dists[2], node_dists[3]);
        // fill in row and column of anchor i square dist to anchor j
        for (int i = anchor; i < 4; i++) {
            float squared = node_dists[i] * node_dists[i];
            S_dist[anchor][i+1] = squared;
            S_dist[i+1][anchor] = squared;
        }
        S_dist[anchor][anchor] = 0.0f;
    }
    /*
        To find the local coordinates of the anchors, we need to create an M matrix based on the 
        square of the euclidean distances to minimize error in coordinate creation. Then, using the
        top 3 (R3) eigenvectors/values of the M matrix, the coordinates of the datapoints can be computed.
        The resulting coordinates are likely not the same as the original anchor points, but are 
        the same up to an orthogonal rotation/translation transformation.
        https://math.stackexchange.com/questions/156161/finding-the-coordinates-of-points-from-distance-matrix
    */
    float M[4][4]; // because the first anchor (the gimbal) is implied as the origin in this network, its row and columns can be ommitted (0 vectors)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            M[i][j] = (S_dist[0][j+1] + S_dist[i+1][0] - S_dist[i+1][j+1]) / 2.0f;
        }
    }

    // Randomly created orthonormal V matrix in python. Statically defined here in code for repeatability if issues arise.
    float V[4][3] = {{ 0.76144030,  0.13568787, 0.03982924},
                     {-0.51907907, -0.02256761, 0.61729982},
                     {-0.03411009,  0.96752281, 0.17663539},
                     { 0.38678430, -0.21208279, 0.76560728}};
    float eigenvals[3];
    find_Eigens(M[0], V[0], 4, 3, eigenvals);
    // gimbal is origin coordinate
    for (int axis = 0; axis < 3; axis++) {
        TAG_DATA.anchor_coords[0][axis] = 0.0f;
    }
    // remaining anchor coordinates are the eigenvector matrix * sqrt(diagonal eigenvalue matrix)
    for (int coord = 1; coord < 5; coord++) {
        for (int axis = 0; axis < 3; axis++) {
            TAG_DATA.anchor_coords[coord][axis] = sqrtf(eigenvals[axis])*V[coord-1][axis];
        }
    }
    /*localize barrier*/ }
    // precompute matrices to find tag coordinate in relative frame
    TAG_initPrecomputeDeps();

    // transform predicted coords to fit gimbal reference frame
    COORD init[3] = {INIT_COORD0, INIT_COORD1, INIT_COORD2};
    COORD recorded[3];
    for (int i = 0; i < 3; i++) { // move gimbal, wait till user presses button, record tag location
        TAG_moveGimbal(hspi, &init[i], TRUE);
        printf("\rwaiting on button...\n");
        detect_button_pulse(hspi);       // blocking function
        TAG_Network(hspi, &recorded[i]); // record location
    }
    TAG_initTransformAnchors(recorded);
    TAG_initPrecomputeDeps();
    // finished
}

/* TAG_demandINIT():
FUNCTION:
    This function tells each anchor node to become a master to collect initialization distance data.
    Each anchor node becomes an effective "TAG" where it pings all other anchors nodes (anchors with address above its own) 
    and collects distance data to those anchors. This is packaged into a final message and pinged to the tag node.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    node_dists: Distances from the anchor node to the rest of the anchor
*/
void TAG_demandINIT(SPI_HandleTypeDef* hspi, float* node_dists) {
    // fsm vars
    enum {
        SEND_ANC_INIT,
        WAIT_TXPHS,         // filler state
        WAIT_INIT_ACK,      // ACK from ANC
        WAIT_NODE_DATA,     // Anchor sends distance data
        SEND_ACK,
        WAIT_ACK_TXPHS,     // filler state
        WAIT_ACKFAIL        // If ACK failed reception, anchor will be pinging again. 
                            // This waits until the TAG can assume anchor has received ACK
                            // and can assume master control.
    } INIT_STATE = SEND_ANC_INIT;
    MESSAGE msg;

    // RUN fsm
    while(1) {
        switch(INIT_STATE) {
        case SEND_ANC_INIT:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur
            INIT_STATE = WAIT_TXPHS;
            // set init message
            msg = TAG2ANC_INIT;
            DWM_initTransmitBuffer(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE);
            DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + 1U);
            SEND(hspi);
            break;
        case WAIT_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                INIT_STATE = WAIT_INIT_ACK;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                INIT_STATE = SEND_ANC_INIT;
            }
            break;
        case WAIT_INIT_ACK:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                reset_TX_WATCHDOG();
                // assumed ACK due to protocol. Assumed correct sender/good frame for ACK due to frame filtering
                DWM_enableRX(hspi); // turn RX back on to listen for anchor data in the future
                INIT_STATE = WAIT_NODE_DATA;
            } else if (TX_WATCHDOG_EXP) {
                INIT_STATE = SEND_ANC_INIT;
            }
            break;
        case WAIT_NODE_DATA:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                // Check frame condition. Node data message is assumed in this state.
                if (DWM_checkFrameStatus(hspi) == HAL_OK) {
                    // read node data into float buffer
                    DWM_readReceive(hspi, (uint8_t*)node_dists, sizeof(float)*4U, BASE_FRAME_SIZE);
                    INIT_STATE = SEND_ACK;
                } // else the anchor will resend data due to lack of ACK
            }
            break;
        case SEND_ACK:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur
            INIT_STATE = WAIT_ACK_TXPHS;
            MAX_TX_WATCHDOG(); // for ample time
            SEND(hspi); // implicit ACK message
            break;
        case WAIT_ACK_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                INIT_STATE = WAIT_ACKFAIL;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                INIT_STATE = SEND_ACK;
            }
            break;
        case WAIT_ACKFAIL: // When the tag sends an ACK, we wait to listen for any further transmission. if none is heard for
                           // sufficient time, it is assumed the anchor has downgraded to a slave state and the tag can resume control
                           // of the network.
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) { // received something from anchor. This means it did not receive the ACK
                DWM_IRQ_PREV++;
                INIT_STATE = SEND_ACK;
            } else if (TX_WATCHDOG_EXP) { // successful ACK
                TX_WATCHDOG->ARR = TX_TIMEOUT; // reset to default watchdog timer
                TX_WATCHDOG->CNT = TX_TIMEOUT;
                TX_WATCHDOG->EGR = TIM_EGR_UG;
                __DSB();
                // DWM_TRXOFF(hspi); // set to idle state to TX can occur
                return;
            }
            break;
        }
    }
}

/* TAG_initPrecomputeDeps():
FUNCTION:
    To compute the tag location, multiple matrices that are only dependent on the geometry
    of the static anchor nodes are used. These are precomputed to reduce tag calculation time.
    Reference: Exact and Approximate Solutions of Source Localization Problems 
    by Amir Beck, Petre Stoica, and Jian Li [https://doi.org/10.1109/TSP.2007.909342]
    for more information on the matrices precomputed here.
*/
void TAG_initPrecomputeDeps() {
    // precompute:
    // A.T @ A:     AT_A
    // A.T:         AT
    // |||a||^2:    sq_l2_a
    // lambda lower limit: lambda_lower_bound
    
    // make squared l2-norm of anchor coordinates
    for (int coord = 0; coord < 5; coord++) {
        TAG_DATA.sql2_a[coord] = dot_product(TAG_DATA.anchor_coords[coord], 
                                             TAG_DATA.anchor_coords[coord],
                                             3);
    }
    /*localize barrier*/ {
    // Make A.T; A: anchor coord times -2, augmented with 1
    for (int coord = 0; coord < 5; coord++) {
        for (int axis = 0; axis < 3; axis++) {
            // this could be simplified to augment anchor_coords directly, but later problem
            TAG_DATA.AT[axis][coord] = (-2.0f) * TAG_DATA.anchor_coords[coord][axis];
        }
        TAG_DATA.AT[3][coord] = 1; // augment
    }
    float A[5][4];
    transpose(TAG_DATA.AT[0], A[0], 4, 5);
    // make A.T @ A matrix
    matrix_mult(TAG_DATA.AT[0], A[0], TAG_DATA.AT_A[0], 4, 4, 5);
    // save first 3 diagonal elements of A.T @ A. This will be used to reset A.T @ A in the tag calculation function
    for (int i = 0; i < 3; i++) {
        TAG_DATA.AT_A_diag[i] = TAG_DATA.AT_A[i][i];
    }
    /*localize barrier*/ }

    float ATAh_D_ATAh[4][4];
    /*localize barrier*/ {
    // calculate lower bound for binary search algorithm in MLE of tag location
    // Using Power iteration with Gram-Schmidt, we can define (A.T @ A)^(-1/2) through eigendecomposition
    float V[4][4] = {{-0.58327955,  0.46947144, -0.58487514, -0.31193367},
                     {-0.08045370,  0.77864130,  0.54669384,  0.29727222},
                     { 0.77870782,  0.40216641, -0.22738803, -0.42446555},
                     {-0.21662482, -0.10759403,  0.55438399, -0.79633888}}; // randomly defined 4x4 orthonormal matrix
    float eigenvals[4];
    find_Eigens(TAG_DATA.AT_A[0], V[0], 4, 4, eigenvals);
    // save transpose of V before applying eigenvalue diagonal matrix
    float V_T[4][4];
    transpose(V[0], V_T[0], 4, 4);
    // calculate invsq_AT_A
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            V[j][i] *= 1.0f/sqrtf(eigenvals[i]); // ^(-1/2)
        }
    }

    float invsq_AT_A[4][4];
    matrix_mult(V[0], V_T[0], invsq_AT_A[0], 4, 4, 4);
    // Reusing memory allocated to V to make (A.T @ A)^(-1/2) @ D, where D is an identity matrix with [3][3] as 0.
    memcpy(V[0], invsq_AT_A[0], sizeof(float) * 16U);
    for (int i = 0; i < 4; i++) {
        V[i][3] = 0.0f;
    }
    // compute (A.T @ A)^(-1/2) @ D @ (A.T @ A)^(-1/2)
    matrix_mult(V[0], invsq_AT_A[0], ATAh_D_ATAh[0], 4, 4, 4);
    /*localize barrier*/ }

    // We need to find the largest eigenvalue of the ATAh_D_ATAh matrix.
    // Again, we use Power iteration with Gram Schmidt
    float V[4][4] = {{-0.58327955,  0.46947144, -0.58487514, -0.31193367},
                     {-0.08045370,  0.77864130,  0.54669384,  0.29727222},
                     { 0.77870782,  0.40216641, -0.22738803, -0.42446555},
                     {-0.21662482, -0.10759403,  0.55438399, -0.79633888}}; // randomly defined 4x4 orthonormal matrix
    float eigenvals[4];
    find_Eigens(ATAh_D_ATAh[0], V[0], 4, 4, eigenvals);
    TAG_DATA.lambda_lower_bound = (-1.0f / eigenvals[0]); // eigenval[0] is guaranteed to be largest due to power iteration method
}

/* TAG_initTransformAnchors():
FUNCTION:
    - This function takes in the recorded location of tag collinear with a known vector in respect
       to the gimbal (origin) node, makes them unit-vectors for proper mapping, and solves an orthogonal 
       Procrustes equation to minimize Gaussian noise and human error impacts on transformation calculation.
       This transformation (H) is then used to adjust the predicted geometry to its correct orientation.
    
    - Procrustes aims to minimize the squared Frobenius norm of (H @ R - V), where H is the orthogonal 
       transformation matrix mapping R -> V. By definition, this is tr((H @ R - V).T @ (H @ R − V)), or 
       tr(R.T @ R) - 2*tr(V.T @ H @ R) + tr(V.T @ V).
    - Minimizing this function implies maximizing the objective function tr(V.T @ H @ R) as our unknown is H.
    - If we define M = R.T @ V, then our objective function is tr(M.T @ H).
    - Given our constraint that H must be orthogonal (to preserve the geometry of the recorded anchors after applying H),
       tr(M.T @ H) is maximized when H ~ M, which can be met with polar decomposition.
    - Since M is a real matrix, M = U @ P where U is an orthogonal matrix as U = M @ P^(-1), and P is a 
       positive-semidefinite matrix as P = (M.T @ M)^(1/2). Then, H ~ M -> H = U, or H = M @ (M.T @ M)^(-1/2)
    - By using eigendecomposition of (M.T @ M), we can easily find (M.T @ M)^(-1/2)
DEFS:
    recorded:       The three recorded coordinates of the tag (coordinates of matrix R)
*/
void TAG_initTransformAnchors(COORD* recorded) {
    // normalize the recorded vectors
    for (int coord = 0; coord < 3; coord++) {
        float l2 = norm((float*)(recorded[coord]), 3);
        for (int axis = 0; axis < 3; axis++) {
            ((float*)(recorded[coord]))[axis] /= l2;
        }
    }
    // steps taken below to solve for H via polar decomposition:
    // R.T @ V                = M
    // M.T @ M                = E @ D @ E.T
    // M @ (M.T @ M)^(-1/2)   = H
    // M @ E @ D^(-1/2) @ E.T = H

    float M[3][3];
    float V[3][3] = {INIT_COORD0,
                     INIT_COORD1,
                     INIT_COORD2}; // target coordinates (where recorded should transform to)

    float recorded_T[3][3];
    transpose((float*)recorded[0], recorded_T[0], 3, 3);
    matrix_mult(recorded_T[0], V[0], M[0], 3, 3, 3); // R.T @ V
    float A[3][3];
    float M_T[3][3];
    transpose(M[0], M_T[0], 3, 3);
    matrix_mult(M_T[0], M[0], A[0], 3, 3, 3); // M.T @ M
    
    float EV[3][3] = {{ 0.1786719 , -0.12995002, -0.97528936},
                      { 0.71611949, -0.6625734 ,  0.21947523},
                      { 0.6747216 ,  0.73763777,  0.0253235 }}; // eigenvectors (E)
    float EV_T[3][3];
    float ev[3]; // eigenvalues (D)
    find_Eigens(A[0], EV[0], 3, 3, ev); // find eigenvalues of M.T @ M
    transpose(EV[0], EV_T[0], 3, 3);
    float S_inv[3][3] = {{0}};
    for (int i = 0; i < 3; i++) {
        S_inv[i][i] = 1.0f / sqrtf(ev[i]); // diagonal matrix S that is D^(-1/2), where D is the diagonal matrix of eigenvalues 
    }
    float MEV[3][3];
    matrix_mult(M[0], EV[0], MEV[0], 3, 3, 3);    // M @ E
    float U[3][3];
    matrix_mult(MEV[0], S_inv[0], U[0], 3, 3, 3); // M @ E @ S
    float H[3][3];
    matrix_mult(U[0], EV_T[0], H[0], 3, 3, 3);    // M @ E @ S E.T = H (the transformation matrix)

    float cpy_anccoords[5][3]; // must copy as matrix_mult modifies destination matrix
    memcpy(cpy_anccoords[0], TAG_DATA.anchor_coords[0], sizeof(float)*15U);
    matrix_mult(cpy_anccoords[0], H[0], TAG_DATA.anchor_coords[0], 5, 3, 3); // relative anchors @ H = actual anchors
}

#elif (CONFIG != TAG) && (CONFIG != AN5)
/* anchor_initGeometry():
FUNCTION:
    This function assumes master control over the network and pings all other anchor nodes to collect distance
    data. 384 ranging samples are taken between nodes to average out Gaussian noise. Every 3 samples have a median
    + averaging filter applied, and these 128 intermediate results are averages with a logN merging average (used to
    reduce float error). This distance data is then communicated back to the tag node.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void ANC_initGeometry(SPI_HandleTypeDef* hspi) {
    float           node_dists[4] = {-1.0, -1.0, -1.0, -1.0};
    uint8_t  verified_nodes = SHORT_ADDR - TAG_ADDR; // AN5 doesn't need to sample
    // sample distances to other anchors
    float    sample_set[128];
    float    batch[3];
    while(verified_nodes != ALL_NODES_VERIFIED) {
        uint16_t dest_addr = GIM_ADDR + verified_nodes; // shortcut
        // initialize target node address
        DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
        for (int sample = 0; sample < (128*3); sample++) {
            // find distance
            float dist = rangingMaster_stateMachine(hspi);
            // filter noisy/faulty measurement by assuming the median as the best sample,
            //      and averaging it with the other batch samples which are within tolerance.
            batch[sample % 3] = dist;
            if ((sample % 3) == 2) { // process batch of 3 samples
                // find median of 3 samples
                uint8_t median_idx = argMedian(batch[0], batch[1], batch[2]);
                // find delta to median of the 2 other samples
                float diff_med1  = batch[(median_idx + 1) % 3] - batch[median_idx];
                float diff_med2  = batch[(median_idx + 2) % 3] - batch[median_idx];
                // compute average of batch, discarding outside of delta tolerance samples
                float batch_avg = 0;
                uint32_t div    = 1;
                if (fabsf(diff_med1) < MAX_SAMPLE_DELTA) {
                    batch_avg += diff_med1;
                    div++;
                }
                if (fabsf(diff_med2) < MAX_SAMPLE_DELTA) {
                    batch_avg += diff_med2;
                    div++;
                }
                batch_avg /= div;
                batch_avg += batch[median_idx];
                if (batch_avg > 2E4) { // if more than 200m (physically unlikely), it's likely a faulty set of samples
                    sample -= 3;
                } else {
                    // add batch average to sample set, with a 1/N applied (N being batch samples)
                    sample_set[sample/3] = batch_avg / 128;
                }
            }
        }
        // logN merge of the sample set. This is done this way to preserve exponents across each merge layer
        //      and reduce float sum error.
        int jump = 1;
        for (int layer = 0; layer < (int)log2(128); layer++) {
            for(int idx = 0; idx < 128; idx += jump*2) {
                sample_set[idx] += sample_set[idx+jump];
            }
            jump = jump << 1;
        }
        // final sample average distance value
        node_dists[verified_nodes++ - 1] = sample_set[0];
    }
    ANC_replyINITDATA(hspi, node_dists);
}

/* ANC_replyINITDATA():
FUNCTION:
    This state machine is used to send the distance data collected by an anchor node (for initialization) back
    to the tag node so it can use it for calculating the anchor coordinates. Since this anchor is in a master
    setting, it will keep repinging the tag until it confirms the data is received and control is transferred.
    A TX Watchdog is used to reping the tag if it does not respond.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    node_dists: Distances from the anchor node to the rest of the anchor
*/
void ANC_replyINITDATA(SPI_HandleTypeDef* hspi, float* node_dists) {
    // fsm vars
    uint8_t send_data_attempts = 0;
    enum {
        SEND_NODE_DISTS,
        WAIT_TXPHS,
        WAIT_ACK      // ACK
    } INIT_STATE = SEND_NODE_DISTS;
    uint16_t dest_addr = TAG_ADDR;
    DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
    DWM_initTransmitBuffer(hspi, (uint8_t*)node_dists, sizeof(float)*4U, BASE_FRAME_SIZE);
    DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + sizeof(float)*4U);
    while(1) {
        switch(INIT_STATE) {
        case SEND_NODE_DISTS:
            send_data_attempts++;
            DWM_TRXOFF(hspi); // set to idle state so TX can occur
            INIT_STATE = WAIT_TXPHS;
            HAL_Delay(2);     // safety to give TRXOFF ample time to process on-chip
            SEND(hspi);
            break;
        case WAIT_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                INIT_STATE = WAIT_ACK;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                INIT_STATE = SEND_NODE_DISTS;
            }
            break;
        case WAIT_ACK:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                reset_TX_WATCHDOG();
                // assumed ACK due to protocol. Assumed correct sender/good frame for ACK due to frame filtering
                DWM_enableRX(hspi); // turn RX back on to listen for commands
                return;
            } else if (TX_WATCHDOG_EXP) {
                INIT_STATE = SEND_NODE_DISTS;
                if (send_data_attempts == DATA_SEND_RETRIES) { // Return anchor to receiver state to not compete with future TAG transmissions
                    DWM_enableRX(hspi); // turn RX back on to listen for commands
                    return;
                }
            }
            break;
        }
    }
}
#endif

#if (CONFIG == TAG)
/* TAG_MoveGimbal():
FUNCTION:
    This is a basic state machine used by the tag to send a MOVE_GIMBAL message to the gimbal. It packages
    the tag coordinate into the RF packet and waits for the gimbal to ACK the move message
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    target:     the coordinate that you want the gimbal to point to
    INIT:       boolean whether this is a INIT move gimbal or typical (INIT turns on laser)
*/
void TAG_moveGimbal(SPI_HandleTypeDef* hspi, COORD* target, uint8_t INIT) {
    // fsm vars
    enum {
        CMD_MV_GIM,
        WAIT_TXPHS,
        WAIT_ACK
    } MOVE_GIM_STATE = CMD_MV_GIM;
    uint16_t dest_addr = GIM_ADDR;
    DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET);
    uint8_t retries = GIMBAL_MOVE_RETRIES;
    while(1) {
        switch(MOVE_GIM_STATE) {
            case CMD_MV_GIM:
                DWM_TRXOFF(hspi); // set to idle state to TX can occur
                MOVE_GIM_STATE = WAIT_TXPHS;
                // set init message
                MESSAGE msg = INIT ? MOVE_GIMBAL_INIT : MOVE_GIMBAL;
                DWM_initTransmitBuffer(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE);
                DWM_initTransmitBuffer(hspi, (uint8_t*)target, sizeof(float)*3U, BASE_FRAME_SIZE + 1U);
                DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + 1U + sizeof(float)*3U);
                SEND(hspi);
                break;
            case WAIT_TXPHS:
                // spin lock
                if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                    DWM_IRQ_PREV++;
                    MOVE_GIM_STATE = WAIT_ACK;
                } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                    MOVE_GIM_STATE = CMD_MV_GIM;
                }
                break;
            case WAIT_ACK:
                // spin lock
                if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                    DWM_IRQ_PREV++;
                    reset_TX_WATCHDOG();
                    // assumed ACK due to protocol. Assumed correct sender/good frame for ACK due to frame filtering
                    return;           
                } else if (TX_WATCHDOG_EXP) {
                    if (retries-- || INIT) { // if retry counter is non-zero or INIT phase (mandatory movement in INIT)
                        MOVE_GIM_STATE = CMD_MV_GIM;
                    }
                }
                break;
        }
    }
}

/* TAG_calcCoordinate():
FUNCTION:
    This function utilized the precomputed matrices to calculate the MLE location of the tag based 
    on the distances collected to all of the anchor nodes. It uses the SR-LS approach noted below, and
    does a binary search on the noted phi function. Since the bounds are infinite, it implements methods
    to expand the bounds as needed to always converge, and monitors for convergence issues with max iterations
    and floatf precision.
DEFS:
    tag_coord:      point to a COORD type to obtain calculation output
    dists:          vector of 5 floats for the distances to the 5 anchors from the tag
*/
char TAG_calcCoordinate(COORD* tag_coord, float* dists) {
    // All precomputed math and dependencies are built upon the works of this paper:
    //      Exact and Approximate Solutions of Source Localization Problems 
    //      by Amir Beck, Petre Stoica, and Jian Li [https://doi.org/10.1109/TSP.2007.909342]
    // Here, we use their described SR-LS approach due to its decent robustness to Gaussian noise.

    // define b matrix (dists^2 - (anchor coord)^2)
    float b_mat[5];
    for (int i = 0; i < 5; i++) {
        b_mat[i] = (dists[i]*dists[i]) - TAG_DATA.sql2_a[i];
    }
    // define A.T @ b
    float AT_b[4];
    for (int i = 0; i < 4; i++) {
        AT_b[i] = dot_product(TAG_DATA.AT[i], b_mat, 5);
    }
    float AT_b3_save = AT_b[3];

    // binary search lambda bias value that minimizes phi(lambda) for MLE
    float lambda_test = 0.0f; // In testing, most lambda values land near 0. While this can perturb
                              //    binary search effectiveness if true lambda deviates strongly from 0,
                              //    in the average case this will converge faster.
    float upper_lambda = 1000.0f;
    float lower_lambda = TAG_DATA.lambda_lower_bound;
    float upper_MAX    = upper_lambda; // The paper specifies technically an infinite upper bound.
                                       //   This allows us to modify the range of binary search
                                       //   without oscillatory behavior if the upper bound must be changed.
    uint8_t LIMIT_OF_FLOAT = FALSE;    // safety flag set if the upper and lower bounds are within 1 decimal value in the mantissa
    uint8_t iter = 0;
    float y_hat[4];
    float phi;
    char fail = TAG_calc_yhat(AT_b, y_hat, lambda_test);
        if (fail) return 1;
    do {
        iter++;
        char fail = TAG_calc_yhat(AT_b, y_hat, lambda_test);
        if (fail) return 1;
        // phi = y_hat.T @ D @ y_hat - 2f.T @ y_hat
        phi = dot_product(y_hat, y_hat, 3) - y_hat[3];
        // binary search update criteria
        if (phi > 0.0f) {
            lower_lambda = lambda_test;
            lambda_test += (upper_lambda - lambda_test) / 2;
        } else {
            upper_lambda = lambda_test;
            lambda_test -= (lambda_test - lower_lambda) / 2;
        }
        // lambda_test within 10% of the max upper limit: expand the upper bound to double the prior.
        if (lambda_test >= (upper_MAX * 0.90)) {
            upper_MAX = upper_lambda *= 2;
        }
        AT_b[3] = AT_b3_save;
        // check if binary search boundaries are within 1 float mantissa increment. ASSUMES LITTLE ENDIAN COMPILATION.
        if (fabs((*(char*)&upper_lambda & 0xFF) - (*(char*)&lower_lambda & 0xFF)) == 1) LIMIT_OF_FLOAT = TRUE;
    } while ((fabs(phi) > 5E-4) && (!LIMIT_OF_FLOAT) && (iter < MAX_ITER)); // if within 5/100th of 0.0f

    // first 3 elements of y_hat are the predicted coordinate. The 4th element is produced by the bias augmentations
    for (int axis = 0; axis < 3; axis++) {
        (*tag_coord)[axis] = y_hat[axis];
    }
    return 0;
}

/* TAG_calc_yhat():
FUNCTION:
    This function calculates the y hat (predicted coordinate) of the MLE algorithm. The linear algebra specified
    in the paper is mostly identity matrices or zero vectors, so a significant part of the computation
    has been cut down to the intrinsic operations that occur.
DEFS:
    AT_b:        computed A.T @ b matrix in TAG_calcCoordinate()
    y_hat:       pointer to output vector of 4 floats
    lambda_test: lambda bias value used to compute y hat
*/
char TAG_calc_yhat(float* AT_b, float* y_hat, float lambda_test) {
    for (int i = 0; i < 3; i++) {
        TAG_DATA.AT_A[i][i] = TAG_DATA.AT_A_diag[i] + lambda_test; // A.T @ A + lambda @ D
    }
    AT_b[3] += (lambda_test * 0.5f);   // A.T @ b - lambda @ f

    // inverse of A.T @ A + lambda @ D
    float inv_ATA_lD[4][4];
    char fail = inv_4x4(TAG_DATA.AT_A[0], inv_ATA_lD[0]);
    if (fail) return 1;

    // (A.T @ A + lambda @ D)^{-1} @ (A.T @ b - lambda @ f)
    for (int i = 0; i < 4; i++) {
        y_hat[i] = dot_product(inv_ATA_lD[i], AT_b, 4);
    }
    return 0;
}

/* TAG_sendDataSCN():
FUNCTION:
    This is a simple state machine that the tag uses to send data to the screen. Since the screen
    is not vital to the operation of the network, the tag does not wait for the screen to ACK, it assumes
    the data is received.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    data:       pointer to byte buffer for data to be transmitted
    size:       length of data buffer in bytes
    msg:        msg to be appended to the data packet for the receiver to process
*/
void TAG_sendDataSCN(SPI_HandleTypeDef* hspi, uint8_t* data, uint8_t size, MESSAGE msg) {
    enum {
        SEND_DATA,
        WAIT_TXFRS
    } SEND_STATE = SEND_DATA;
    DWM_clearIRQ(hspi); // makes sure TXFRS is clear; this is a problem only if 
                        // the chip was TX beforehand and was relying on PHS
    while(1) {
        switch(SEND_STATE) {
        case SEND_DATA:
            DWM_TRXOFF(hspi);
            SEND_STATE = WAIT_TXFRS;
            DWM_initTransmitBuffer(hspi, (uint8_t*)&msg, 1U, BASE_FRAME_SIZE); // message
            DWM_initTransmitBuffer(hspi, data, size, BASE_FRAME_SIZE + 1U);    // data payload
            uint16_t dest_addr = SCN_ADDR;
            DWM_initTransmitBuffer(hspi, (uint8_t*)&dest_addr, 2U, D_ADDR_OFFSET); // frame filter setting
            DWM_initTransmitSize(hspi, BASE_FRAME_SIZE + size + 1U);
            // Need to wait for FRS rather than PHS so future DWM_TRXOFF does not race this frame TX
            DWM_setTXPHS_or_TXFRS_IRQ(hspi, FALSE);
            SEND(hspi);
            break;
        case WAIT_TXFRS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                DWM_setTXPHS_or_TXFRS_IRQ(hspi, TRUE); // set PHS IRQ; reset FRS
                reset_TX_WATCHDOG();
                return;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                SEND_STATE = SEND_DATA;
            }
            break;
        }
    }
}

#else
/* ANC_respondACK():
FUNCTION:
    This is a basic state machine that the anchor uses to send an ACK to the master node.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    INIT:       If this ACK is during the anchor geometry INIT phase (assuming master control)
*/
void ANC_respondACK(SPI_HandleTypeDef* hspi, uint8_t INIT) {
    // fsm vars
    enum {
        SEND_ACK,
        WAIT_ACK_TXPHS,     // filler state
        WAIT_ACKFAIL        // If ACK failed reception, anchor will be pinging again. This waits until the TAG can assume anchor has received ACK
    } RESPOND_STATE = SEND_ACK;

    while(1) {
        switch(RESPOND_STATE) {
        case SEND_ACK:
            DWM_TRXOFF(hspi); // set to idle state to TX can occur
            RESPOND_STATE = WAIT_ACK_TXPHS;
            DWM_initTransmitSize(hspi, BASE_FRAME_SIZE);
            if (INIT) {
                MAX_TX_WATCHDOG(); // ample time for master handoff confirmation
            }
            SEND(hspi); // implicit ACK message
            break;
        case WAIT_ACK_TXPHS:
            // spin lock
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) {
                DWM_IRQ_PREV++;
                if (!INIT) {
                    reset_TX_WATCHDOG();
                    return;
                }
                RESPOND_STATE = WAIT_ACKFAIL;
            } else if (TX_WATCHDOG_EXP) { // This is a safety. Ideally, this would NEVER hit here
                RESPOND_STATE = SEND_ACK;
            }
            break;
        case WAIT_ACKFAIL: // This state is only accessible during the INIT phase for an anchor
            if (DWM_IRQ_SEQ != DWM_IRQ_PREV) { // implied receive of message that causes this FSM to run. Master did not receive ACK.
                DWM_IRQ_PREV++;
                reset_TX_WATCHDOG();
                RESPOND_STATE = SEND_ACK; // retry sending ACK
            } else if (TX_WATCHDOG_EXP) { // successful ACK
                TX_WATCHDOG->ARR = TX_TIMEOUT; // reset to default watchdog timer
                TX_WATCHDOG->CNT = TX_TIMEOUT;
                TX_WATCHDOG->EGR = TIM_EGR_UG; // Prevents race-condition of CEN and timer params
                __DSB();
                // DWM_TRXOFF(hspi);
                return;
            }
            break;
        }
    }
}
#endif

#if (CONFIG == TAG)
/*  poll_button():
FUNCTION:
    polls the button until a stable even is captured of either high or low
*/
uint8_t poll_button() {
    static uint8_t stable_state = 0;
    static uint8_t count = 0;
    uint8_t sample = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
    if (sample == stable_state) { count = 0; }
    else {
        if (++count >= BUTTON_SMP_THRESHOLD) {
            // handle state change
            stable_state = sample;
            count = 0;
            return (stable_state);
        }
    }
    return (0b10 | stable_state); // if > 1, unstable or no new change
}

/*  detect_button_pulse():
FUNCTION:
    returns out of the function once a low -> high -> low sequence is recorded on
    the button press.
hspi:       SPI peripheral address connected to DWM1000
*/
void detect_button_pulse(SPI_HandleTypeDef* hspi) {
    uint8_t high_edge_seen = FALSE;
    uint8_t blink_counter = 0;
    while(1) {
        if (blink_counter == 0x00) {
            DWM_writeGPIO2(hspi, 1);
        } else if (blink_counter == 0x80) {
            DWM_writeGPIO2(hspi, 0);
        }
        uint8_t polling = poll_button();
        if (polling == 1) {
            high_edge_seen = TRUE;
        } else if (high_edge_seen & (polling == 0)) {
            DWM_writeGPIO2(hspi, 0);
            return;
        }
        // else its > 1, which is an unstable state
        HAL_Delay(15);
        blink_counter += 0x10;
    }
}

/* simple_Kalhman_filter():
FUNCTION:
    This is a really basic and crude "Kalhman" Filter. It takes the current coordinate and compares it to the historic
    coordinate and velocity data recorded. By applying an RBF function to the (expected - acquired) velocity
    magnitude and direction values, a joint RBF probability weight function is created that prefers similar
    speeds and directions, therefore mitigating some noise jitters. Since the filter is not adaptive, it could easily
    get stuck or become sluggish depending on the coordinate update rate. For this, a mandatory 8% update is given.
DEFS:
    current_coord:      The currently computed coordinate
*/
void simple_Kalhman_filter(COORD current_coord) {
    COORD current_dir, cpy_historic;
    memcpy(cpy_historic, TAG_DATA.historic_coord, sizeof(float)*3U);
    scale_vector(-1.0f, cpy_historic, cpy_historic, 3U);
    // calculate the direction vector of the new coordinate relative to the historic coordinate
    add_vectors(current_dir, current_coord, cpy_historic, 3U);
    float current_dir_mag = norm(current_dir, 3U);
    if (current_dir_mag <= 1E-3) {
        current_dir_mag = 1E-3;
    }
    // unitize the direction vector
    for (int i = 0; i < 3; i++) {
        current_dir[i] /= current_dir_mag;
    }
    // baseline probability based on direction similarity of historic and current direction vectors
    float dir_prob = dot_product(current_dir, TAG_DATA.historic_dir, 3U);

    // create a direction magnitude delta
    float mag_diff = (current_dir_mag - TAG_DATA.historic_dir_mag);
    float rbf_mag  = expf(-0.2f * (mag_diff * mag_diff));           // Preference to similar velocity is given with the rbf function
    float rbf_dir  = expf(-5.0f * (dir_prob - 1) * (dir_prob - 1)); // Preference to similar velocity direction is given
    float update_weight = rbf_mag * rbf_dir * 0.6f + 0.08f;         // Max 68% update to historic data

    // update historic coordinate and direction vectors
    for (int i = 0; i < 3; i++) {
        TAG_DATA.historic_coord[i] =   update_weight  * current_coord[i] + 
                                    (1-update_weight) * TAG_DATA.historic_coord[i];
        TAG_DATA.historic_dir[i]   =   update_weight  * current_dir[i] + 
                                    (1-update_weight) * TAG_DATA.historic_dir[i];
    }
    // unitize historic direction vector
    float l2_historic_dir = norm(TAG_DATA.historic_dir, 3U);
    if (l2_historic_dir <= 1E-3) {
        l2_historic_dir = 1E-3;
    }
    for (int i = 0; i < 3; i++) {
        TAG_DATA.historic_dir[i] /= l2_historic_dir;
    }
    // update historic magnitude vector
    TAG_DATA.historic_dir_mag = update_weight * current_dir_mag + (1-update_weight)* TAG_DATA.historic_dir_mag;
}
#endif