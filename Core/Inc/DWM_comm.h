#include "main.h"
#include "DWM_driver.h"
#pragma once

extern volatile uint8_t TX_WATCHDOG_EXP;

typedef enum {
    // ranging messages
    START_RANGE,       // Master initiates ranging with another node
    // SLV2MST_RANGE,  // Slave responds to Master in first ranging transaction (implied; deprecated)
    MSTRESP_RANGE,     // Master responds to Slave in second ranging transaction
    // SLV_RANGE_DATA  // Slave sends Master ranging data acquired (implied; deprecated)
    
    // init messages
    TAG2ANC_INIT,
    // ANC2TAG_DATA,     // implied; deprecated
    // TAG2ANC_DATA_ACK, // implied; deprecated
    
    // normal messages
    // ANC2TAG_ACK,      // implied; deprecated
    MOVE_GIMBAL,
    MOVE_GIMBAL_INIT,

    // screen update messages
    SCN_UPDATE_ANCHORS,
    SCN_UPDATE_TAGLOC
} MESSAGE;

#if (CONFIG == TAG)
#define BUTTON_SMP_THRESHOLD 5U
#define INIT_COORD0     {0.373878250553f, -0.460157846834f, -0.80527623196f} // right chair
#define INIT_COORD1     {0.373878250553f,  0.460157846834f, -0.80527623196f} // left chair
#define INIT_COORD2     {0.912568246142f, -0.052146756922f, -0.40558588717f} // center

// #define INIT_COORD0     {1.0f,  0.0f,  0.0f}
// #define INIT_COORD1     {0.0f,  1.0f,  0.0f}
// #define INIT_COORD2     {0.0f,  0.0f,  1.0f}
#endif

/* SEND():
FUNCTION:
    Send function initiates transmission and start the TX Watchdog
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
static inline void SEND(SPI_HandleTypeDef* hspi) {
    TX_WATCHDOG_EXP   = FALSE;
    DWM_transmit(hspi);
    TX_WATCHDOG->CR1 |= TIM_CR1_CEN;    // turn on watchdog timer
}

/* reset_TX_WATCHDOG():
FUNCTION:
    resets the TX Watchdog to default values. Used if the timer is stopped
    prematurely.
*/
static inline void reset_TX_WATCHDOG() {
    TX_WATCHDOG->CR1 &= ~TIM_CR1_CEN;   // clear CEN for timer
    TX_WATCHDOG->CNT  =  TX_TIMEOUT;    // reset timer counter
    TX_WATCHDOG->SR  &= ~TIM_SR_CC3IF;  // clear channel 3 capture interrupt flag (deassert pending for NVIC)
    TX_WATCHDOG->EGR  =  TIM_EGR_UG;    // Prevents race-condition of CEN and timer params
    TX_WATCHDOG_EXP   =  FALSE;
    __DSB();
}

/* MAX_TX_WATCHDOG():
FUNCTION:
    sets the maximum possible count for ARR and CNT. Useful for transfer master control.
*/
static inline void MAX_TX_WATCHDOG() {
    TX_WATCHDOG->ARR  =  0xFFFF;        // set max
    TX_WATCHDOG->CNT  =  0xFFFF;        // set max
    TX_WATCHDOG->EGR  =  TIM_EGR_UG;    // Prevents race-condition of CEN and timer params
    __DSB();
}

#if (CONFIG == TAG)
typedef struct {
    // TAG distance measurements
    uint8_t     verified_nodes;         // To process which anchors are remaining for pinging
    float       node_dists[5];          // Distance buffer of all anchors to the tag
    // precomputed dependencies
    float       anchor_coords[5][3];
    float       AT_A[4][4];             // A: anchor coord augmented with 1. This is A.T @ A
    float       AT  [4][5];             // A.T
    float       sql2_a [5];             // squared l2-norm of anchor coordinates
    float       lambda_lower_bound;     // binary search lower bound for tag location MLE equation
    float       AT_A_diag[3];           // save the state of the first 3 diagonal values in AT_A matrix

    COORD       historic_coord;
    COORD       historic_dir;
    float       historic_dir_mag;
} TAG_METADATA;

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
void TAG_Network(SPI_HandleTypeDef* hspi, COORD* dest_coord);

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
void ANC_Network(SPI_HandleTypeDef* hspi);
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
float rangingMaster_stateMachine(SPI_HandleTypeDef* hspi);

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
void rangingSlave_stateMachine(SPI_HandleTypeDef* hspi);
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
void TAG_initGeometry(SPI_HandleTypeDef* hspi);

/* TAG_demandINIT():
FUNCTION:
    This function tells each anchor node to become a master to collect initialization distance data.
    Each anchor node becomes an effective "TAG" where it pings all other anchors nodes (anchors with address above its own) 
    and collects distance data to those anchors. This is packaged into a final message and pinged to the tag node.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    node_dists: Distances from the anchor node to the rest of the anchor
*/
void TAG_demandINIT(SPI_HandleTypeDef* hspi, float* node_dists);

/* TAG_initPrecomputeDeps():
FUNCTION:
    To compute the tag location, multiple matrices that are only dependent on the geometry
    of the static anchor nodes are used. These are precomputed to reduce tag calculation time.
    Reference: Exact and Approximate Solutions of Source Localization Problems 
    by Amir Beck, Petre Stoica, and Jian Li [https://doi.org/10.1109/TSP.2007.909342]
    for more information on the matrices precomputed here.
*/
void TAG_initPrecomputeDeps();

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
    - Since M is a real matrix, M = U @ P where U is an orthogonal matrix as M @ P^(-1), and P is a 
       positive-semidefinite matrix as (M.T @ M)^(1/2). Then, H ~ M -> H = U, or H = M @ (M.T @ M)^(-1/2)
    - By using eigendecomposition of (M.T @ M), we can easily find (M.T @ M)^(-1/2)
DEFS:
    recorded:       The three recorded coordinates of the tag (coordinates of matrix R)
*/
void TAG_initTransformAnchors(COORD* recorded);

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
void ANC_initGeometry(SPI_HandleTypeDef* hspi);

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
void ANC_replyINITDATA(SPI_HandleTypeDef* hspi, float* node_dists);
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
void TAG_moveGimbal(SPI_HandleTypeDef* hspi, COORD* target, uint8_t INIT);

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
char TAG_calcCoordinate(COORD* tag_coord, float* dists);

/* TAG_calc_yhat():
FUNCTION:
    This function calculates the y hat (predicted coordinate) of the MLE algorithm. The linear algebra specified
    in the paper is mostly identity matrices or mostly zero vectors, so a significant part of the computation
    has been cut down to the intrinsic operations that occur.
DEFS:
    AT_b:        computed A.T @ b matrix in TAG_calcCoordinate()
    y_hat:       pointer to output vector of 4 floats
    lambda_test: lambda bias value used to compute y hat
*/
char TAG_calc_yhat(float* AT_b, float* y_hat, float lambda_test);

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
void TAG_sendDataSCN(SPI_HandleTypeDef* hspi, uint8_t* data, uint8_t size, MESSAGE msg_type);
#else
/* ANC_respondACK():
FUNCTION:
    This is a basic state machine that the anchor uses to send an ACK to the master node.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    INIT:       If this ACK is during the anchor geometry INIT phase (assuming master control)
*/
void ANC_respondACK(SPI_HandleTypeDef* hspi, uint8_t INIT);
#endif

#if (CONFIG == TAG)
/*  poll_button():
FUNCTION:
    polls the button until a stable even is captured of either high or low
*/
uint8_t poll_button();

/*  detect_button_pulse():
FUNCTION:
    returns out of the function once a low -> high -> low sequence is recorded on
    the button press.
hspi:       SPI peripheral address connected to DWM1000
*/
void detect_button_pulse(SPI_HandleTypeDef* hspi);

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
void simple_Kalhman_filter(COORD current_coord);
#endif