/******************************************************************************
* rc_pru.c
* functions in the roboticscape library that use the pru are kept here for
* organizational purposes.
******************************************************************************/

#include "../roboticscape.h"
#include "../rc_defs.h"
#include "rc_pru.h"
#include <stdio.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <sys/mman.h>	// mmap
#include <string.h>

#define PRU_UNBIND_PATH "/sys/bus/platform/drivers/pru-rproc/unbind"
#define PRU_BIND_PATH "/sys/bus/platform/drivers/pru-rproc/bind"
#define PRU0_NAME "4a334000.pru0"
#define PRU1_NAME "4a338000.pru1"
#define PRU_NAME_LEN 13
#define PRU0_UEVENT "/sys/bus/platform/drivers/pru-rproc/4a334000.pru0/uevent"
#define PRU1_UEVENT "/sys/bus/platform/drivers/pru-rproc/4a338000.pru1/uevent"

#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU_SHAREDMEM	0x10000			// Offset to shared memory
#define CNT_OFFSET 		64

static unsigned int *prusharedMem_32int_ptr;

#ifdef USE_RCINPRU0
static void rc_rcin_init(uint8_t *pru_mem_start);
#endif

/*******************************************************************************
* int initialize_pru()
* 
* Enables the PRU and gets a pointer to the PRU shared memory which is used by 
* the servo and encoder functions in this C file.
*******************************************************************************/
int initialize_pru(){
	int  bind_fd;
	unsigned int	*pru;		// Points to start of PRU memory.
	int	fd;
	
	// reset memory pointer to NULL so if init fails it doesn't point somewhere bad
	prusharedMem_32int_ptr = NULL;

	// open file descriptors for pru rproc driver
	bind_fd = open(PRU_BIND_PATH, O_WRONLY);
	if(bind_fd == -1){
		printf("ERROR: pru-rproc driver missing\n");
		return -1;
	}

	// if pru0 is not loaded, load it
	if(access(PRU0_UEVENT, F_OK)!=0){
		if(write(bind_fd, PRU0_NAME, PRU_NAME_LEN)<0){
			printf("ERROR: pru0 bind failed\n");
			return -1;
		}
	}
	// if pru1 is not loaded, load it
	if(access(PRU1_UEVENT, F_OK)!=0){
		if(write(bind_fd, PRU1_NAME, PRU_NAME_LEN)<0){
			printf("ERROR: pru1 bind failed\n");
			return -1;
		}
	}

	close(bind_fd);

	// start mmaping shared memory
	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		printf ("ERROR: could not open /dev/mem.\n\n");
		return 1;
	}
	#ifdef DEBUG
	printf("mmap'ing PRU shared memory\n");
	#endif
	pru = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED) {
		printf ("ERROR: could not map memory.\n\n");
		return 1;
	}
	close(fd);

	// set global shared memory pointer
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;	// Points to start of shared memory

#ifdef USE_RCINPRU0
        rc_rcin_init((uint8_t *)pru);
#endif

	// zero out the 8 servo channels and encoder channel
	#ifdef DEBUG
	printf("zeroing out PRU shared memory\n");
	#endif
	memset(prusharedMem_32int_ptr, 0, 9*4);

	// zero out 4th encoder, eQEP encoders are already zero'd previously
	rc_set_encoder_pos(4,0);

	return 0;
}

/*******************************************************************************
* int restart
* Enables the PRU and gets a pointer to the PRU shared memory which is used by 
* the servo and encoder functions in this C file.
*******************************************************************************/
int restart_pru(){
	int unbind_fd, bind_fd;

	// open file descriptors for pru rproc driver
	unbind_fd = open(PRU_UNBIND_PATH, O_WRONLY);
	if(unbind_fd == -1){
		printf("open unbind fail\n");
		return -1;
	}
	bind_fd = open(PRU_BIND_PATH, O_WRONLY);
	if(bind_fd == -1){
		printf("open bind fail\n");
		return -1;
	}


	// if pru0 is loaded, unload it
	if(access(PRU0_UEVENT, F_OK)==0){
		//printf("unbinding pru0\n");
		if(write(unbind_fd, PRU0_NAME, PRU_NAME_LEN)<0){
			printf("ERROR: pru0 unbind failed\n");
			return -1;
		}
	}
	// if pru1 is loaded, unload it
	if(access(PRU1_UEVENT, F_OK)==0){
		//printf("unbinding pru1\n");
		if(write(unbind_fd, PRU1_NAME, PRU_NAME_LEN)<0){
			printf("ERROR: pru1 unbind failed\n");
			return -1;
		}
	}

	// now bind both
	if(write(bind_fd, PRU0_NAME, PRU_NAME_LEN)<0){
		printf("ERROR: pru0 bind failed\n");
		return -1;
	}
	if(write(bind_fd, PRU1_NAME, PRU_NAME_LEN)<0){
		printf("ERROR: pru1 bind failed\n");
		return -1;
	}

	close(unbind_fd);
	close(bind_fd);
	return 0;
}


#ifdef USE_RCINPRU0

int get_pru_encoder_pos()
{
	return -1;
}

int set_pru_encoder_pos(int val)
{
	(void)val;
	return -1;
}

#else

/*******************************************************************************
* int get_pru_encoder_pos();
* 
* returns the encoder position or -1 if there was a problem.
*******************************************************************************/
int get_pru_encoder_pos(){
	if(prusharedMem_32int_ptr == NULL) return -1;
	else return (int) prusharedMem_32int_ptr[CNT_OFFSET/4];
}

/*******************************************************************************
* int set_pru_encoder_pos(int val)
* 
* Set the encoder position, return 0 on success, -1 on failure.
*******************************************************************************/
int set_pru_encoder_pos(int val){
	if(prusharedMem_32int_ptr == NULL) return -1;
	else prusharedMem_32int_ptr[CNT_OFFSET/4] = val;
	return 0;
}

#endif


/*******************************************************************************
* int rc_send_servo_pulse_us(int ch, int us)
* 
* Sends a single pulse of duration us (microseconds) to a single channel (ch)
* This must be called regularly (>40hz) to keep servo or ESC awake.
* returns -2 on fatal error (if the PRU is not set up or channel out of bounds)
* returns -1 if the pulse was not sent because a pulse is already going
* returns 0 if all went well.
*******************************************************************************/
int rc_send_servo_pulse_us(int ch, int us){
	// Sanity Checks
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -2;
	} if(prusharedMem_32int_ptr == NULL){
		printf("ERROR: PRU servo Controller not initialized\n");
		return -2;
	}

	// first check to make sure no pulse is currently being sent
	if(prusharedMem_32int_ptr[ch-1] != 0){
		printf("WARNING: Tried to start a new pulse amidst another\n");
		return -1;
	}

	// PRU runs at 200Mhz. find #loops needed
	unsigned int num_loops = ((us*200.0)/PRU_SERVO_LOOP_INSTRUCTIONS); 
	// write to PRU shared memory
	prusharedMem_32int_ptr[ch-1] = num_loops;
	return 0;
}

/*******************************************************************************
* int rc_send_servo_pulse_us_all(int us)
* 
* Sends a single pulse of duration us (microseconds) to all channels.
* This must be called regularly (>40hz) to keep servos or ESCs awake.
*******************************************************************************/
int rc_send_servo_pulse_us_all(int us){
	int i, ret_ch;
	int ret = 0;
	for(i=1;i<=SERVO_CHANNELS; i++){
		ret_ch = rc_send_servo_pulse_us(i, us);
		if(ret_ch == -2) return -2;
		else if(ret_ch == -1) ret=-1;
	}
	return ret;
}

/*******************************************************************************
* int rc_send_servo_pulse_normalized(int ch, float input)
* 
*
*******************************************************************************/
int rc_send_servo_pulse_normalized(int ch, float input){
	if(ch<1 || ch>SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input<-1.5 || input>1.5){
		printf("ERROR: normalized input must be between -1 & 1\n");
		return -1;
	}
	float micros = SERVO_MID_US + (input*(SERVO_NORMAL_RANGE/2));
	return rc_send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int rc_send_servo_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int rc_send_servo_pulse_normalized_all(float input){
	int i, ret_ch;
	int ret = 0;
	for(i=1;i<=SERVO_CHANNELS; i++){
		ret_ch = rc_send_servo_pulse_normalized(i, input);
		if(ret_ch == -2) return -2;
		else if(ret_ch == -1) ret=-1;
	}
	return ret;
}

/*******************************************************************************
* int rc_send_esc_pulse_normalized(int ch, float input)
* 
* normalized input of 0-1 corresponds to output pulse from 1000-2000 us
* input is allowed to go down to -0.1 so ESC can be armed below minimum throttle
*******************************************************************************/
int rc_send_esc_pulse_normalized(int ch, float input){
	if(ch < 1 || ch > SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input < -0.1 || input > 1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = 1000.0 + (input*1000.0);
	return rc_send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int rc_send_esc_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int rc_send_esc_pulse_normalized_all(float input){
	int i, ret_ch;
	int ret = 0;
	for(i=1;i<=SERVO_CHANNELS; i++){
		ret_ch = rc_send_esc_pulse_normalized(i, input);
		if(ret_ch == -2) return -2;
		else if(ret_ch == -1) ret=-1;
	}
	return ret;
}


/*******************************************************************************
* int rc_send_oneshot_pulse_normalized(int ch, float input)
* 
* normalized input of 0-1 corresponds to output pulse from 125-250 us
* input is allowed to go down to -0.1 so ESC can be armed below minimum throttle
*******************************************************************************/
int rc_send_oneshot_pulse_normalized(int ch, float input){
	if(ch < 1 || ch > SERVO_CHANNELS){
		printf("ERROR: Servo Channel must be between 1&%d\n", SERVO_CHANNELS);
		return -1;
	}
	if(input < -0.1 || input > 1.0){
		printf("ERROR: normalized input must be between 0 & 1\n");
		return -1;
	}
	float micros = 125.0 + (input*125.0);
	return rc_send_servo_pulse_us(ch, micros);
}

/*******************************************************************************
* int rc_send_oneshot_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int rc_send_oneshot_pulse_normalized_all(float input){
	int i, ret_ch;
	int ret = 0;
	for(i=1;i<=SERVO_CHANNELS; i++){
		ret_ch = rc_send_oneshot_pulse_normalized(i, input);
		if(ret_ch == -2) return -2;
		else if(ret_ch == -1) ret=-1;
	}
	return ret;
}

/******************************************************************************/

#ifdef USE_RCINPRU0

#define RCIN_RBUF_OFFSET    0x12000
#define RCIN_RBUF_NENTRIES  300

typedef struct rcin_ring_buffer_t
{
    uint16_t ring_head; /* Owned by ARM CPU */
    uint16_t ring_tail; /* Owned by PRU */
    struct
    {
        uint16_t  pin_value;
        uint16_t  delta_t;
    } buffer[RCIN_RBUF_NENTRIES];
} rcin_ring_buffer_t;

typedef struct rcin_sbus_decoder_t
{
    volatile rcin_ring_buffer_t  *rbuf;
    uint16_t                      s0_time;
} rcin_sbus_decoder_t;


static rcin_sbus_decoder_t g_sbus_dec =
{
    .rbuf     = NULL,
    .s0_time  = 0,
};


static void rc_rcin_init(uint8_t *pru_mem_start)
{
    g_sbus_dec.rbuf = (volatile rcin_ring_buffer_t *)((uint8_t *)pru_mem_start + RCIN_RBUF_OFFSET);
    g_sbus_dec.rbuf->ring_head = 0;

    //printf("%s %p %p\n", __func__, pru_mem_start, g_sbus_dec.rbuf);
}

static void rc_rcin_sbus_process_pulse(rcin_sbus_decoder_t *dec, uint16_t width_s0, uint16_t width_s1)
{
    printf("%u %u: %u %u\n", dec->rbuf->ring_head, dec->rbuf->ring_tail, width_s0, width_s1);
    /* TODO */
}

void rc_rcin_sbus_update(void)
{
    volatile rcin_ring_buffer_t *rbuf = g_sbus_dec.rbuf;

    //printf("%s %u %u\n", __func__, rbuf->ring_head, rbuf->ring_tail);

    while (rbuf->ring_head != rbuf->ring_tail)
    {
        if (rbuf->ring_tail >= RCIN_RBUF_NENTRIES)
        {
            /* Invalid ring_tail from PRU - ignore RC input */
            return;
        }

        if (rbuf->buffer[rbuf->ring_head].pin_value == 1)
        {
            /* Remember the time we spent in the low state */
            g_sbus_dec.s0_time = rbuf->buffer[rbuf->ring_head].delta_t;
        }
        else
        {
            /* The pulse value is the sum of the time spent in the low and high states */
            rc_rcin_sbus_process_pulse(&g_sbus_dec, g_sbus_dec.s0_time, rbuf->buffer[rbuf->ring_head].delta_t);
        }

        /* Move to the next ring buffer entry */
        rbuf->ring_head = (rbuf->ring_head + 1) % RCIN_RBUF_NENTRIES;
    }
}

#endif /* USE_RCINPRU0 */
