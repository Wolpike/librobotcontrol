/*******************************************************************************
* rc_test_encoders.c
*
* Prints out current encoder ticks for all 4 channels
* channels 1-3 are counted using eQEP 0-2. Channel 4 is counted by PRU0
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"


#ifdef USE_RCINPRU0


static uint64_t timespec_diff_us(struct timespec *s, struct timespec *e)
{
  return (uint64_t)e->tv_sec * 1000000ULL + (uint64_t)e->tv_nsec / 1000ULL -
         (uint64_t)s->tv_sec * 1000000ULL - (uint64_t)s->tv_nsec / 1000ULL;
}

int main(){
	struct timespec prev = { 0, 0 };
	struct timespec now;

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	while(rc_get_state() != EXITING){
		rc_rcin_sbus_update();

		/* Print the SBUS channels each 100ms */
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (timespec_diff_us(&prev, &now) > 100000)
		{
			uint8_t nchannels = rc_sbus_decoder_get_num_channels();
			uint8_t i;

			prev = now;

			if (nchannels == 0)
				continue;

			for (i = 0; i < nchannels; i++)
				printf("%.4d ", rc_sbus_decoder_get_channel_pwm(i));
			printf("\n");
		}
	}

	rc_cleanup();
	return 0;
}


#else /* USE_RCINPRU0 */


int main(){
	int i;

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	printf("\nRaw encoder positions\n");
	printf("   E1   |");
	printf("   E2   |");
	printf("   E3   |");
	printf("   E4   |");
	printf(" \n");

	while(rc_get_state() != EXITING){
		printf("\r");
		for(i=1;i<=4;i++){
			printf("%6d  |", rc_get_encoder_pos(i));
		}
		fflush(stdout);
		rc_usleep(50000);
	}

	rc_cleanup();
	return 0;
}


#endif /* USE_RCINPRU0 */
