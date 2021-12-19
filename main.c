#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/* Check! */
#define SUBARU_YAW_RATE_SCALE		(0.005f)		/* degree/s/digit */
/* Check! */
#define SUBARU_YAW_ACC_SCALE		(0.125f)		/* degree/s/s/digit */
#define SUBARU_ACC_SCALE 			(1.0f / 256)	/* g/digit */

static int get_be16(unsigned char *d)
{
	return (int16_t)(d[1] << 8) | (d[0] << 0);
}

static uint16_t get_unsigned_be16(unsigned char *d)
{
	return (uint16_t)(d[1] << 8) | (d[0] << 0);
}


/* Subaru pn: 27542AG011, Bosch pn: 0 265 005 627 (black cap)
 * Subaru pn: 27542AG012, Bosch pn: 0 265 005 716 (white cap)
 * Sensor needs to be feed with message 075#0000 at refresh rate (15 Hz?):
 * # cangen can0 -I 075 -L 2 -D 0000 -g 67 -x */
static int subaru_yaw_decode(struct can_frame *frame)
{
	if (frame->can_id == 0x070) {
		/* 0x070 packet can be 6 or 8 bytes.
		 * 8-bytes lenght packet contains accel and yaw rate
		 * 6-bytes lenght packet contains some static data */
		if (frame->can_dlc == 8) {
			int16_t yaw_rate, yaw_acc, y_acc;
			char *d = frame->data;

			/* bytes 0 and 1 are always zero */
			yaw_rate = get_be16(d + 2);
			y_acc    = get_be16(d + 4);
			yaw_acc  = get_be16(d + 6);

			printf("Y Acc: %8.5f g, Yaw rate: %8.5f degree/s, Yaw acc: %10.5f degree/s/s\n",
				y_acc * SUBARU_ACC_SCALE,
				yaw_rate * SUBARU_YAW_RATE_SCALE,
				yaw_acc * SUBARU_ACC_SCALE);

			return 0;
		} else if (frame->can_dlc == 6) {
			/* Reply in case of 075#2000 message */
			/* some constant unknown data */
			/* 27542AG011: A8 FF 81 00 A2 94 */
			/* 27542AG012 goes crazy and continuesly transmit: A8 FF 81 00 A2 94 flooding bus */
			return 0;
		}
	} else if (frame->can_id == 0x576) {
		int16_t x[4] = {0};
		/* this message is emited by sensor with every 10th 0x070 message */
		/* unknown data */
		/* 27542AG011: 1E 76 4C 88 12 00 00 00 */
		/* 27542AG012: 94 F6 6A F7 13 00 00 00 */
		char *d = frame->data;

		/* bytes 0 and 1 are always zero */
		x[0] = get_be16(d + 0);
		x[1] = get_be16(d + 2);
		x[2] = get_be16(d + 4);
		x[2] = get_be16(d + 6);

		if (0) {
			printf("CAL?: %d, %d, %d, %d\n", x[0], x[1], x[2], x[3]);
		}

		return 0;
	}

	/* unsupported frame */
	return -EPROTONOSUPPORT;
}

/* Check! */
#define NISSAN_YAW_RATE_SCALE		(0.005f)		/* degree/s/digit - from Bosch datasheet */
/* Check! */
#define NISSAN_YAW_ACC_SCALE		(0.125f)		/* degree/s/s/digit - from Bosch datasheet */
#define NISSAN_ACC_SCALE 			(0.0001274f)	/* g/digit - from Bosch datasheet */

/* Infinity/Nissan pn: 47931JK00A, Bosch: 0 265 005 693
 * No need to be ffed with Sync message, starts spreaming after power on
 * rate is about 20mS => 50 Hz */
static int infinity_yaw_decode(struct can_frame *frame)
{
	if ((frame->can_id == 0x130) && (frame->can_dlc == 8)) {
		int y_acc;
		int yaw_rate;
		char *d = frame->data;

		yaw_rate = get_be16(d + 0);
		y_acc = 0x8000 - get_unsigned_be16(d + 4);
		/* byte 6 - low 4 bits - counter */
		/* byte 7 - look like crc */

		if (1) {
			int x[4];

			x[0] = 0x8000 - get_unsigned_be16(d + 0);
			x[1] = 0x8000 - get_unsigned_be16(d + 2);
			x[2] = 0x8000 - get_unsigned_be16(d + 4);
			//x[3] = 0x8000 - get_unsigned_be16(d + 6);
			printf("RAW: %5d, %5d, %5d\n",
				x[0], x[1], x[2]);
		}

		printf("Y Acc: %f g, Yaw rate: %f degree/s, Yaw acc: %f degree/s/s\n",
			y_acc * NISSAN_ACC_SCALE,
			yaw_rate * NISSAN_YAW_RATE_SCALE,
			0 * NISSAN_ACC_SCALE);

		return 0;
	}
	if ((frame->can_id == 0x140) && (frame->can_dlc == 8)) {
		char *d = frame->data;

		/* byte 6 - low 4 bits - counter */
		/* byte 7 - look like crc */
		if (1) {
			int x[4];

			x[0] = 0x8000 - get_unsigned_be16(d + 0);
			x[1] = 0x8000 - get_unsigned_be16(d + 2);
			x[2] = 0x8000 - get_unsigned_be16(d + 4);

			printf("RAW: %5d, %5d, %5d\n",
				x[0], x[1], x[2]);
		}

/*
		printf("Y Acc: %f g, Yaw rate: %f degree/s, Yaw acc: %f degree/s/s\n",
			acc * SUBARU_ACC_SCALE,
			yaw_rate * SUBARU_YAW_RATE_SCALE,
			yaw_acc * SUBARU_ACC_SCALE);
*/

		return 0;
	}

	/* unsupported frame */
	return -EPROTONOSUPPORT;
}

#define MERCEDES_YAW_RATE_SCALE		(0.005f)	/* degree/s/digit - from Bosch datasheet */
#define MERCEDES_ACC_SCALE 			(0.0001274f) /* g/digit - from Bosch datasheet */
/* Mercedes pn: A 006 542 26 18
 * No need to be feed with Sync message, starts spreaming after power on
 * rate is about 20mS => 50 Hz */
static int mercedes_yaw_decode(struct can_frame *frame)
{
	if ((frame->can_id == 0x150) && (frame->can_dlc == 8)) {
		int y_acc;
		int yaw_rate;
		char *d = frame->data;

		yaw_rate = 0x8000 - get_unsigned_be16(d + 0);
		/* bytes 2..3: 0C 9D */
		y_acc = 0x8000 - get_unsigned_be16(d + 4);
		/* byte 6[3..0] - low 4 bits - counter */
		/* byte 7 - look like crc */

		if (0) {
			int x[4];

			x[0] = 0x8000 - get_unsigned_be16(d + 0);
			x[1] = 0x8000 - get_unsigned_be16(d + 2);
			x[2] = 0x8000 - get_unsigned_be16(d + 4);
			//x[3] = 0x8000 - get_unsigned_be16(d + 6);
			printf("RAW: %5d, %5d, %5d\n",
				x[0], x[1], x[2]);
		}

		printf("Y Acc: %f g, yaw: %f\n",
			y_acc * MERCEDES_ACC_SCALE,
			yaw_rate * MERCEDES_YAW_RATE_SCALE);

		return 0;
	}
	if ((frame->can_id == 0x151) && (frame->can_dlc == 8)) {
		int x_acc;
		int yaw_rate;
		char *d = frame->data;

		yaw_rate = 0x8000 - get_unsigned_be16(d + 0);
		/* bytes 2..3: 00 00 */
		x_acc = 0x8000 - get_unsigned_be16(d + 4);
		/* byte 6[3..0] - low 4 bits - counter */
		/* byte 7 - look like crc */
		if (0) {
			int x[4];

			x[0] = 0x8000 - get_unsigned_be16(d + 0);
			x[1] = 0x8000 - get_unsigned_be16(d + 2);
			x[2] = 0x8000 - get_unsigned_be16(d + 4);

			printf("RAW: %5d, %5d, %5d\n",
				x[0], x[1], x[2]);
		}


		printf("X Acc: %f g, yaw: %f\n",
			x_acc * MERCEDES_ACC_SCALE,
			yaw_rate * MERCEDES_YAW_RATE_SCALE);
		return 0;
	}
	if ((frame->can_id == 0x156) && (frame->can_dlc == 8)) {
		char *d = frame->data;

		/* bytes 0..5,5: const data 30 00 00 00 00 00 2x */
		/* byte 6[3..0] - low 4 bits - counter */
		/* byte 7 - look like crc */

		return 0;
	}

	/* unsupported frame */
	return -EPROTONOSUPPORT;
}

int main(int argc, char **argv)
{
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	printf("CAN Yaw Sensors Demo\r\n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

	do {
		int ret;
		nbytes = read(s, &frame, sizeof(struct can_frame));

		if (nbytes < 0) {
			perror("Read");
			break;
		}

		/* RAW hex */
		if (0) {
			int i;

			printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
			for (i = 0; i < frame.can_dlc; i++)
				printf("%02X ",frame.data[i]);
			printf("\r\n");
		}

		if ((frame.can_id == 0x070) || (frame.can_id == 0x576)) {
			ret = subaru_yaw_decode(&frame);
		}

		if ((frame.can_id == 0x130) || (frame.can_id == 0x140)) {
			ret = infinity_yaw_decode(&frame);
		}

		if ((frame.can_id == 0x150) || (frame.can_id == 0x151) || (frame.can_id == 0x156)) {
			ret = mercedes_yaw_decode(&frame);
		}
	} while (nbytes >= 0);

	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

	return 0;
}
