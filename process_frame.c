/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;
const int Border = 1;

float bgrImg[IMG_SIZE];

const float avgFac = 0.99;
const int frgLimit = 100;


int TextColor;
void SetBackground() {
	int r, c;
	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			bgrImg[r + c] = (float) data.u8TempImage[SENSORIMG][r + c];
		}
	}
	memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);
}

void ResetProcess() {
//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
	SetBackground();
}

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}
void ChangeDetection() {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			float pImg = data.u8TempImage[SENSORIMG][r + c];
			float pBgr = bgrImg[r + c];
			float Dif = fabs(pImg - pBgr);
			//if the difference is larger than threshold value
			if (Dif > data.ipc.state.nThreshold) {
				//set pixel value to 255 in THRESHOLD
				data.u8TempImage[THRESHOLD][r + c] = 255;
				//increase foreground counter
				data.u8TempImage[INDEX1][r + c]++;
				//check whether limit is reached
				if (data.u8TempImage[INDEX1][r + c] == frgLimit) {
					//set pixel to background
					bgrImg[r + c] = (float) data.u8TempImage[SENSORIMG][r + c];
					//reset foreground counter
					data.u8TempImage[INDEX1][r + c] = 0;

				} else {
					// update background image
					bgrImg[r + c] = avgFac * bgrImg[r + c] + (1 - avgFac) * (float) data.u8TempImage[SENSORIMG][r+ c];
					// set value for display
					data.u8TempImage[BACKGROUND][r + c] = (unsigned char) bgrImg[r + c];
					//set foreground counter to zero
					data.u8TempImage[INDEX1][r+c] = 1;
				}
			}
		}
	}
}

	void ProcessFrame() {
//initialize counters
		if (data.ipc.state.nStepCounter == 1) {
			//use for initialization; only done in first step
			memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
			TextColor = CYAN;
		} else {
			//example for copying sensor image to background image
			memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG],
			IMG_SIZE);
			ChangeDetection();
			Erode_3x3(THRESHOLD, INDEX0);
			Dilate_3x3(INDEX0, THRESHOLD);

		}
	}

