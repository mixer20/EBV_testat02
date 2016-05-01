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
//#include <stdbool.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define PI 3.14159

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

const char bin0[] =   "0 deg";
const char bin45[] =  "45 deg";
const char bin90[] =  "90 deg";
const char bin135[] = "135 deg";
double AngleThreshold[] = {0, 22.5, 67.5, 112.5, 157.5, 180};
uint16 AngleBins[sizeof(AngleThreshold)/sizeof(double)-2];
const char* AngleBinNames[] = {bin0, bin45, bin90, bin135};


int TextColor;

float bgrImg[IMG_SIZE];

const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

void ChangeDetection();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void DetectAngle();
void BinAngle(double angle);
uint8 getMaxBin();
void clearAngleBins();

void ResetProcess() {

}

void ProcessFrame() {
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//SetBackground();

	} else {

		ChangeDetection();

		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);

		DetectRegions();

		DrawBoundingBoxes();

		DetectAngle();
	}
}

void ChangeDetection() {
	int r, c; //set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE); //loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) { //loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];

			/* implement Sobel filter in x-direction */
			int32 dx = -(int32) *(p - nc - 1) + (int32) *(p - nc + 1)
					- 2 * (int32) *(p - 1) + 2 * (int32) *(p + 1)
					- (int32) *(p + nc - 1) + (int32) *(p + nc + 1);

			int32 dy = (int32) *(p - nc - 1) + 2 * (int32) *(p - nc)
					+ (int32) *(p - nc + 1) - (int32) *(p + nc - 1)
					+ -2 * (int32) *(p + nc) + -(int32) *(p + nc + 1);

			/* check if norm is larger than threshold */
			int32 df2 = dx * dx + dy * dy;
			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;
			if (df2 > thr2) { //avoid square root
				//set pixel value to 255 in THRESHOLD image for gui
				data.u8TempImage[THRESHOLD][r + c] = 255;
			}
			//store derivatives (int16 is enough)
			imgDx[r + c] = (int16) dx;
			imgDy[r + c] = (int16) dy;

			//possibility to visualize data
			data.u8TempImage[BACKGROUND][r + c] = (uint8) MAX(0,
					MIN(255, 128+dx));
		}
	}
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

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, GREEN);
		}
	}
}

void DetectAngle() {
	uint16 o;
	int c;
	uint8 maxIdx;
	//loop over objects
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			//get pointer to root run of current object
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
			//loop over runs of current object
			do {
				//loop over pixel of current run
				for (c = currentRun->startColumn; c <= currentRun->endColumn; c++) {
					int r = currentRun->row;
					//processing for individual pixel at row r and column c
					double angle = atan2(imgDy[r * nc + c], imgDx[r * nc + c]);
					BinAngle(angle);
				}
				currentRun = currentRun->next; //get net run of current object
			} while (currentRun != NULL); //end of current object

			maxIdx = getMaxBin();
			DrawString(ImgRegions.objects[o].centroidX,
					ImgRegions.objects[o].centroidY,
					sizeof(AngleBinNames[maxIdx]),
					LARGE,
					BLUE,
					AngleBinNames[maxIdx]);
			clearAngleBins();
		}
	}
}

void BinAngle(double angle) {
	double angleDeg;
	angleDeg = angle*180/PI;
	if( angleDeg < AngleThreshold[0]) {
		angleDeg += 180;
	}
	if ( angleDeg < AngleThreshold[1]) {
		AngleBins[0]++;
	} else if ( angleDeg < AngleThreshold[2]) {
		AngleBins[1]++;
	} else if ( angleDeg < AngleThreshold[3]) {
		AngleBins[2]++;
	} else if ( angleDeg < AngleThreshold[4]) {
		AngleBins[3]++;
	} else if ( angleDeg <= AngleThreshold[5]) {
		AngleBins[0]++;
	}
}

uint8 getMaxBin() {
	printf("0: %i\n", AngleBins[0]);
	double max=0;
	uint8 i, idxMax=0;
	max=AngleBins[idxMax];
	for(i=1; i<sizeof(AngleBins)/sizeof(uint16); i++) {
		if(AngleBins[i] > max) {
			max = AngleBins[i];
			idxMax = i;
		}
	}
	return idxMax;
}

void clearAngleBins() {
	int i;
	for(i=0; i<sizeof(AngleBins)/sizeof(uint16); i++) {
		AngleBins[i] = 0;
	}
}

