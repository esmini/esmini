#include "stdio.h"
#include <string>
#include "esminiLib.hpp"


void img_callback(SE_Image* img, void* data)
{
	(void)data;

	static int counter = 0;

	if (img != nullptr && img->data != nullptr)
	{
		char filename[64];
		snprintf(filename, 64, "snap_from_cb_%03d.tga", counter++);
		SE_WriteTGAImage(filename, img->width, img->height, img->data, img->pixelSize, img->pixelFormat, true);
	}
}

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	SE_Image img;

	// Set custom window size
	SE_SetWindowPosAndSize(60, 60, 1000, 400);

	// Enable continuous save screenshot to file
	SE_Init("../resources/xosc/cut-in_simple.xosc", 0, 7, 0, 0);  // 7 = viewer + offscreen only + capture-to-file

	// First frame is always saved to RAM. Fetch it.
	// It should be identical with automatically created screenshot screen_shot_00000.tga
	if (SE_FetchImage(&img) != 0)
	{
		printf("Error 0\n");
	}
	SE_WriteTGAImage("snap0.tga", img.width, img.height, img.data, img.pixelSize, img.pixelFormat, true);

	// In order to fetch further rendered images we need to activate transfer from frame buffer during rendering
	SE_SaveImagesToRAM(true);

	SE_StepDT(0.1f);
	if (SE_FetchImage(&img) != 0)
	{
		printf("Error 1\n");
	}
	SE_WriteTGAImage("snap1.tga", img.width, img.height, img.data, img.pixelSize, img.pixelFormat, true);

	// Now when two screenshots have been saved, turn off automatic saveing screenshot to file
	SE_SaveImagesToFile(0);
	SE_StepDT(0.1f);

	// But still fetch the image via API
	if (SE_FetchImage(&img) != 0)
	{
		printf("Error 2\n");
	}
	SE_WriteTGAImage("snap2.tga", img.width, img.height, img.data, img.pixelSize, img.pixelFormat, true);

	// Run forward a few seconds and then grab another snapshot
	while (SE_GetSimulationTime() < 2.0f) SE_StepDT(0.1f);
	if (SE_FetchImage(&img) != 0)
	{
		printf("Error 3\n");
	}

	// Try another image format
	SE_WritePPMImage("snap2sec.ppm", img.width, img.height, img.data, img.pixelSize, img.pixelFormat, true);

	SE_Close();

	// Next alternative: Fetch images by callback, which potentially is more efficient since image can be
	// created and fetched while next scenario frame is already being processed
	// When image callback has been registered, images will be transfered to RAM during rendering
	// I.e. no need for SE_SaveImagesToRAM(true);
	SE_RegisterImageCallback(img_callback, 0);  // Must be before SE_Init
	SE_SetWindowPosAndSize(60, 60, 600, 300);  // Try another image size

	SE_Init("../resources/xosc/lane_change_crest.xosc", 0, 1, 0, 0);  // 1 = viewer

	// Run forward a small duration to grab some images
	while (SE_GetSimulationTime() < 2.0f) SE_StepDT(0.1f);

	SE_Close();

	return 0;
}