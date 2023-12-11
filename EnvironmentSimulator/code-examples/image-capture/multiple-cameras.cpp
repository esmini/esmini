#include "stdio.h"
#include "esminiLib.hpp"
#include <string>

int SaveImage(int img_index, int cam_index)
{
    static SE_Image img;
    std::string     filename = "snap_" + std::to_string(img_index) + "_" + std::to_string(cam_index) + ".tga";

    if (SE_FetchImage(&img) != 0)  // one frame delay
    {
        printf("Failed fetch image\n");
        return -1;
    }

    SE_WriteTGAImage(filename.c_str(), img.width, img.height, img.data, img.pixelSize, img.pixelFormat, true);

    return 0;
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SE_SaveImagesToRAM(true);
    if (SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0) != 0)
    {
        printf("Failed to initialize scenario\n");
        return -1;
    }

    int first_cam_index = SE_AddCustomCamera(3.0, 0.0, 0.8, 0.0, 0.0);
    SE_AddCustomCamera(-2.0, 0.0, 0.4, 3.14159, 0.0);
    SE_AddCustomCamera(1.0, 1.0, 0.5, 1.5708, 0.0);

    for (int i = 0; i < 5; i++)  // simulation steps
    {
        for (int j = 0; j < 3; j++)  // camera views
        {
            SE_SetCameraMode(first_cam_index + j);
            SE_StepDT(0.0f);  // render scene at same timestep but from another camera
            SaveImage(i, j);
        }

        SE_StepDT(0.1f);  // step simulation
    }

    SE_Close();

    return 0;
}
