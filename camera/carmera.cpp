#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>  
#include "carmera.h"


void SetExposure(const char* file_video, int exposure) {
    std::cout<<exposure<<std::endl;
    int fd;
    if ((fd = open(file_video, O_RDWR)) == -1)  {
        printf("Error opening V4L interface\n");
        return ;
    }
    int ret;
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ret = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
    if (ret < 0) {
    printf("Get exposure auto Type failed\n");
    return ;
    }
    printf("Exposure Auto Type:[%d]\n", ctrl.value);

    ctrl.id = V4L2_CID_EXPOSURE_AUTO; 
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl); 
    if (ret < 0) {
        printf("Get exposure auto Type failed\n"); 
        return  ;
    }

    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ret = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
    if (ret < 0) {
        printf("Get exposure auto Type failed\n");
        return ;
    }
    printf("Exposure Auto Type:[%d]\n", ctrl.value);

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ret = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
    if (ret < 0) {
        printf("Set exposure failed (%d)\n", ret);
        return ;
    }
    printf("Get ABS EXP Success:[%d]\n", ctrl.value);
    sleep(1);
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure;
    ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
    if (ret < 0) {
        printf("Set exposure failed (%d)\n", ret);
        return ;
    }

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ret = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
    if (ret < 0) {
        printf("Set exposure failed (%d)\n", ret);
        return ;
    }
    printf("Get ABS EXP Success:[%d]\n", ctrl.value);

    close(fd);
}