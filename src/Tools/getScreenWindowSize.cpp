#include "Tools.h"

#if 1 

namespace PKVIO{
namespace Tools{
    
    void getScreenWindowSize(int& nWidth, int& nHeight){
        nWidth = 1920; nHeight = 1080;
    }
}
}

#else
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <math.h>
//#include <linux/ioctl.h>
#include <i386-linux-gnu/sys/ioctl.h>
//#include <sys/ioctl.h>

namespace PKVIO{
namespace Tools{
    
    void getScreenWindowSize(int& nWidth, int& nHeight){
        int w,h ,bpp;
        //int *fbmem;
        int fd;
        struct fb_var_screeninfo fb_var;
        fd = open("/dev/fb0",O_RDWR);
        ioctl (fd,FBIOGET_VSCREENINFO,&fb_var);
        w = fb_var.xres;
        h = fb_var.yres;
        bpp = fb_var.bits_per_pixel;
        // result is 0,0
        throw;
        printf ("Framebuffer %d*%d-%dbpp\n",w,h,bpp);
        //fbmem = mmap (0,w*h*bpp/8,PROT_WRITE|PROT_READ, MAP_SHARED,fd,0);
    }
}
}
#endif
