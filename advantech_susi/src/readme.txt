Linux SUSI driver

Howto install SUSI driver & run demo AP
1.$ tar zxf release_file.tar.gz
2.$ sudo make install    //Install will copy all libSUSI-3.02.* to /usr/lib
3.$ ./demo_xx

complie sample sources AP
1.demo_xx.c are the sample sources.
2.make sure all libSUSI-3.02.* in your sample sources folder.
3.you can refer to Makefile, then use "make" to complie it.



Driver Version Naming Rule follow GCC standard
real name:  libSUSI-3.02.so.X.Y.Z
Soname:     libSUSI-3.02.so.X
link name:  libSUSI-3.02.so

X (major ID):   this value will be increased when the org API is changed.
Y (minor ID):   this value will be increased when the new API is added.
Z (release ID): this value will be increased when the Algorithm change or bug fix, but all API is not changed.
