libroad
=======

Get TVMET [here](https://sourceforge.net/projects/tvmet/files/Tar.Gz_Bz2%20Archive/1.7.2/)

It's basically header-only, so there isn't much to build. Still, managing dependencies is easier if you just install it into the system locations.

Install these:

```
$ sudo apt install libboost-dev \
                 libboost-regex-dev \
                 libboost-system-dev \
                 libboost-thread-dev \
                 libboost-filesystem-dev \
		 libboost-iostreams-dev \
                 libtool \
		 autoconf \
		 automake \
		 libxml++2.6-dev \
		 graphicsmagick-libmagick-dev-compat \
		 libfltk1.3-dev \
		 libglew-dev \
		 freeglut3-dev \ 
		 libfltk-gl1.3-dev
```

Configure and build:
```
$ autoreconf -i
$ CXXFLAGS="-std=c++11" ./configure
$ make -j 8
```
