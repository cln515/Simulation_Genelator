From ubuntu:18.04
RUN mkdir -p /home/repo/simulation-gen/src 
RUN mkdir -p /home/repo/simulation-gen/include && \ 
 mkdir -p /home/repo/simulation-gen/lib_utils && mkdir -p /home/repo/simulation-gen/spline  && mkdir -p /home/repo/simulation-gen/cmake 
COPY src /home/repo/simulation-gen/src
COPY include /home/repo/simulation-gen/include
COPY lib_utils /home/repo/simulation-gen/lib_utils
COPY spline /home/repo/simulation-gen/spline
COPY cmake /home/repo/simulation-gen/cmake
COPY CMakeLists.txt /home/repo/simulation-gen/CMakeLists.txt

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential wget git cmake nano unzip libeigen3-dev libgtk-3-dev qt5-default freeglut3-dev libvtk6-qt-dev libtbb-dev \
    libjpeg-dev libpng++-dev libtiff-dev libopenexr-dev libwebp-dev \
    libhdf5-dev libopenblas-dev liblapacke-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /
ENV OPENCV_VERSION="3.4.7"
RUN wget --no-check-certificate https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
&& unzip ${OPENCV_VERSION}.zip \
&& mkdir /opencv-${OPENCV_VERSION}/cmake_binary \
&& cd /opencv-${OPENCV_VERSION}/cmake_binary \
&& cmake -DWITH_CUDA=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DENABLE_PRECOMPILED_HEADERS=OFF \
# -DCMAKE_INSTALL_PREFIX=$(python3.7 -c "import sys; print(sys.prefix)") \
#  -DPYTHON_EXECUTABLE=$(which python3.7) \
#  -DPYTHON_INCLUDE_DIR=$(python3.7 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
#  -DPYTHON_PACKAGES_PATH=$(python3.7 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
  .. \
&& make install

RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev libcgal-dev ca-certificates libglew-dev\
    && rm -rf /var/lib/apt/lists/*

RUN cd /home/repo && git clone https://github.com/cln515/ann 

RUN mkdir -p /home/repo/ann/build && cd /home/repo/ann/build \
    && cmake .. && make -j4 && make install

RUN cd /home/repo && git clone https://github.com/nlohmann/json 
RUN mkdir -p /home/repo/json/build && cd /home/repo/json/build \
    && cmake .. && make -j4 && make install

RUN mkdir -p /home/repo/simulation-gen/build && cd /home/repo/simulation-gen/build \
    && cmake .. && make -j4
