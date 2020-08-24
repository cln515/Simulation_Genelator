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
# install dependencies and tools
# install Eigen3
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential wget git cmake nano unzip libeigen3-dev libgtk-3-dev qt5-default freeglut3-dev libvtk6-qt-dev libtbb-dev \
    libjpeg-dev libpng++-dev libtiff-dev libopenexr-dev libwebp-dev \
    libhdf5-dev libopenblas-dev liblapacke-dev \
    && rm -rf /var/lib/apt/lists/*

# install OpenCV
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
  .. \
&& make install

# install cgal
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev libcgal-dev ca-certificates\
    && rm -rf /var/lib/apt/lists/*

# install ANN
RUN cd /home/repo && git clone https://github.com/cln515/ann 
RUN mkdir -p /home/repo/ann/build && cd /home/repo/ann/build \
    && cmake .. && make -j4 && make install

# install json
RUN cd /home/repo && git clone https://github.com/nlohmann/json 
RUN mkdir -p /home/repo/json/build && cd /home/repo/json/build \
    && cmake .. && make -j4 && make install

RUN mkdir -p /home/repo/simulation-gen/build && cd /home/repo/simulation-gen/build \
    && cmake .. && make -j4
