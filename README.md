# Simulation Data Renderer
Simulation for multi-sensor (LiDAR and camera) system
## Dependency
Eigen3, CGAL, json, OpenCV, ANN, Boost

## installation
After dependency installation,
```
$ git clone --recursive https://github.com/cln515/Simulation_Generator.git
$ mkdir -p Simulation_Generator/build && cd Simulation_Generator/build
$ cmake .. 
$ make -j4 
```

### Dependency installation
Please refer Dockerfile

(Tested on Windows10 (VS2017) and Ubuntu:18.04)

### Docker install
```
$ cd /path/to/simulation-generator
$ docker build -t <image>:<tag> -f docker/Dockerfile .
```
## How to use
```
$ ./Simulation_genelator /path/to/input.json
```

the json file is like this
```
{
  "input": "/home/user/data/testdata/simple.ply",
  "motion": "/home/user/data/testdata/straightpath.txt",
  "output": "/home/user/data/testdata/out_ubuntu",
  "scan_start": 0.0,
  "scan_end": 2.0,
  "LiDAR": [
    {
      "type": "VLP-16",
      "file": "point/binary",
      "x": 0,
      "y": 0,
      "z": 0,
      "rx": 0,
      "ry": 90,
      "rz": 0
    }
  ],
  "Camera": [
    {
      "type": "Fisheye",
      "file": "image/image",
      "width": 2048,
      "height": 1024,
      "x": 0,
      "y": 0,
      "z": 0,
      "rx": 0,
      "ry": 0,
      "rz": 0
    }
  ]
}
```
path file format is based-on gluLookAt function OpenGL Utility Toolkit

(time) (camera position) (look at point) (upper direction)


sample file: https://www.cvl.iis.u-tokyo.ac.jp/~ishikawa/sample.tar.gz

after extract, modify path in "input.json", and then run
```
$./Simulation_generator /path/to/input.json 
```