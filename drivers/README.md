# Studica Device VMX drivers
## Build and install
* Once in the drivers folder of ROS2:
``` bash
make
make install
make clean
```

## Dependencies
* Run imu example to ensure all dependencies are installed
```bash
cd examples/imu_example
make
sudo su
./imu_example
```

## Compile and run examples
``` bash
# build all examples
cd examples
make
# clean
make clean
```
``` bash
cd example_example
# build single example
make
# run example
./example_example
```
