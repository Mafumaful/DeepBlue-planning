# configure-docker

Start the container with the following commands

```bash
xhost +local:docker

docker compose build
docker compose up -d
```

To stop the container

```bash
docker compose down
```

To access the container

right click on the docker and attach vscode, after build the container, please reattach the vscode again to make sure that you can debug the code.

```bash

To debug the code:

```bash
catkin_make build -DCMAKE_BUILD_TYPE=Debug
```

and make sure the optimization is turned off in the CMakeLists.txt file, for example:

```bash
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") -> set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O0 -Wall")
```
