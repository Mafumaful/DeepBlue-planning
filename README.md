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

right click on the docker and attach vscode

To debug the code:

```bash
catkin_make build -DCMAKE_BUILD_TYPE=Debug
