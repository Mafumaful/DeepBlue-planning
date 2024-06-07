# Use the official Ubuntu 20.04 as a base image
FROM ubuntu:20.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && \
    apt-get install -y lsb-release gnupg2 curl

# Set up ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && \
    apt-get install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Install additional packages for RViz and Gazebo
RUN apt-get install -y ros-noetic-rviz ros-noetic-gazebo-ros

# Install X server packages
RUN apt-get install -y x11-apps mesa-utils zsh git vim

# Install oh-my-zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" || true

# Set up oh-my-zsh for root user
RUN git clone https://github.com/ohmyzsh/ohmyzsh.git /root/.oh-my-zsh && \
    cp /root/.oh-my-zsh/templates/zshrc.zsh-template /root/.zshrc 

#################### Project configuration ####################

# install eigen, pcl, navigation ros-noetic-rviz-visual-tools ros-noetic-rviz-plugin-tutorials
RUN apt-get install -y libeigen3-dev libpcl-dev ros-noetic-navigation ros-noetic-rviz-visual-tools ros-noetic-rviz-plugin-tutorials

################# End of project configuration #################

# Set zsh as the default shell
RUN chsh -s /bin/zsh

# Set up environment
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN /bin/zsh -c "source ~/.zshrc"

# Create a working directory
WORKDIR /root

# Set entrypoint
ENTRYPOINT ["/bin/zsh"]
