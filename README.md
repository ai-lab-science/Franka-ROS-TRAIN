# Installation instructions:
Based on [frankaemika](https://frankaemika.github.io/docs/installation_linux.html)

## Prerequesits:

- [Ubuntu 18.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Installing from the ROS repositories:

```
sudo apt install ros-melodic-libfranka ros-melodic-franka-ros
```

## Building from source

1. Uninstall existing versions of libfranka and franka_ros:
```
sudo apt remove "*libfranka*"
```
2. Build libfranka:
   1. Install dependencies:
    ```
    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
    ```
   2. Download source code by cloning from [Github](https://github.com/frankaemika/libfranka):
    ```
    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka
    ```
   3. We used version 0.8.0 of libfranka but we recommend to use the latest release. If you encounter problems and want to build another version, check out the corresponding Git tag:
    ```
    git checkout <version>
    git submodule update
    ```
   4. In the source directory, create a build directory and run CMake:
    ```
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .
    ```
3. Build the ROS packages:
   1. Create catkin workspace in a directory of your choice:
    ```
    cd /path/to/desired/folder
    mkdir -p catkin_ws/src
    cd catkin_ws
    source /opt/ros/melodic/setup.sh
    catkin_init_workspace src
    ```
   2. Clone "franka_ros" from the melodic-devel branch from [Github](https://github.com/frankaemika/franka_ros/tree/melodic-devel)
    ```
    git clone --recursive https://github.com/frankaemika/franka_ros/tree/melodic-devel src/franka_ros
    ```
   3. Again we used version 0.7.1 but recommend using the latest release version if possible. Elsewhere you can checkout the corresponding Git tag of your desired version:
    ```
    git checkout <version>
    ```
   4. Install missing dependencies and build the packages:
    ```
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh
    ```
## Setting up the real-time kernel

1. Install necessary dependencies:
    ```
    apt-get install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex
    ```
2. Find out currently used kernel version:
    ```
    uname -r
    ```
3. Download the source files and exchange the version numbers with your respective numbers (here e.g. 4.14.12):
   ```
    curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
    curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign
   ```
4. Decompress them:
    ```
    xz -d linux-4.14.12.tar.xz
    xz -d patch-4.14.12-rt10.patch.xz
    ```
5. Verify the file integrity:
   1. Verify the .tar archives:
    ```
    gpg2 --verify linux-4.14.12.tar.sign
    ```
   2. If the output is similar to:
    ```
    $ gpg2 --verify linux-4.14.12.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Can't check signature: No public key
    ```
    then you have to first download the public key of the person who signed the above file. As you can see from the above output, it has the ID 6092693E. You can obtain it from the key server:
    ```
    gpg2  --keyserver hkp://keys.gnupg.net --recv-keys 0x6092693E
    ```
    and similarly for the patch:
    ```
    gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x2872E4CC
    ```
    Also check for your ID since other kernel versions have different IDs and adapt accordingly in the above commands
   3. You can try verifying again:
    ```
    gpg2 --verify linux-4.14.12.tar.sign
    ```
    and a correct output should look similar to:
    ```
    $ gpg2 --verify linux-4.14.12.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
    gpg: WARNING: This key is not certified with a trusted signature!
    gpg:          There is no indication that the signature belongs to the owner.
    Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E
    ```
   4. Also verify the patch:
    ```
    gpg2 --verify patch-4.14.12-rt10.patch.sign
    ```
2. Compile the kernel:
   1. Extract source code and apply patch:
    ```
    tar xf linux-4.14.12.tar
    cd linux-4.14.12
    patch -p1 < ../patch-4.14.12-rt10.patch
    ```
   2. Configure kernel:
    ```
    make oldconfig
    ```
   3. This opens a text-based configuration menu. When asked for the Preemption Model, choose the Fully Preemptible Kernel:
    ```
    Preemption Model
       1. No Forced Preemption (Server) (PREEMPT_NONE)
       2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
       3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
       4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
       > 5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
    ```
   4. It is recommended to leave the other options at their default. Now compile the kernel, set the multithreading option -j to the number of your CPU cores:
    ```
    fakeroot make -j4 deb-pkg
    ```
   5. Finally install the newly created packages:
    ```
    sudo dpkg -i ../linux-headers-4.14.12-rt10_*.deb ../linux-image-4.14.12-rt10_*.deb
    ```
3. Allow a user to set real-time permissions for its processes:
```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```
4. Add limitations to the realtime group in /etc/security/limits.conf:
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```
5. To apply them log out and in again.