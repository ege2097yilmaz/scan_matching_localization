# nd0013_cd2693_Exercise_Starter_Code

## Dependencies
Before you practice the exercises in this repository, ensure that your development environment is configured with the following tools and dependencies:

- [CARLA simulator 0.9.9.4](https://github.com/carla-simulator/carla/releases/tag/0.9.9) 
- [NICE DCV Server](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html). This step will install Nvidia drivers along with CUDA libraries for the underlying Tesla T4 GPU
- C++ 
- Git
- [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
- [CMake](https://askubuntu.com/questions/161104/how-do-i-install-make) and Make
- [VSCode](https://code.visualstudio.com/download), or a similar IDE
- [Eigen Library for C++](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Point Cloud Library](https://pointclouds.org/downloads/)
- Python3 and Pip
- ROS


# Instructions

## Step 1: Fork and Clone the Repository
Fork the repository to your Github account and clone it to your local development environment using the following commands:

```bash
git clone https://github.com/udacity/nd0013_cd2693_Exercise_Starter_Code.git
cd nd0013_cd2693_Exercise_Starter_Code
```

### Lesson_7_Project_Scan_Matching_Localization

Navigate to the project starter code director. 

```bash
cd nd0013_cd2693_Exercise_Starter_Code/Lesson_7_Project_Scan_Matching_Localization/c3-project
```

Follow the instructions per the specific [README](/Lesson_7_Project_Scan_Matching_Localization/c3-project/README.md). 

# Usage

Press the blue button "Desktop". Start one terminal. Run the Carla simulator by using these Unix commands:

```
su - student # Ignore Permission Denied, if you see student@ you are good
cd /home/workspace/c3-project
./run_carla.sh
```

Start another terminal. Compile the project by using these Unix commands:

```
cd /home/workspace/c3-project
cmake .
make
```

Run the project with the NDT algorithm by using Unix command:

```
./cloud_loc
```

Or run the project with the ICP algorithm by using Unix command:

```
./cloud_loc 2
```
