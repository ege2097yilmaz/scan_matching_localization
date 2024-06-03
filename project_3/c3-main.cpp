
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

// Define PointCloudT and PointT types
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef pcl::PointXYZ PointT;

bool refresh_view = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}


// Function to perform scan matching using NDT
Eigen::Matrix4d NDT(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose) {
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
    PointCloudT::Ptr transformSource(new PointCloudT);
    pcl::transformPointCloud(*source, *transformSource, initTransform);

    pcl::console::TicToc time;
    time.tic();
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(1e-8); 
    int iterations = 35;
    ndt.setMaximumIterations(iterations);
    ndt.setResolution(1.0);
    ndt.setStepSize(0.1);
    ndt.setInputSource(transformSource);
    ndt.setInputTarget(target);

    PointCloudT::Ptr cloud_ndt(new PointCloudT); // NDT output point cloud
    ndt.align(*cloud_ndt);
    std::cout << "NDT has converged: " << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << " time: " << time.toc() << " ms" << std::endl;

    if (ndt.hasConverged()) {
        transformation_matrix = ndt.getFinalTransformation().cast<double>();
        transformation_matrix = transformation_matrix * initTransform;
        return transformation_matrix;
    } else {
        std::cout << "WARNING: NDT did not converge" << std::endl;
    }

    return transformation_matrix;
}

Eigen::Matrix4d ICP(PointCloudT::Ptr mapCloud, PointCloudT::Ptr source, Pose startingPose) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // Setting minimum transformation difference for termination condition.
    gicp.setTransformationEpsilon(1e-8); 
    gicp.setInputTarget(mapCloud);

    pcl::console::TicToc time;
    time.tic();
    Eigen::Matrix4f init_guess = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();

    // Setting max number of registration iterations.
    int iterations = 60; 
    gicp.setMaximumIterations(iterations);
    gicp.setInputSource(source);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gicp(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*cloud_gicp, init_guess);
    std::cout << "GICP has converged: " << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << " time: " << time.toc() << " ms" << std::endl;
    Eigen::Matrix4d transformation_matrix = gicp.getFinalTransformation().cast<double>();
    return transformation_matrix;
}

int main(int argc, char *argv[]){
	// Declare the variable USE_NDT. Its default value is true.
	int USE_NDT = 1;
	// Use the ICP algorithm only if we execute the command "./cloud_loc 2"
	if(argc == 2 && strcmp(argv[1], "2") == 0) USE_NDT = 0;
	// Print the matching algorithm we will use.
	printf("\n%s\n\n", USE_NDT ? "Using Algorithm 1: Normal Distributions Transform (NDT)" : "Using Algorithm 2: Iterative Closest Point (ICP)");
	// Declare the variable number of scans.

	int n_scans = 0;

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(1.54,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
					pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		
		if(!new_scan){
			
			new_scan = true;
			// TODO: (Filter scan using voxel filter)
			
			// Declare the voxel grid, which will speed up the matching algorithms.
			pcl::VoxelGrid<PointT> vg;
			// Set the input cloud
			vg.setInputCloud(scanCloud);
			// Declare the filter resolution
			double filterRes = 0.5; //0.1; //0.5;
			// Set the leaf size of the voxel grid.
			vg.setLeafSize(filterRes, filterRes, filterRes);
			// Declare the cloud filtered.
			typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
			// Filter the cloud points.
			vg.filter(*cloudFiltered);

			// TODO: Find pose transform by using ICP or NDT matching
			//pose = ....
			
			// Find the matching transform by using either NDT or ICP, depending on the command parameter.
			Eigen::Matrix4d matching_transform = USE_NDT ? NDT(mapCloud, cloudFiltered, pose) : ICP(mapCloud, cloudFiltered, pose);
			// Get the pose based on the matching transform.
			pose = getPose(matching_transform);

			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			
			// Declare the corrected scan.
			PointCloudT::Ptr corrected_scan (new PointCloudT);
			// Transform the cloud filtered into the corrected scan.
			pcl::transformPointCloud (*cloudFiltered, *corrected_scan, matching_transform);
			//sprintf(corrected_scan_string, "corrected_scan_%d", iteration);
			//renderPointCloud(viewer, corrected_scan, corrected_scan_string, Color(0,1,1)); // render corrected scan

			viewer->removePointCloud("scan");
			// TODO: Change `scanCloud` below to your transformed scan
			
			//print_transform(matching_transform);
			// Render the corrected scan.
			renderPointCloud(viewer, corrected_scan, "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
					viewer->removeShape("eval");
				if(maxError > 1.2){
					viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
				}
				else{
					viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
				}
			}

			pclCloud.points.clear();
		}
  	}
	return 0;
}
