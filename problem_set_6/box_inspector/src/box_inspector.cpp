//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!

  ROS_WARN("NEED TO WRITE update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();

  int num_desired_models = desired_models_wrt_world.size();
  std::vector<int> satisfied_image_ints;  // vector that keeps track of which parts in the image are perfectly placed
  std::vector<int> satisfied_order_ints;  // vector that keeps track of which parts in the ORDER are satisfied
  std::vector<int> unsat_image_ints;      // the opposite
  std::vector<int> unsat_order_ints;      // again, the opposite

  for (int i=0;i<num_parts_seen;i++) {    // fill in those vectors above
  	unsat_image_ints.push_back(i);
  }
  for (int j=0;j<num_desired_models;j++) {
  	unsat_order_ints.push_back(j);
  }

  osrf_gear::Model image_model;
  osrf_gear::Model order_model;

  for (int i=0;i<num_parts_seen;i++) {  // i is the parts seen index 
  	image_model = box_inspector_image_.models[i];
  	for (int j=0; j<num_desired_models;j++){  // j is the desired model index
      order_model = desired_models_wrt_world[j];
      if (image_model.type == order_model.type){
      	if (image_model.pose.position.x == order_model.pose.position.x &&  // I'm comparing the poses. Sorry.
      		image_model.pose.position.y == order_model.pose.position.y &&
      		image_model.pose.position.z == order_model.pose.position.z &&
      		image_model.pose.orientation.x == order_model.pose.orientation.x &&
      		image_model.pose.orientation.y == order_model.pose.orientation.y &&
      		image_model.pose.orientation.z == order_model.pose.orientation.z &&
      		image_model.pose.orientation.w == order_model.pose.orientation.w){
      		satisfied_models_wrt_world.push_back(image_model);  // MODEL SATISFIED
      		satisfied_image_ints.push_back(i);        // we know this part is satisfied, so stop looking at these parts
      		satisfied_order_ints.push_back(j);         // we know this order is satisfied, so stop looking at this order
      		for (int k=0;k<unsat_image_ints.size();k++){ // this loop deletes the index numbers corresponding to the parts in the image that aren't unsatisfied
      			if (unsat_image_ints[k] == i){  // if the unsatisfied image index equals the part seen index we satsifed
      				unsat_image_ints.erase(unsat_image_ints.begin() + k);
      			}
      		}
      		for (int k=0;k<unsat_order_ints.size();k++){	
      			if (unsat_order_ints[k] == j){  // if the unsatisfied image index equals the part seen index we satsifed
      				unsat_order_ints.erase(unsat_order_ints.begin() + k);
      			}
      		}
      	}
      }
  	}
  }
  	while (unsat_image_ints.size() > 0) {
  		int image_index = unsat_image_ints[0];       // always taking the first one. no matter what happens, this will be removed from this vector during this loop
  		image_model = box_inspector_image_.models[image_index];
  		bool matched = false;
  		for (int i=0;i<unsat_order_ints.size();i++){
  			order_model = desired_models_wrt_world[unsat_order_ints[i]];
  			if (matched == false){
  				if (image_model.type == order_model.type){
  					misplaced_models_actual_coords_wrt_world.push_back(image_model);  // MISPLACED ACTUAL
  					misplaced_models_desired_coords_wrt_world.push_back(order_model); // MISPLACED DESIRED
 					for (int k=0;k<unsat_image_ints.size();k++){ // this loop deletes the index numbers corresponding to the parts in the image that aren't unsatisfied
      					if (unsat_image_ints[k] == image_index){  // if the unsatisfied image index equals the part seen index we found
      						unsat_image_ints.erase(unsat_image_ints.begin() + k);
      					}
      				}
      				for (int k=0;k<unsat_order_ints.size();k++){	
      					if (unsat_order_ints[k] == i){  // if the unsatisfied order index equals the part seen index we found
      						unsat_order_ints.erase(unsat_order_ints.begin() + k);
      					}
      				}	
      				matched = true;
      			}
  			}
  		}
  		if (matched ==false){  // if there is STILL no match for this part, it's an orphan
  			orphan_models_wrt_world.push_back(image_model); // ORPHAN
  			for (int k=0;k<unsat_image_ints.size();k++){ // this loop deletes the index numbers corresponding to the parts in the image that aren't unsatisfied
      			if (unsat_image_ints[k] == image_index){  // if the unsatisfied image index equals the part seen index we found
      				unsat_image_ints.erase(unsat_image_ints.begin() + k);
      			}
      		}
  		}
  	}

  	// And the rest are missing...
  	for (int i;i<unsat_order_ints.size();i++){
  		order_model = desired_models_wrt_world[unsat_order_ints[i]];
  		missing_models_wrt_world.push_back(order_model);
  	}




  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
/*
  int num_desired_models = desired_models_wrt_world.size();
  for (int i=0;i<num_parts_seen;i++) {
  	for (int j=0; j<num_desired_models;j++){
  		if (box_inspector_image_.models[i].type == desired_models_wrt_world[j].type && box_inspector_image_.models[i].pose == desired_models_wrt_world[j].pose){
  			satisfied_models_wrt_world.push_back(box_inspector_image_.models[i]);
  		}
  		else if (box_inspector_image_.models[i].type == desired_models_wrt_world[j].type && box_inspector_image_.models[i].pose != desired_models_wrt_world[j].pose){
  			misplaced_models_actual_coords_wrt_world.push_back(box_inspector_image_.models[i]);
  			misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world[j]);
     	//orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
  		}
  	} 
  }
  */
}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
