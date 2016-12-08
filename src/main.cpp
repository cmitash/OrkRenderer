//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// Copyright (c) 2013, Aldebaran Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

// ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
// ----------------------------------------------------------------------------

#define GL_GLEXT_PROTOTYPES

#include <set>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer2d.h>

#define AUGMENTED_IMAGES 1000
#define NUM_OBJECTS 15
#define OBJECTS_PER_BIN 4
#define SHELF_DEPTH 0.43
#define SHELF_FRONT 0.38

class binImage {
public:
  cv::Vec2d p_a;
  cv::Vec2d p_b;
  cv::Vec2d p_c;
  cv::Vec2d p_d;
};

class apc_object {
public:
  int index;
  cv::Mat image;
  cv::Mat mask;
  double distance;
};

class apc_object_pos {
public:
  apc_object obj;
  cv::Mat mask_on_background;
  cv::Mat final_mask;
  cv::Rect bounding_box;
};

bool operator >(const apc_object& o1, const apc_object& o2) {
  return o1.distance > o2.distance;
}

struct greater
{
    template<class T>
    bool operator()(T const &a, T const &b) const { return a > b; }
}; 

//--------------------------------------------------
//Configuration FOR APC
//--------------------------------------------------

// const char *objects_list[] = {
// //multiple models
// "expo_dry_erase_board_eraser","barkely_hide_bones","command_hooks","cool_shot_glue_sticks",
// "creativity_chenille_stems_1","creativity_chenille_stems_2","dove_beauty_bar",
// "dr_browns_bottle_brush","elmers_washable_no_run_school_glue","expo_dry_erase_board_eraser",
// "folgers_classic_roast_coffee","hanes_tube_socks","i_am_a_bunny_book","jane_eyre_dvd",
// "kleenex_tissue_box","oral_b_toothbrush_green","oral_b_toothbrush_red","laugh_out_loud_joke_book",
// "peva_shower_curtain_liner","platinum_pets_dog_bowl","rolodex_jumbo_pencil_cup",
// "scotch_bubble_mailer","safety_first_outlet_plugs","scotch_duct_tape","ticonderoga_12_pencils",
// "up_glucose_bottle","soft_white_lightbulb","staples_index_cards",

// //single model
// "crayola_24_ct","kleenex_paper_towels","dasani_water_bottle","rawlings_baseball",
// "fitness_gear_3lb_dumbbell","woods_extension_cord","easter_turtle_sippy_cup",
// "cherokee_easy_tee_shirt","fiskars_scissors_red",

// //models not good
// "cloud_b_plush_bear","clorox_utility_brush",
// "kyjen_squeakin_eggs_plush_puppies","womens_knit_gloves"
// };

// int models[] = {
//   1,4,4,3,3,4,4,4,4,4,4,4,4,4,3,3,3,4,4,4,4,4,4,4,4,4,4,
//   1,1,1,1,1,1,1,1,1,1,1,1,1
// };

// int num_real_object[] = {
//   31,0,5,0,0,0,5,31,0,0,17,0,0,0,0,0,23,0,0,0,15,23,23,0,
//   24,28,27,22,0,23,17,21,21,47,19,23,27,39,20,18
// };

// int num_object[] = {
//    5115,5115,5115,3836,3836,5115,5115,5115,5115,5115,5115,5115,5115,5115,
//    3836,3836,3836,5115,5115,5115,5115,5115,5115,5115,5115,5115,5115,
//    1278,1278,1278,1278,1278,1278,1278,1278,1278,1278,1278,1278,1278
// };

// std::string background_image = "bin_APC.png";
// size_t im_width = 1920, im_height = 1080;
// double near = 0.1, far = 1000;
// double focal_length_x = 1051.89, focal_length_y = 1060.192;

// int bbox_threshold = 1500;

//--------------------------------------------------
//Configuration FOR APC Ends
//--------------------------------------------------

//--------------------------------------------------
//Configuration FOR ICRA
//--------------------------------------------------

const char *objects_list[] = {
"crayola_24_ct", "expo_dry_erase_board_eraser", "folgers_classic_roast_coffee",
"scotch_duct_tape", "dasani_water_bottle", "jane_eyre_dvd",
"up_glucose_bottle", "laugh_out_loud_joke_book", "soft_white_lightbulb",
"kleenex_tissue_box", "ticonderoga_12_pencils", "dove_beauty_bar",
"dr_browns_bottle_brush", "elmers_washable_no_run_school_glue", "rawlings_baseball"
};

int models[] = {
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};

// Old list
// int num_real_object[] = {
//   22,17,14,35,35,5,36,32,52,23,5,14,5,37,17
// };

// New list
// int num_real_object[] = {
//   24,18,12,9,9,20,10,20,16,8,18,16,13,16,5
// };

int num_real_object[] = {
  49,49,49,49,0,0,49,49,49,49,0,49,0,49,49
};

int num_object[] = {
   1022,1022,1022,1022,1022,1022,1022,1022,1022,1022,1022,1022,1022,1022,1022
};

std::string background_image = "bin_ICRA.png";
size_t im_width = 1920, im_height = 1080;
double near = 0.1, far = 1000;
double focal_length_x = 1035.47, focal_length_y = 1043.82;

int bbox_threshold = 1500;

//--------------------------------------------------
//Configuration for ICRA ends
//--------------------------------------------------

void renderByConfig(std::string foldername, 
              std::string file_name,
              int anglex,
              int angley,
              int anglez,
              int mode,
              int pic_num) {
  Renderer3d renderer = Renderer3d(file_name);
  renderer.set_parameters(im_width, im_height, focal_length_x, focal_length_y, near, far);

  RendererIterator renderer_iterator = RendererIterator(&renderer);
  //renderer_iterator.set_view_params(longtitude, latitude, rotation);
  renderer.anglex = anglex;
  renderer.angley = angley;
  renderer.anglez = anglez;
  renderer.mode = mode;

  try {
      cv::Rect rect;
      cv::Mat image, depth, mask;

      cv::Vec3d trans, rot;
      //ofstream file;
      
      renderer_iterator.render(image, depth, mask, rect, trans, rot);
      double radius = cv::norm(trans, cv::NORM_L2);

      if (depth.empty() || image.empty() || mask.empty() ) {
        std::cout << "empty depth/image/mask generated, skip!" << std::endl;
        std::cout << file_name << ", " 
          << pic_num <<", "
          << renderer.anglex << ", " 
          << renderer.angley << ", " 
          << renderer.anglez << std::endl;
          return;
      }

      //cv::imwrite(boost::str(boost::format(foldername + "/depth_%05d.png") % (pic_num)), depth);
      cv::imwrite(boost::str(boost::format(foldername + "/image_%05d.png") % (pic_num)), image);
      cv::imwrite(boost::str(boost::format(foldername + "/mask_%05d.png") % (pic_num)), mask);
      //file.open(boost::str(boost::format(foldername + "/pose_%05d.txt") % (pic_num)));
      // std::cout << file_name << ", " 
      //     << pic_num <<", "
      //     << renderer.angle << ", " 
      //     << renderer.vec_x << ", " 
      //     << renderer.vec_y << ", "
      //     << renderer.vec_z << std::endl;
      //file.close();

    } catch (...) {
      std::cout << "Exception in renderByConfig!!!" << std::endl;
      std::cout << file_name << ", " 
          << pic_num <<", "
          << renderer.anglex << ", " 
          << renderer.angley << ", " 
          << renderer.anglez << std::endl;
    }

}

void validate() {
  std::cout << "begin validating...\n"; 
  int classid;
  int anglex;
  int angley;
  int anglez;
  int mode;

  //Create Folder
  DIR *dir;
  struct dirent *ent;
  std::string foldername = "Poses";
  if((dir = opendir(foldername.c_str())) == NULL)
    mkdir(foldername.c_str(), 0775);

  std::cout << "validation folder created!" << std::endl;

  std::string result_file_name = "Poses/result.txt";
  ifstream file(result_file_name);
  int count = 0;
  std::cout << "open result file..." << std::endl;
  if (file.is_open()) {
    while (file >> classid >> anglex >> angley >> anglez >> mode) {
      std::cout << classid << " " << anglex << " " << angley << " " << anglez << std::endl;
      std::string file_name(objects_list[classid]);
      if (models[classid] == 1) {          
        file_name += ".obj";
      } else {
        file_name += "_00000.obj";
      }
      std::cout << foldername << ", " 
      << file_name << ", " 
      << anglex << ", " 
      << angley << ", " 
      << anglez << std::endl;
      renderByConfig(foldername, file_name, anglex, angley, anglez, mode, count);
      count++;
    }
  } else {
    std::cout << "file " << result_file_name << "can't be openned!!" << std::endl;
  }

  file.close();

}

int render3d(std::string file_name, 
              std::string foldername,
              size_t width, size_t height, int initnum, int stanfordclass,
              ofstream& stanfordpose, ofstream& stanfordImage) {

  Renderer3d renderer = Renderer3d(file_name);
  renderer.set_parameters(width, height, focal_length_x, focal_length_y, near, far);

  RendererIterator renderer_iterator = RendererIterator(&renderer);

  cv::Rect rect;
  cv::Mat image, depth, mask;
  std::cout << "\n\n" << file_name << ", " << foldername << ", " << initnum << "\n\n"<< std::endl;

  int lastIndex = 0;
  for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator) {
    try {
      cv::Vec3d trans, rot;
      renderer_iterator.render(image, depth, mask, rect, trans, rot);
      double radius = cv::norm(trans, cv::NORM_L2);
      
      // Print every 100th iteration
      if (i % 100 == 0) {
              std::cout << i + initnum <<", "<<renderer_iterator.lon_ << ", " 
          << renderer_iterator.lat_ << ", " 
          << renderer_iterator.rot_ << ", "
          << renderer_iterator.rad_ << std::endl;
      }

      lastIndex = i + initnum;
      
      cv::imwrite(boost::str(boost::format(foldername + "/image_%05d.png") % (i+initnum)), image);
      cv::imwrite(boost::str(boost::format(foldername + "/mask_%05d.png") % (i+initnum)), mask);

      // Store the depth image
      // cv::imwrite(boost::str(boost::format(foldername + "/depth_%05d.png") % (i+initnum)), depth);

      // For Pose estimation
        ofstream file;
        file.open(boost::str(boost::format(foldername + "/pose_%05d.txt") % (i+initnum)));
        file << renderer_iterator.class_id << ", " 
            << renderer_iterator.lon_ << ", " 
            << renderer_iterator.lat_ << ", " 
            << renderer_iterator.rot_ << ", "
            << renderer_iterator.rad_ << std::endl;
        file.close();

      // For Stanford Pose estimation
      // double lon_ = renderer_iterator.lon_ * 180 / CV_PI;
      // if (lon_ < 0) lon_ += 360;
      // double lat_ = renderer_iterator.lat_ * 180 / CV_PI;
      // if (lat_ < 0) lat_ += 360;
      // double rot_ = renderer_iterator.rot_ * 180 / CV_PI;
      // if (rot_ < 0) rot_ += 360;

      // stanfordpose << stanfordclass << " " 
      //     << lon_ << " " 
      //     << lat_ << " " 
      //     << rot_ << std::endl;

      // stanfordImage << boost::str(boost::format("image_%05d.png") % (i+initnum))
      //               << std::endl;

    } catch (...) {
      std::cout<<"Caught Exception :) "<<std::endl;
    }
  }

  return lastIndex;
}

bool copyObject(cv::Mat& background, const cv::Mat* object, const cv::Mat &mask, const cv::Vec2d &pos)
{
  if (pos[0] < 0 || pos[1] < 0) return false;
  int row_min = pos[0] - mask.rows/2;
  int row_max = pos[0] + mask.rows/2;
  int col_min = pos[1] - mask.cols/2;
  int col_max = pos[1] + mask.cols/2;

  //check if row position is feasible
  if(row_min < 0 || row_max > background.rows){
    std::cout<<"Failure: High/Low, "<<"Object Size : "<<mask.rows<<", "<<
                mask.cols<<" position : "<<pos[0]<<", "<<pos[1]<<std::endl;
    return false;
  }
  //check if column position is feasible
  if(col_min < 0 || col_max > background.cols){
    std::cout<<"Failure: Left/Right, "<<"Object Size : "<<mask.rows<<
                ", "<<mask.cols<<" position : "<<pos[0]<<", "<<pos[1]<<std::endl;
    return false;
  }

  for(int r = 0;r<mask.rows;r++)
    for(int c = 0;c<mask.cols;c++)
    {
      int i = row_min + r;
      int j = col_min + c;

      if((int)mask.at<uchar>(r,c) != 0){
        if (object != nullptr) {
          background.at<cv::Vec3b>(i,j) = object->at<cv::Vec3b>(r,c);
        } else {
          background.at<uchar>(i, j) = 255;
        }
      }
    }

  return true;
}

//Selecting position at the plane of the shelf
cv::Vec2d SamplePoint(const cv::Mat &background, const cv::Mat &object, double distance,  binImage bin)
{
  int offset_left = (bin.p_b[1] - bin.p_a[1])*(distance - SHELF_FRONT)/SHELF_DEPTH;
  int offset_right = (bin.p_d[1] - bin.p_c[1])*(distance - SHELF_FRONT)/SHELF_DEPTH;
  int left = offset_left + object.cols/2;
  int right = background.cols - offset_right - object.cols/2;

  int col_pixel = left + (right - left) * static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  int row_pixel_ab =  bin.p_a[0] + (bin.p_b[0] - bin.p_a[0]) * (distance - SHELF_FRONT)/SHELF_DEPTH;
  int row_pixel_cd =  bin.p_d[0] - (bin.p_d[0] - bin.p_c[0]) * (distance - SHELF_FRONT)/SHELF_DEPTH;
  int col_pixel_ab = offset_left + bin.p_a[1];
  int col_pixel_cd = bin.p_d[1] - offset_right;
  int row_pixel = row_pixel_ab + (row_pixel_cd - row_pixel_ab) * (col_pixel - col_pixel_ab)/(col_pixel_cd - col_pixel_ab);

  row_pixel -= object.rows/2;
  return cv::Vec2d(row_pixel, col_pixel);
}

//Selecting valid point in the background image
cv::Vec2d SamplePoint(const cv::Mat &background, const cv::Mat &object) {
  int range_min_row = object.rows / 2;
  int range_max_row = background.rows - object.rows / 2;

  int range_max_col = background.cols - object.cols / 2;
  int range_min_col = object.cols / 2;

  if (range_min_col > range_max_col || range_min_row > range_max_row) {
    return cv::Vec2d( -1, -1);
  } else {
    int row_idx = (double)rand()/RAND_MAX * (range_max_row - range_min_row) + range_min_row;
    int col_idx = (double)rand()/RAND_MAX * (range_max_col - range_min_col) + range_min_col;
    return cv::Vec2d(row_idx, col_idx);
  }
}

void train()
{
  //For stanford Pose estimation method
  ofstream file;
  ofstream imglist;
  file.open("PoseList.txt");
  imglist.open("StanfordImageList.txt");

  for(int i=0; i < NUM_OBJECTS; i++){
    int num_models = models[i];
    int index = 0;

    std::string foldername(objects_list[i]);

    //Create Folder
    DIR *dir;
    struct dirent *ent;
    if((dir = opendir(foldername.c_str())) == NULL)
      mkdir(foldername.c_str(), 0775);

    if (models[i] == 1) {    
      index = render3d(foldername + ".obj",foldername, im_width, im_height, index, i, file, imglist);
    } else {
      for(int j=0;j<models[i];j++){
        std::string file_name = boost::str(boost::format(foldername + "_%05d.obj") % (j));
        std::cout<< file_name <<std::endl;
        index = render3d(file_name, foldername, im_width, im_height, index,i, file, imglist);
        index += 1;
      }
    }
  }
  file.close();
}

int main(int argc, char **argv) {
  int begin_idx = 0;
  srand(time(NULL));
  std::string img_dir = "real_images";

  int pose_ob = 0;
  int classid = -1;

  ofstream imglist;

  if (argc >= 3) {
    img_dir = argv[2];
  }
  
  //If we want to train object models
  if(argc >= 2)
  {
    if(strcmp(argv[1],"train") == 0)
      train();
    else if (strcmp(argv[1],"pose") == 0) {
      validate();
    }
    else{
      begin_idx = atoi(argv[1]);
      for(int i = begin_idx;i < begin_idx + AUGMENTED_IMAGES; i++)
      {
        classid = -1;
        if (i % 100 == 0) std::cout<<"Generating Image : "<<i<<std::endl;
        
        if(opendir("result") == NULL){
          mkdir("result",0775);
        }
        imglist.open("result/Imagelist.txt",std::ios_base::app);

        //Chose random objects
        std::set<int> objects_selected;
        int objectsbin = rand()%OBJECTS_PER_BIN + 1;

        //For pose estimation, we need to select the given object
        if(argc == 4)
        {
          pose_ob = atoi(argv[3]);
          objects_selected.insert(pose_ob-1);
        }

        //Select random objects
        while (objects_selected.size() < objectsbin) {
          int objectid = rand()%NUM_OBJECTS;

          if(std::strcmp(&img_dir[0], "real_images") == 0){
            if(num_real_object[objectid] == 0)continue;
          }

          objects_selected.insert(objectid);
        }
        
        //Read Background image
        cv::Mat background = cv::imread(background_image, CV_LOAD_IMAGE_COLOR);
        if (!background.data){
            std::cout << "Could not find bg " << std::endl;
        }

        cv::Mat background_mask = cv::Mat::zeros(background.rows, background.cols, CV_8UC1);
        cv::Mat segmentedim = cv::Mat::zeros(background.rows, background.cols, CV_8UC1);
     
        //Read image, mask, distance from camera for each selected object
        std::vector<apc_object> objlist;
        objlist.clear();
        for(auto it = objects_selected.begin(); it != objects_selected.end(); it++)
        {
          int imageid;
          std::string folder = img_dir + "/" + objects_list[*it];

          // Select one image of the selected object and read it's pose
          std::vector<std::string> poses;
          if(std::strcmp(&img_dir[0],"virtual_images")==0){
            imageid= rand()%num_object[*it];
            ifstream posefile(boost::str(boost::format(folder + "/pose_%05d.txt") % (imageid)));
            std::string line;
            while(getline(posefile,line,','))
              poses.push_back(line);
            posefile.close();
          }

          else if(std::strcmp(&img_dir[0],"pose_images")==0){
            imageid= rand()%num_object[*it];
            if(*it == (pose_ob-1)){
              ifstream posefile(boost::str(boost::format(folder + "/pose_%05d.txt") % (imageid)));
              std::string line;
              while(getline(posefile,line,',')){
                poses.push_back(line);
              }
              posefile.close();
              classid = atoi(poses[0].c_str());
            }
          }

          else if(std::strcmp(&img_dir[0], "real_images") == 0)
            imageid= rand()%num_real_object[*it];
          else{
            std::cerr << "img_dir is wrong!!!" << std::endl;
            exit(-1);
          }
            
          std::string file = boost::str(boost::format(folder + "/image_%05d.png") % (imageid));
          std::string filemask = boost::str(boost::format(folder + "/mask_%05d.png") % (imageid));

          //Init Object
          apc_object ob;
          ob.index = (*it) + 1;

          //Error Check
          if(ob.index >40)
          {
            for(int i=0;i<100;i++)std::cout<<std::endl;
              std::cout<<ob.index<<std::endl;
            for(int i=0;i<100;i++)std::cout<<std::endl;
          }

          try {
            ob.image = cv::imread(file, CV_LOAD_IMAGE_COLOR);
            if (!ob.image.data) {
              std::cout << "Could not find image " << file << std::endl;
              continue;
            }
            ob.mask = cv::imread(filemask, CV_8UC1);
            if (!ob.image.data) {
              std::cout << "Could not find image " << filemask << std::endl;
              continue;
            }

            if(std::strcmp(&img_dir[0],"virtual_images")==0|| std::strcmp(&img_dir[0],"pose_images")==0){  
              ob.distance = atoi(poses[4].c_str());
            }
            else
              ob.distance = 0.4 + static_cast <double> (rand()) / static_cast <double> (RAND_MAX) * 0.4;

            objlist.push_back(ob);  
          } catch (...) {}
        
      }

      std::sort(objlist.begin(),objlist.end(), greater());

      //Copy objects to the background, and generate masks
      std::vector<apc_object_pos> objects_pos(objlist.size());
      for(int i=0; i < objlist.size(); i++){
        cv::Vec2d point;
        // APC Format
        // if(std::strcmp(&img_dir[0],"virtual_images")==0 || std::strcmp(&img_dir[0],"pose_images")==0) {
        //   //binImage bin;
        //   // bin.p_a = cv::Vec2d(570, 0);
        //   // bin.p_b = cv::Vec2d(470, 190);
        //   // bin.p_c = cv::Vec2d(470, 550);
        //   // bin.p_d = cv::Vec2d(570, 700);
        //   //point = SamplePoint(background, objlist[i].image, objlist[i].distance, bin);
        //   point = SamplePoint(background, objlist[i].image);
        // } 
        // else
        // {
        //   //For real images paste at the centre of the background image
        //   point[1] = background.cols/2;
        //   point[0] = background.rows/2;
        // }

        // ICRA Format
        point = SamplePoint(background, objlist[i].image);

        //Copying objects on the background
        copyObject(background, &objlist[i].image, objlist[i].mask, point);
        
        //Background mask is the OR of all the masks
        copyObject(background_mask, nullptr, objlist[i].mask, point);

        objects_pos[i].obj = objlist[i];
        objects_pos[i].mask_on_background = cv::Mat::zeros(background.rows, background.cols, CV_8UC1);

        //The object position class contains the mask of the object in the background
        copyObject(objects_pos[i].mask_on_background, nullptr, objlist[i].mask, point); 
      }

      //Store Bounding Boxes for each object
      std::string bboxfilename = boost::str(boost::format("result/cr_bbox_%05d.txt") % (i));
      ofstream bbox;
      bbox.open(bboxfilename);

      for (int i = objects_pos.size() - 1; i >= 0; i--) {
        cv::bitwise_and(background_mask, objects_pos[i].mask_on_background, objects_pos[i].final_mask);
        cv::bitwise_xor(background_mask, objects_pos[i].mask_on_background, background_mask, background_mask);

        // For whole bounding-boxes
        // cv::Mat locations;
        // cv::findNonZero( objects_pos[i].final_mask, locations);
        // if (locations.empty()) continue;
        // objects_pos[i].bounding_box = cv::boundingRect(locations);

        // For non-occluded bounding-boxes 
        std::vector<cv::Point> locations;
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourest;
        objects_pos[i].final_mask.copyTo(contourest);
        findContours( contourest, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        double maxcontour = 0;
        int max_idx = -1;
        for(int c = 0;c<contours.size();c++)
        {
          double temp = cv::contourArea(contours[c]);
          if( temp > maxcontour) {
            maxcontour = temp;
            max_idx = c;
          }
        }

        if(maxcontour < bbox_threshold)
        {
          std::cout<<"Object Occluded/small portion visible"<<std::endl;
          continue;
        }

        objects_pos[i].bounding_box = cv::boundingRect(contours[max_idx]);
        
        // Debug :: Draw bounding boxes
        // cv::rectangle(background, objects_pos[i].bounding_box, cv::Scalar(255, 0, 0));

        // Debug :: View the generated image
        // const std::string winName = "image";
        // cv::imshow(winName, background);
        // cv::waitKey(0);
        // cv::destroyWindow(winName);

        //For Poses
        if(argc == 4)
        {
          if(objects_pos[i].obj.index == pose_ob)
          {
            if (classid < 0) {
              std::cerr << "classid is not assigned!!!" << std::endl;
              exit(-1);
            }

            bbox << classid << "," <<
                objects_pos[i].bounding_box.x <<"," <<
                objects_pos[i].bounding_box.y <<","<< 
                (objects_pos[i].bounding_box.br()).x <<"," <<
                (objects_pos[i].bounding_box.br()).y<<std::endl;
          }
        }
        else
          bbox << objects_pos[i].obj.index << ","<<
                objects_pos[i].bounding_box.x <<"," <<
                objects_pos[i].bounding_box.y <<","<< 
                (objects_pos[i].bounding_box.br()).x <<"," <<
                (objects_pos[i].bounding_box.br()).y<<std::endl;
      }

      std::string res = boost::str(boost::format("result/image_%05d.png") % (i));
      cv::imwrite(res, background);
      bbox.close();

      std::string genimg = boost::str(boost::format("%05d") % (i));
      imglist << genimg << std::endl;
      
      imglist.close();

      if (i % 100 == 0) std::cout<<"##############################"<<std::endl;
      }
    }
  }
  return 0; 
}
