/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Vincent Rabaud
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <object_recognition_renderer/renderer3d.h>
#include <iostream>
#include <stdlib.h>
#include <GL/gl.h>
#include "model.h"
#include <boost/date_time/posix_time/posix_time.hpp>

#if USE_GLUT
#include "renderer3d_impl_glut.h"
#else
#include "renderer3d_impl_osmesa.h"
#endif

 Renderer3d::Renderer3d(const std::string & mesh_path)
 :
 renderer_(new Renderer3dImpl(mesh_path, 0, 0)),
 angle_(0),
 focal_length_x_(0),
 focal_length_y_(0),
 near_(0),
 far_(0),
 model_(new Model()),
 scene_list_(0)
 {
  // get a handle to the predefined STDOUT log stream and attach
  // it to the logging system. It remains active for all further
  // calls to aiImportFile(Ex) and aiApplyPostProcessing.
  ai_stream_ = new aiLogStream(aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL));
  aiAttachLogStream(ai_stream_);
}

Renderer3d::~Renderer3d()
{
  // We added a log stream to the library, it's our job to disable it
  // again. This will definitely release the last resources allocated
  // by Assimp.
  aiDetachAllLogStreams();
}

void
Renderer3d::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near,
 double far, int lighting_mode)
{
  renderer_->width_ = width;
  renderer_->height_ = height;

  focal_length_x_ = focal_length_x;
  focal_length_y_ = focal_length_y;

  near_ = near;
  far_ = far;

  renderer_->clean_buffers();

  // Initialize the OpenGL context
  renderer_->set_parameters_low_level();

  model_->LoadModel(renderer_->mesh_path_);

  // Initialize the environment
  glClearColor(0.f, 0.f, 0.f, 1.f);

  glEnable(GL_LIGHTING);

  // Having max RGBA global light does not help much
  GLfloat lmodel_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat lmodel_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_diffuse);

  glEnable(GL_LIGHT0); // Uses default lighting parameters

  glEnable(GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);



  
  // GLfloat LightAmbient[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  // GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  // GLfloat LightPosition[]= { 0.0f, 0.0f, 1.0f, 1.0f };
  // glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  // glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
  // glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
  // glEnable(GL_LIGHT1);

  if(lighting_mode == 2)
  {
    std::cout << "line mode 2" << std::endl;
    GLfloat LightAmbient2[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightDiffuse2[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightPosition2[]= { 0.0f, 0.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT2, GL_AMBIENT, LightAmbient2);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, LightDiffuse2);
    glLightfv(GL_LIGHT2, GL_POSITION, LightPosition2);
    glEnable(GL_LIGHT2);

    GLfloat LightAmbient3[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightDiffuse3[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightPosition3[]= { -1.0f, 0.0f, 0.0f, 1.0f };
    glLightfv(GL_LIGHT3, GL_AMBIENT, LightAmbient2);
    glLightfv(GL_LIGHT3, GL_DIFFUSE, LightDiffuse2);
    glLightfv(GL_LIGHT3, GL_POSITION, LightPosition2);
    glEnable(GL_LIGHT3);

    GLfloat LightAmbient4[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightDiffuse4[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightPosition4[]= { 0.0f, -1.0f, 0.0f, 1.0f };
    glLightfv(GL_LIGHT4, GL_AMBIENT, LightAmbient2);
    glLightfv(GL_LIGHT4, GL_DIFFUSE, LightDiffuse2);
    glLightfv(GL_LIGHT4, GL_POSITION, LightPosition2);
    glEnable(GL_LIGHT4);
  }

  // Initialize the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  double fx = Renderer3d::focal_length_x_;
  double fy = Renderer3d::focal_length_y_;
  double fovy = 2 * atan(0.5 * renderer_->height_ / fy) * 180 / CV_PI;
  double aspect = (renderer_->width_ * fy) / (renderer_->height_ * fx);

  // set perspective
  gluPerspective(fovy, aspect, near, far);
  glViewport(0, 0, renderer_->width_, renderer_->height_);
}

void
Renderer3d::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near,
 double far)
{
  renderer_->width_ = width;
  renderer_->height_ = height;

  focal_length_x_ = focal_length_x;
  focal_length_y_ = focal_length_y;

  near_ = near;
  far_ = far;

  renderer_->clean_buffers();

  // Initialize the OpenGL context
  renderer_->set_parameters_low_level();

  model_->LoadModel(renderer_->mesh_path_);

  // Initialize the environment
  glClearColor(0.f, 0.f, 0.f, 1.f);

  GLfloat lmodel_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0); // Uses default lighting parameters

  glEnable(GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

  //
  GLfloat LightAmbient[]= { 0.8f, 0.8f, 0.8f, 0.8f };
  GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat LightPosition[]= { 0.0f, 0.0f, -1.0f, 1.0f };
  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
  glEnable(GL_LIGHT1);

  // Initialize the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  double fx = Renderer3d::focal_length_x_;
  double fy = Renderer3d::focal_length_y_;
  double fovy = 2 * atan(0.5 * renderer_->height_ / fy) * 180 / CV_PI;
  double aspect = (renderer_->width_ * fy) / (renderer_->height_ * fx);

  // set perspective
  gluPerspective(fovy, aspect, near, far);
  glViewport(0, 0, renderer_->width_, renderer_->height_);
}

void Renderer3d::randomLight(cv::Vec3d T, cv::Vec3d up) {


  int num_of_light = rand() % 4;

    // Initialize the environment
  glClearColor(0.f, 0.f, 0.f, 1.f);

  GLfloat lmodel_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);


  glEnable(GL_LIGHTING);
  // light #1: from the camera
  float r = (double) rand() / (RAND_MAX) ;
  if (r < 0.5) r = 0.5;
  //if ( r > 0.5) T[1] = -T[1];
  //r = 1;
  GLfloat LightAmbient[]= { r, r, r, 1.0f };
  GLfloat LightDiffuse[]= { r, r, r, 1.0f };
  GLfloat LightPosition[]= { T[0], T[1], T[2], 1.0f };
  glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
  glEnable(GL_LIGHT0); // Uses default lighting parameters

  // std::cout << "light 0 configuration: intense, " << r <<
  // "; position: " << T << std::endl;

  glEnable(GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

  //light #1: from up of camera
  cv::Vec3d pos = T + up;
  if ((double)rand() / RAND_MAX > 0.1) {
   r = (double) rand() / (RAND_MAX);
   //if (r < 0.5) r = 0.5;
   GLfloat LightAmbient1[]= { r, r, r, 1.0f };
   GLfloat LightDiffuse1[]= { r, r, r, 1.0f };
   GLfloat LightPosition1[]= { pos[0], pos[1], pos[2], 1.0f };
   glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient1);
   glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse1);
   glLightfv(GL_LIGHT1, GL_POSITION, LightPosition1);
   glEnable(GL_LIGHT1);
 } else {
  glDisable(GL_LIGHT1);
}

  // std::cout << "random light 2 set." << std::endl;
if ((double)rand() / RAND_MAX > 0.1) {
 pos = T - up;
 r = (double) rand() / (RAND_MAX) ;
 //if (r < 0.5) r = 0.5;
 GLfloat LightAmbient2[]= { r, r, r, 1.0f };
 GLfloat LightDiffuse2[]= { r, r, r, 1.0f };
 GLfloat LightPosition2[]= { pos[0], pos[1], pos[2], 1.0f };
 glLightfv(GL_LIGHT2, GL_AMBIENT, LightAmbient2);
 glLightfv(GL_LIGHT2, GL_DIFFUSE, LightDiffuse2);
 glLightfv(GL_LIGHT2, GL_POSITION, LightPosition2);
 glEnable(GL_LIGHT2);
} else {
  glDisable(GL_LIGHT2);
} 


up = T.cross(up);
cv::normalize(up, up, 1.0);

if ((double)rand() / RAND_MAX > 0.1) {
 pos = T + up;
 r = (double) rand() / (RAND_MAX) ;
 //if (r < 0.5) r = 0.5;
 GLfloat LightAmbient3[]= { r, r, r, 1.0f };
 GLfloat LightDiffuse3[]= { r, r, r, 1.0f };
 GLfloat LightPosition3[]= { pos[0], pos[1], pos[2], 1.0f };
 glLightfv(GL_LIGHT3, GL_AMBIENT, LightAmbient3);
 glLightfv(GL_LIGHT3, GL_DIFFUSE, LightDiffuse3);
 glLightfv(GL_LIGHT3, GL_POSITION, LightPosition3);
 glEnable(GL_LIGHT3);
} else {
 glDisable(GL_LIGHT3);
}


if ((double)rand()/RAND_MAX > 0.1) {
 pos = T - up;
 r = (double) rand() / (RAND_MAX) ;
 //if (r < 0.5) r = 0.5;
 GLfloat LightAmbient4[]= { r, r, r, 1.0f };
 GLfloat LightDiffuse4[]= { r, r, r, 1.0f };
 GLfloat LightPosition4[]= { pos[0], pos[1], pos[2], 1.0f };
 glLightfv(GL_LIGHT4, GL_AMBIENT, LightAmbient4);
 glLightfv(GL_LIGHT4, GL_DIFFUSE, LightDiffuse4);
 glLightfv(GL_LIGHT4, GL_POSITION, LightPosition4);
 glEnable(GL_LIGHT4);
} else {
  glDisable(GL_LIGHT4);
}

   // Initialize the projection

   //glMatrixMode(GL_PROJECTION);
   //glLoadIdentity();



  // if (num_of_light > 2) {

  // }


  // if(lighting_mode == 2)
  // {
  //   std::cout << "line mode 2" << std::endl;


  //   GLfloat LightAmbient3[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  //   GLfloat LightDiffuse3[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  //   GLfloat LightPosition3[]= { -1.0f, 0.0f, 0.0f, 1.0f };
  //   glLightfv(GL_LIGHT3, GL_AMBIENT, LightAmbient2);
  //   glLightfv(GL_LIGHT3, GL_DIFFUSE, LightDiffuse2);
  //   glLightfv(GL_LIGHT3, GL_POSITION, LightPosition2);
  //   glEnable(GL_LIGHT3);

  //   GLfloat LightAmbient4[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  //   GLfloat LightDiffuse4[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  //   GLfloat LightPosition4[]= { 0.0f, -1.0f, 0.0f, 1.0f };
  //   glLightfv(GL_LIGHT4, GL_AMBIENT, LightAmbient2);
  //   glLightfv(GL_LIGHT4, GL_DIFFUSE, LightDiffuse2);
  //   glLightfv(GL_LIGHT4, GL_POSITION, LightPosition2);
  //   glEnable(GL_LIGHT4);
  // }
}

void Renderer3d::rotateXYZ() {
  glRotatef(anglex, 1, 0, 0);
  glRotatef(angley, 0, 1, 0);
  glRotatef(anglez, 0, 0, 1);
}

void Renderer3d::rotateZYX() {
  glRotatef(anglez, 0, 0, 1);
  glRotatef(angley, 0, 1, 0);
  glRotatef(anglex, 1, 0, 0);
}


void
Renderer3d::lookAt(double x, double y, double z, double upx, double upy, double upz)
{
  renderer_->bind_buffers();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(x, y, z, 0, 0, 0, upx, upy, upz);
  // gluLookAt(0, 0, 0.2, 0, 0, 0, 0, 1, 0);

  switch (mode) {
    case 0: rotateXYZ();break;
    case 1: rotateZYX();break;
  }

  // scale the whole asset to fit into our view frustum
   aiVector3D scene_min, scene_max, scene_center;
   model_->get_bounding_box(&scene_min, &scene_max);
   scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
   scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
   scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

  // // center the model
   glTranslatef(-scene_center.x, -scene_center.y, -scene_center.z);

  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_->Draw();
    glEndList();
  }

  glCallList(scene_list_);
}

void
Renderer3d::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);
  cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
  cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

  glFlush();

  // Get data from the depth/image buffers
  renderer_->bind_buffers();

  // Deal with the RGB image
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  // Deal with the depth image
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
  for (unsigned int j = 0; j < renderer_->height_; ++j)
    for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      if (*it > max_allowed_z)
        *it = 0;
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
    cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
    depth.convertTo(depth_scale, CV_16UC1, 1e3);

  // Crop the images, just so that they are smaller to write/read
    if (i_min > 0)
      --i_min;
    if (i_max < renderer_->width_ - 1)
      ++i_max;
    if (j_min > 0)
      --j_min;
    if (j_max < renderer_->height_ - 1)
      ++j_max;
    rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

    if ((rect.width <=0) || (rect.height <= 0)) {
      depth_out = cv::Mat();
      image_out = cv::Mat();
      mask_out = cv::Mat();
    } else {
      depth_scale(rect).copyTo(depth_out);
      image(rect).copyTo(image_out);
      mask(rect).copyTo(mask_out);
    }
  }

  void
  Renderer3d::renderDepthOnly(cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
  {
  // Create images to copy the buffers to
    cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
    cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

    glFlush();

  // Get data from the OpenGL buffers
    renderer_->bind_buffers();

  // Deal with the depth image
    glReadBuffer(GL_DEPTH_ATTACHMENT);
    glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

    float zNear = near_, zFar = far_;
    cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
    float max_allowed_z = zFar * 0.99;

    unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
    for (unsigned int j = 0; j < renderer_->height_; ++j)
      for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
      {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
        *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
        if (*it > max_allowed_z)
          *it = 0;
        else
        {
          mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
          if (j > j_max)
            j_max = j;
          else if (j < j_min)
            j_min = j;
          if (i > i_max)
            i_max = i;
          else if (i < i_min)
            i_min = i;
        }
      }

      // Rescale the depth to be in millimeters
      cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
      depth.convertTo(depth_scale, CV_16UC1, 1e3);

      // Crop the images, just so that they are smaller to write/read
      if (i_min > 0)
        --i_min;
      if (i_max < renderer_->width_ - 1)
        ++i_max;
      if (j_min > 0)
        --j_min;
      if (j_max < renderer_->height_ - 1)
        ++j_max;
      rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

      //std::cout << "rect.width = " << rect.width << ", rect.height = " << rect.height << std::endl;

      if ((rect.width <=0) || (rect.height <= 0)) {
        depth_out = cv::Mat();
        mask_out = cv::Mat();
      } else {
        depth_scale(rect).copyTo(depth_out);
        mask(rect).copyTo(mask_out);
      }
    }

    void
    Renderer3d::renderImageOnly(cv::Mat &image_out, const cv::Rect &rect) const
    {
      // Create images to copy the buffers to
      cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);
      // std::cout << renderer_->height_ << " " << renderer_->width_ << std::endl;

      glFlush();
      // Get data from the OpenGL buffers
      renderer_->bind_buffers();
      // Deal with the RGB image
      glReadBuffer(GL_COLOR_ATTACHMENT0);
      glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

      if ((rect.width <=0) || (rect.height <= 0)) {
        image_out = cv::Mat();
      } else {
        image(rect).copyTo(image_out);
      }
    }
