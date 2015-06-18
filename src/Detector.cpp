/* ----------------------------------------------------------------------------
 * Detector.cpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This provides a feature-detector in sonar-images using image-processing
 * Not finished!!!
 * Use Sonar-Processing.hpp insted for detection in sonarscans
 * ----------------------------------------------------------------------------
*/

#include "Detector.hpp"
#include <iostream>
#include "frame_helper/FrameHelper.h"

using namespace cv;
using namespace sonar_image_feature_extractor;

void Detector::init(){
  
}

SonarFeatures Detector::detect(base::samples::frame::Frame &frame, base::samples::frame::Frame &debug_frame, const DetectorConfig &config){
 
      std::vector<Feature> result;
      SonarFeatures feat;
  
      if(config.debug_mode != 0)
	debug_frame.init(frame.getWidth(), frame.getHeight(), frame.getDataDepth(), base::samples::frame::MODE_GRAYSCALE, false);
	
      
      
      Mat cvmat = frame_helper::FrameHelper::convertToCvMat(frame);
      Mat sobelcv(cvmat.size(), 16, 1);
      Mat smallcv(cvmat.size(), 8, 1);
      Mat thresholdcv(cvmat.size(), 8, 1);
      //Mat morphcv(cvGetSize(&cvmat), 8, 1);
      Mat debugMat;
      
      int blur = config.blur;
      if(!(blur % 2)){
          blur += 1;
      }
      if(blur > 1 && blur < cvmat.size().width/2){
	
	if(config.smooth_mode == GAUSSIAN)	
	  GaussianBlur(cvmat, cvmat, Size(blur,blur), 0);
	if(config.smooth_mode == AVG)
	  cv::blur( cvmat, cvmat, Size( blur, blur ), Point(-1,-1) );
	if(config.smooth_mode == MEDIAN)
	  medianBlur(cvmat, cvmat, blur);
	
      }      
      
      int sobel = config.sobel;
      if(!(sobel % 2)){
        sobel += 1;
      }
      
      if(config.sobel > 0.0){
        //cvLaplace(&cvmat, sobelcv, hessian);
        //sobel(cvmat, sobelcv, 0, 1, hessian);
	Sobel( cvmat, sobelcv, -1, 0, 1, sobel, 1, 0, BORDER_DEFAULT );
      }
      else{
         cvmat.copyTo(sobelcv);
      }
      
      convertScaleAbs(sobelcv, smallcv, 1./40);
      
      double min,max;
      minMaxLoc(smallcv, &min, &max);
      
      //cvThreshold(sobelcv, thresholdcv, _threshold.get() * max, max , CV_THRESH_BINARY); //CV_THRESH_TOZERO);
      if(config.threshold > 0.0){
	
	if(config.threshold_mode == ABSOLUTE)
	{
	  //Absolute thresholding
	  threshold(smallcv, thresholdcv, config.threshold * max, 255 , CV_THRESH_BINARY); //CV_THRESH_TOZERO);
	}
	else if(config.threshold_mode == ADAPTIVE_MEAN)
	{
	  //Adaptive threshold with mean of neighborhood, use 7 neighborhood
	  adaptiveThreshold(smallcv, thresholdcv, 255, ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 7 ,0);
	}
	else if(config.threshold_mode == ADAPTIVE_GAUSSIAN)
	{
	  //Adaptive threshold with weighted gaussian mean of neighborhood
	  adaptiveThreshold(smallcv, thresholdcv, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
	}
	else if(config.threshold_mode == OTSU)
	{
	  //Adaptive threshold based on Otsu's method -> analysis of histogramm
	  threshold(smallcv, thresholdcv, 0, 255, CV_THRESH_BINARY + CV_THRESH_OTSU);
	}
	
      }
      else{
        smallcv.copyTo(thresholdcv);
      }
      
//      if(_morph > 0.0){
//        dilate(thresholdcv, morphcv, 0, morph);
//        erode(morphcv, morphcv, 0, (morph / 2) + 1);
//      }
//      else{
//        cvCopy(thresholdcv, morphcv);
//      }
      
      SimpleBlobDetector detector;
      std::vector<KeyPoint> keypoints;
      detector.detect( thresholdcv, keypoints);

      
      feat = cluster(thresholdcv, config); 
      
      
      if(config.debug_mode == SMOOTHING){
	debugMat = cvmat.clone();
	//frame_helper::FrameHelper::copyMatToFrame(cvmat,debug_frame);
      }
      else if(config.debug_mode == SOBEL){
	debugMat = sobelcv.clone();
	//frame_helper::FrameHelper::copyMatToFrame(sobelcv, debug_frame);
      }
      else if(config.debug_mode == THRESHOLD){
	debugMat = thresholdcv.clone();
	//frame_helper::FrameHelper::copyMatToFrame(thresholdcv, debug_frame);
      }
      if(config.debug_mode != NO_DEBUG){
	
	double ratio = config.sonar_max_range / cvmat.size().height;
	base::Vector3d origin;
	origin.x() = (cvmat.size().width/2.0) * ratio;
	origin.y() = config.sonar_max_range;
	origin.z() = 0.0;	
	  
	  
	  for(std::vector<Feature>::iterator it = feat.features.begin(); it != feat.features.end(); it++){
	    base::Vector3d realPoint(origin.x() - it->position.y(), origin.y() - it->position.x(), 0.0);
	    realPoint /= ratio;
	    Point p(realPoint.x(), realPoint.y());
	    int radius = (it->size.x() + it->size.y()) / (ratio * 2.0);
	    //std::cout << "Radius: " << radius << std::endl;
	    //std::cout << it->size<<std::endl;
	    
	    circle(debugMat, p,  radius, Scalar(255,255,255,255));
	    
	  }
	  frame_helper::FrameHelper::copyMatToFrame(debugMat, debug_frame);
      }
    
    
    return feat;
}



SonarFeatures Detector::cluster(cv::Mat mat, const DetectorConfig &config){
  
  std::vector<cv::Point2i> locations;   // output, locations of non-zero pixels 
  cv::findNonZero(mat, locations);
  
  double ratio = config.sonar_max_range / mat.size().height;
  base::Vector3d origin;
  origin.x() = config.sonar_max_range;
  origin.y() = (mat.size().width/2.0) * ratio;
  origin.z() = 0.0;
  
  std::vector<base::Vector3d> dataPoints;
  std::list<base::Vector3d*>  dataPointers;
  
  dataPoints.reserve(locations.size());
    
  for(std::vector<cv::Point2i>::iterator it = locations.begin(); it != locations.end(); it++){
    base::Vector3d v;
    v.x() = it->y * ratio;
    v.y() = it->x * ratio;
    v.z() = 0.0;
    
    v = origin - v;
    double distance = v.norm();
    double angle = std::fabs( std::atan2(v.y(), v.x()) );
    
    if(distance > config.ignore_min_range && distance < config.sonar_max_range - config.sobel  && std::fabs( (config.sonar_opening_angle * 0.5) - angle) > 0.02 ){
    
      dataPoints.push_back(v);
      dataPointers.push_back(&dataPoints.back());
    }  
    
  }
  
  machine_learning::DBScan<base::Vector3d> scan(&dataPointers, config.cluster_min_size, config.cluster_noise); 
  std::map< base::Vector3d* , int> clusteredPoints = scan.scan();
  
  int numberOfCluster = scan.getClusterCount();
  
  std::vector<Cluster> analysedCluster;
  analysedCluster.resize(numberOfCluster);
  
  for(std::map< base::Vector3d* , int> ::iterator it = clusteredPoints.begin(); it != clusteredPoints.end(); it++){
    
    int id = it->second;
    
    if(id >= 0){
      
      base::Vector3d v = *(it->first);
      
      analysedCluster[id].number_of_points++;
      
      if( v.x() < analysedCluster[id].minX)
	analysedCluster[id].minX = v.x();
      if( v.x() > analysedCluster[id].maxX)
	analysedCluster[id].maxX = v.x();
      if( v.y() < analysedCluster[id].minY)
	analysedCluster[id].minY = v.y();
      if( v.y() > analysedCluster[id].maxY)
	analysedCluster[id].maxY = v.y();
      
    }
        
  }
  
  SonarFeatures feat;
  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    if(it->number_of_points < config.cluster_max_size){
      Feature f;
      f.position.x() = (it->maxX + it->minX) * 0.5;
      f.position.y() = (it->maxY + it->minY) * 0.5;
      f.position.z() = 0.0;
      f.size.x() = (it->maxX - it->minX);
      f.size.y() = (it->maxY - it->minY);
      feat.features.push_back(f);
    }
    
  }

  
  return feat;
}

