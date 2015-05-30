#ifndef _SONARIMAGEFEATURE_TOOLS_HPP_
#define _SONARIMAGEFEATURE_TOOLS_HPP_

#include "DetectorTypes.hpp"
#include <list>
#include <limits>
#include <base/samples/SonarScan.hpp>
#include <base/Eigen.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace sonar_image_feature_extractor
{
  
  
  /**
   * Smoothes the sonarimage.
   * Smooth-filter is aplied beam-wise
   */
  void smooth( cv::Mat &sonar_mat, int blur, SMOOTH_MODE const & smooth_mode)
  {
    
    if(!(blur%2)){
      blur += 1;
    }  
    //Smoothing
    if(smooth_mode == AVG){
      cv::blur( sonar_mat, sonar_mat, cv::Size(blur, 1), cv::Point(-1,-1));
    }else if(smooth_mode == GAUSSIAN){
      cv::GaussianBlur( sonar_mat, sonar_mat, cv::Size(blur, 1) ,0);
    }else if(smooth_mode == MEDIAN){
      cv::medianBlur(sonar_mat, sonar_mat, blur);
    }      
    
  }
  
  /**
   * Calculates the threshold of a sonarimage
   * @sonar_mat: Sonarimage in polarcoordinates
   * @threshold_mode: the aplied threshold-algorithm
   * @relative_threshold: Percentage-threshold of the maximum value
   * @adaptive_threshold_neighborhood: neighborhood-size for the local-adaptive-threshold   * 
   */
  bool threshold( cv::Mat &sonar_mat, THRESHOLD_MODE const & threshold_mode, double relative_threshold = 0.5, int adaptive_threshold_neighborhood = 15)
  {
    
      double min,max;
      cv::minMaxLoc( sonar_mat, &min, &max);
      
      //Thresholding
      if(threshold_mode == ABSOLUTE)
	cv::threshold(sonar_mat, sonar_mat, relative_threshold * max, 255, CV_THRESH_BINARY);
      else if(threshold_mode == ADAPTIVE_MEAN)
	cv::adaptiveThreshold(sonar_mat, sonar_mat, 255, cv::ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, adaptive_threshold_neighborhood , -relative_threshold * max);
      else if(threshold_mode == ADAPTIVE_GAUSSIAN)
	cv::adaptiveThreshold(sonar_mat, sonar_mat, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, adaptive_threshold_neighborhood, -relative_threshold * max);
      else if(threshold_mode == OTSU)
	cv::threshold(sonar_mat, sonar_mat, 0 , 255 , CV_THRESH_BINARY + CV_THRESH_OTSU);    
   
      return true;
  }
  
  
  /**
   * Calculates the entropy of an sonar-image
   * @sonar_mat: Sonarimage in polar-coordinates
   */
  float entropy (cv::Mat &sonar_mat)
  {
    cv::MatND hist;
    int histSize = 256;    
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    /// Compute the histograms:
    calcHist( &sonar_mat, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false );
    hist /= sonar_mat.total();

    cv::Mat logP;
    cv::log(hist,logP);

    return -1*cv::sum(hist.mul(logP)).val[0];
      
  }
  
  
}

#endif