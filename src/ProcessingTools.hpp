/* ----------------------------------------------------------------------------
 * ProcessingTools.cpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This file contains processing functions on sonar-scans
 * ----------------------------------------------------------------------------
*/

#ifndef _SONARIMAGEFEATURE_TOOLS_HPP_
#define _SONARIMAGEFEATURE_TOOLS_HPP_

#include "DetectorTypes.hpp"
#include <list>
#include <limits>
#include <math.h>
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
  
  double getRadGemini( int beam_index){
    return -std::asin(((2.0 * (beam_index + 1.0) - 256.0) / 256.0) * 0.86602540);
  }
  

  
/**
 * Process found sonar-cluster and calulate the peak-average and varriance
 * @param analysedCluster: list of found cluster in the sonar_image
 * @param sonar_Scan:  raw-data of the input_scan
 * @param threshold_mat: process sonar-scan withh threshold-values
 */  
void process_points(std::vector<Cluster> &analysedCluster, base::samples::SonarScan &sonar_scan, cv::Mat threshold_mat){
  
  //std::cout << "Threshold_mat " << threshold_mat.size().width << " " << threshold_mat.size().height << std::endl; 
  
  
  cv::Mat sonar_mat = cv::Mat(sonar_scan.number_of_beams, sonar_scan.number_of_bins, CV_8U, (void*) sonar_scan.data.data());
  
  double temp_sum = 0.0;
  int temp_points = 0;
  
  //Calculate avg-singnal in the cluster-areas  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sum = 0.0;
      temp_points = 0;
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
// 	std::cout << "angle_id: " << angle_index << " range_id: " << range_index << std::endl;
// 	std::cout << "angle: " << angle << " range: " << range << std::endl;
// 	std::cout << "min_angle: " << it->min_angle << " max_angle: " << it->max_angle << std::endl;
	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){

	  sum += temp_sum + sonar_mat.at<uint8_t>(angle, range);
	  num_points += temp_points + 1;
	  temp_sum = 0.0;
	  temp_points = 0;
	  
	}else{
	  temp_sum += sonar_mat.at<uint8_t>(angle, range);
	  temp_points++;
	}
      
      }
    
    } 
    
    if(num_points > 0)
      it->avg_signal= sum / num_points;    
    else
      it->avg_signal= 0;
    
  }
  
  
  //Calculate variance in cluster-areas
  temp_sum = 0.0;
  temp_points = 0;

  
  //Calculate avg-singnal in the cluster-areas  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    double avg = it->avg_signal;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sum = 0.0;
      temp_points = 0;
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){
	  
	  sum += temp_sum + std::pow(sonar_mat.at<uint8_t>(angle, range) - avg  , 2.0 );
	  num_points += temp_points + 1;
	  temp_sum = 0.0;
	  temp_points = 0;
	  
	}else{
	  temp_sum += std::pow(sonar_mat.at<uint8_t>(angle, range) - avg, 2.0)  ;
	  temp_points++;
	}
      
      }
    
    } 
    
    if(num_points > 0){
      it->variance = sum / num_points;
      it->variation_coefficient = std::sqrt( it->variance) / 127.5; 
    }else{
      it->variance = 0;
      it->variation_coefficient = 0;
    }
  }  
  
  
  
    //Calculate contrast
    for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      for(int range = it->min_range_id - 10; range <= it->min_range_id; range++){
	
	if(range > 0){
	  sum += sonar_mat.at<uint8_t>(angle, range);
	  num_points++;	  
	}
      
      }
      
      for(int range = it->max_range_id; range <= it->max_range_id + 10; range++){
	
	if(range < sonar_scan.number_of_bins){
	  sum += sonar_mat.at<uint8_t>(angle, range);
	  num_points++;	  
	}
      
      }      
      
    
    } 
    
    double avg_env = sum / num_points;
    
    if(avg_env > 0)
      it->contrast = it->avg_signal / avg_env;
    else
      it->contrast = 0;    
    
  }  
  
}  
  
  
  
  
/**
 *Calculate the central scale-invariant moments, up to order 4
 *@param analysedCluster: list of cluster in the sonar-image
 *@param sonar_scan: raw_data of the input-sonar_scan
 *@param threshold_mat: processed sonar-image with threshold-value 
 */  
 void calculate_moments(std::vector<Cluster> &analysedCluster, base::samples::SonarScan &sonar_scan, cv::Mat threshold_mat){
 
  base::Vector2d temp_sum; 
  double temp_points; 
  cv::Mat sonar_mat = cv::Mat(sonar_scan.number_of_beams, sonar_scan.number_of_bins, CV_8U, (void*) sonar_scan.data.data());
  
  
  //Calculate center_of_mass and sum of points
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    base::Vector2d sum = base::Vector2d::Zero();
    double num_points = 0;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sum = base::Vector2d::Zero();
      temp_points = 0;
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
// 	std::cout << "angle_id: " << angle_index << " range_id: " << range_index << std::endl;
// 	std::cout << "angle: " << angle << " range: " << range << std::endl;
// 	std::cout << "min_angle: " << it->min_angle << " max_angle: " << it->max_angle << std::endl;
	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){

	  sum += temp_sum + (sonar_mat.at<uint8_t>(angle, range) * base::Vector2d(range, angle ) );
	  num_points += temp_points + sonar_mat.at<uint8_t>(angle, range);
	  temp_sum = base::Vector2d::Zero();
	  temp_points = 0;
	  
	}else{
	  temp_sum += sonar_mat.at<uint8_t>(angle, range) * base::Vector2d( range, angle);
	  temp_points += sonar_mat.at<uint8_t>(angle, range);
	}
      
      }
    
    } 
    
    it->moments.setMoment( 0, 0, num_points);
    
    if(num_points > 0)
      it->moments.center_of_mass= sum / num_points;    
    else
      it->moments.center_of_mass = base::Vector2d::Zero();
      
  } 
  
  //Calculate moments
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    std::vector<double> sums(21, 0.0);
    std::vector<double> temp_sums( 21, 0.0);
    int centerX = it->moments.center_of_mass.x();
    int centerY = it->moments.center_of_mass.y();
    double value;
    int index;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sums.insert( temp_sums.begin(), 21, 0.0 );
      double angle_sin = std::sin( ( angle - centerY ) * sonar_scan.angular_resolution.rad );  
      double angle_cos = std::cos( ( angle - centerY ) * sonar_scan.angular_resolution.rad );
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
// 	std::cout << "angle_id: " << angle_index << " range_id: " << range_index << std::endl;
// 	std::cout << "angle: " << angle << " range: " << range << std::endl;
// 	std::cout << "min_angle: " << it->min_angle << " max_angle: " << it->max_angle << std::endl;
	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){
	  value = sonar_mat.at<uint8_t>(angle, range);
	  
	  for( int x = 0; x <= 4; x++){
	    for( int y = 0; y <= 4; y++){
	      
	      if( x + y >= 2 && x + y <= 4){
		index = (x * 5) + y;
		sums[ index] = temp_sums[ index ] 
		+ ( std::pow(range - centerX, x) * std::pow( ((double)range) * angle_sin, y ) * value); 
		
		temp_sums[ index] = 0;
	      }
	      
	    }
	  }

	  temp_sums.insert( temp_sums.begin(), 21, 0.0);
	  
	}else{
	  
	  for( int x = 0; x <= 4; x++){
	    for( int y = 0; y <= 4; y++){
	      
	      if( x + y >= 2 && x + y <= 4){
		index = (x * 5) + y;
		temp_sums[ index] = ( std::pow(range - centerX, x) * std::pow( ((double)range) * angle_sin , y ) 
		* value); 
		
	      }
	      
	    }
	  }
	  
	}
      
      }
    
    } 
    

    for( int x = 0; x <= 4; x++){
      for( int y = 0; y <= 4; y++){
	      
	if( x + y >= 2 && x + y <= 4){
	  index = (x * 5) + y;
	  
	  value = sums[ index] 
	    / std::pow( it->moments.getMoment(0,0), 1 + ((x+y)*0.5 ) ) ;
	  
	  it->moments.setMoment(x, y, value);
		
	}
	      
      }
    }
      
  }  
  
   
   
  
 } //End function
  
} //End namespace

#endif