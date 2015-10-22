/* ----------------------------------------------------------------------------
 * DetectorTypes.hpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This file provides types, used inside the feature-detector
 * ----------------------------------------------------------------------------
*/

#ifndef _SONARIMAGEFEATURE_TYPES_HPP_
#define _SONARIMAGEFEATURE_TYPES_HPP_

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Time.hpp>
#include <base/samples/Pointcloud.hpp>
#include <base/samples/SonarBeam.hpp>
#include <vector>
#include <math.h>
#include "libsvm/svm.h"

namespace sonar_image_feature_extractor
{
  /**
   * Discibes a sonar-feature
   */
  struct Destricptor{
    base::Vector3d dummy; //TODO
    int label;
    
  };
  
  /*
   * A single sonar-feature
   */
   struct Feature{
     double confidence;
     base::Vector2d position;
     double range;
     double angle_h;
     base::Vector2d size;
     Destricptor desc;
     
   };
   

   /**
    * Configuration of the sonar
    */
   struct SonarConfig{
     base::Orientation orientation;
     
     double verticalOpening;
     double horizontalOpening;
     double maximumRange;
     double minimumRange;
   }; 
   
   /**
    * A list of sonar-features, as output of the detector
    */   
   struct SonarFeatures{
     base::Time time;
     SonarConfig conf;
     
     std::vector<Feature> features;
   };
   
   /**
    * A single sonar-peak in the sonar-scan
    */
  struct SonarPeak{
    
    base::Vector2d pos;
    int range_id;
    int angle_id;
    double range;
    double angle;
    
    double norm(){
      return range;      
    }
    
    SonarPeak operator-(const SonarPeak& other){
      SonarPeak result;
      result.pos = this->pos - other.pos;
      result.range = result.pos.norm();
      result.angle = std::atan2(result.pos.y(),result.pos.x()); 
      
      return result;
    }
    
    
  };   
  
  /**
   * A geometric-moment as a descritor of a sonar-cluster
   */
  struct Moment2D{
    
    int orderX, orderY;
    double value;
  
    Moment2D(){
      orderX = 0;
      orderY = 0;
      value = NAN;
    }
    
    Moment2D(int x, int y,double value){
      this->orderX = x;
      this->orderY = y;
      this->value = value;
    }
  };
  
  /**
   * Gemetric moments of a single sonar-cluster
   */
  struct Moments{    
    base::Vector2d center_of_mass;  
    std::vector<Moment2D> moments;
    
    
    /**
     * Getter-Method for a moment with given orders
     * If moments is unknown, return is NAN
     */
    double getMoment( int orderX, int orderY){
      
      for(std::vector<Moment2D>::iterator it = moments.begin(); it != moments.end(); it++){
	
	if( it->orderX == orderX && it->orderY == orderY)
	  return it->value;
	
      }
      
      
      return NAN;
    }
    
    /**
     * Setter-Method for a moment
     */
    void setMoment( int orderX, int orderY, double value){
      
      for(std::vector<Moment2D>::iterator it = moments.begin(); it != moments.end(); it++){
	
	if( it->orderX == orderX && it->orderY == orderY){
	  it->value = value;
	  return;
	}
	
      }      
      moments.push_back( Moment2D( orderX, orderY, value) );      
    }
    
    
  };
  
   
  /**
   * This class represents one 2d-cluster
   * The cluster is defined by the number of datapoints, and the range of values
   */   
    struct Cluster{
   
    Cluster() : number_of_points(0), minX(std::numeric_limits<double>::max()), minY(std::numeric_limits<double>::max()),
	maxX(-std::numeric_limits<double>::max()), maxY(-std::numeric_limits<double>::max()),
	min_range(std::numeric_limits<double>::max()), max_range(0.0), min_angle(M_PI), max_angle(-M_PI),
	min_angle_id(255), max_angle_id(0), min_range_id(std::numeric_limits<int>::max()), max_range_id(0),
	sum_pos(base::Vector2d::Zero()) {}
    
    int number_of_points;
    double minX, minY, maxX, maxY;
    double min_range, max_range, min_angle, max_angle;
    int min_angle_id, max_angle_id, min_range_id, max_range_id;
    
    base::Vector2d sum_pos;
    base::Vector2d middle;    
    base::Vector2d avg_pos;
    
    double variance;
    double contrast;
    double range_size;
    double angle_size;
    
    double variation_coefficient;    
    
    double avg_signal;
    
    Moments moments;
    
  };   
   

   enum SMOOTH_MODE {
            GAUSSIAN = 0,
            AVG = 1,
            MEDIAN = 2
   };
   
   enum DEBUG_MODE {
	  NO_DEBUG = 0,
	  SMOOTHING = 1,
	  SOBEL = 2,
	  THRESHOLD = 3,
	  FEATURES = 4,
	  DETECTED_FEATURES = 5
   };

    enum THRESHOLD_MODE {
	  ABSOLUTE = 0,
	  ADAPTIVE_MEAN = 1,
	  ADAPTIVE_GAUSSIAN = 2,
	  OTSU = 3
    };
    
    enum DISTANCE_MODE {
      EUKLIDIAN = 0,
      MAHALANOBIS =1
    };
  
  //Configuration of the sonar-processing  
  struct DetectorConfig{
    
    int blur;
    int morph;
    int sobel;
    double threshold;
    int adaptive_threshold_neighborhood;
    int cluster_min_size;
    int cluster_max_size;
    double cluster_noise;
    double feature_max_size;
    
    SMOOTH_MODE smooth_mode;
    DEBUG_MODE debug_mode;
    THRESHOLD_MODE threshold_mode;
    DISTANCE_MODE distance_mode;
    
    double sonar_max_range; //meter
    double sonar_opening_angle; //radian
    double ignore_min_range; //meter
    
    bool gemini;
   
  }; 

  
  struct DebugData{
    std::vector<Cluster> cluster;
    double entropy;
    base::samples::Pointcloud points; //Extracted peaks in cartesian-coordinates
    std::vector<base::samples::SonarBeam> center_beams;
    
    double time_preprocessing;
    double time_segmentation;
    double time_clustering;
    double time_extraction;
    double time_classification;
    double time_total;
    
  };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_