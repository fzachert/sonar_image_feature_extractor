#ifndef _SONARIMAGEFEATURE_TYPES_HPP_
#define _SONARIMAGEFEATURE_TYPES_HPP_

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Time.hpp>
#include <vector>

namespace sonar_image_feature_extractor
{
  struct Destricptor{
    base::Vector3d dummy; //TODO
    
  };
  
   struct Feature{
     double confidence;
     base::Vector3d position;
     base::Vector3d size;
     Destricptor desc;
     
   };
   
   struct SonarConfig{
     base::Orientation orientation;
     
     double verticalOpening;
     double horizontalOpening;
     double maximumRange;
     double minimumRange;
   }; 
   
   
   struct SonarFeatures{
     base::Time time;
     SonarConfig conf;
     
     int number_of_features;
     int number_of_points;
     std::vector<Feature> features;
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
	  FEATURES = 4     
   };

    enum THRESHOLD_MODE {
	  ABSOLUTE = 0,
	  ADAPTIVE_MEAN = 1,
	  ADAPTIVE_GAUSSIAN = 2,
	  OTSU = 3
    };
  
  struct DetectorConfig{
    
    int blur;
    int morph;
    int sobel;
    double threshold;
    int cluster_min_size;
    int cluster_max_size;
    int cluster_noise;
    
    SMOOTH_MODE smooth_mode;
    DEBUG_MODE debug_mode;
    THRESHOLD_MODE threshold_mode;
    
    double sonar_max_range; //meter
    double sonar_opening_angle; //radian
    double ignore_min_range; //meter
   
  }; 

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_