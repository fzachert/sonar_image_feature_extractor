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
     Destricptor desc;
     
   };
   
   struct SonarFeatures{
     base::Time time;
     std::vector<Feature> features;
   };
  
  struct DetectorConfig{
    int debug_mode;
    
    int blur;
    int morph;
    int sobel;
    double threshold;
    int cluster_min_size;
    int cluster_max_size;
    int cluster_noise;
    
    double sonar_max_range; //meter
    double sonar_opening_angle; //radian
    double ignore_min_range; //meter
   
  }; 

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_