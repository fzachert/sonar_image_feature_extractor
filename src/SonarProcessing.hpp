#ifndef _SONARIMAGEFEATURE_SONARPROCESSING_HPP_
#define _SONARIMAGEFEATURE_SONARPROCESSING_HPP_

#include "DetectorTypes.hpp"
#include <vector>
#include <list>
#include <limits>
#include <dsp_acoustics/FIRFilter.h>
#include <machine_learning/DBScan.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Eigen.hpp>

namespace sonar_image_feature_extractor
{
  
  struct SonarPeak{
    
    base::Vector2d pos;
    double range;
    double angle;
  };
  
  
  /**
   * This class represents the feature-detector, based on image-processing
   */
   class SonarProcessing{
   private:
     
     /**
      * This function applies clustering to the preprocessed sonar-image
      * @param mat: binary image
      *	@param config: reference to the detector-configuration
      * @return: vector of sonar-features 
      */
     SonarFeatures cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config);
     
     
   public:
     /**
      * Initialise the detector: reset the filter
      */
     void init();
     
     /**
      * Detect sonar-features in a sonar-image
      * @param frame: sonar-image
      * @param debug_frame: empty frame for the debug-image
      * @param config: Configuration of the detector
      * @return: Vector of sonar-features
      */
     SonarFeatures detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config); 
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_