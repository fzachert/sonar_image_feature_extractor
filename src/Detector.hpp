#ifndef _SONARIMAGEFEATURE_DETECTOR_HPP_
#define _SONARIMAGEFEATURE_DETECTOR_HPP_

#include "DetectorTypes.hpp"
#include <vector>
#include <list>
#include <limits>
#include <base/samples/Frame.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <machine_learning/DBScan.hpp>

namespace sonar_image_feature_extractor
{
  /**
   * This class represents one 2d-cluster
   * The cluster is defined by the number of datapoints, and the range of values
   */
  struct Cluster{
   
    Cluster() : number_of_points(0), minX(std::numeric_limits<double>::max()), minY(std::numeric_limits<double>::max()),
	maxX(-std::numeric_limits<double>::max()), maxY(-std::numeric_limits<double>::max()) {}
    
    int number_of_points;
    double minX, minY, maxX, maxY;
    
  };
  
  /**
   * This class represents the feature-detector, based on image-processing
   */
   class Detector{
   private:
     
     /**
      * This function applies clustering to the preprocessed sonar-image
      * @param mat: binary image
      *	@param config: reference to the detector-configuration
      * @return: vector of sonar-features 
      */
     SonarFeatures cluster(cv::Mat mat, const DetectorConfig &config);
     
     
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
     SonarFeatures detect(base::samples::frame::Frame &frame, base::samples::frame::Frame &debug_frame, const DetectorConfig &config); 
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_