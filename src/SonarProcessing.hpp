/* ----------------------------------------------------------------------------
 * SonarProcessing.hpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This class provides a featue-detector on sonar-scans
 * ----------------------------------------------------------------------------
*/

#ifndef _SONARIMAGEFEATURE_SONARPROCESSING_HPP_
#define _SONARIMAGEFEATURE_SONARPROCESSING_HPP_

#include "DetectorTypes.hpp"
#include "ClassificationTypes.hpp"
#include "Classifier.hpp"
#include <vector>
#include <list>
#include <limits>
#include <dsp_acoustics/FIRFilter.h>
#include <machine_learning/DBScan.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Eigen.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace sonar_image_feature_extractor
{
  
  
  /**
   * This class represents the feature-detector, based on image-processing
   */
   class SonarProcessing{
   private:
     
     Classifier classifier;
     SVMConfig svm_conf;
     
     /**
      * This function applies clustering to the preprocessed sonar-image
      * @param mat: binary image
      *	@param config: reference to the detector-configuration
      * @return: vector of sonar-features 
      */  
     std::vector<Cluster> cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config, base::samples::SonarScan &sonar_scan, base::samples::SonarScan &debug_scan);
     
     std::vector<SonarPeak> process(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd, cv::Mat &threshold_mat);
     
   public:
     
     /**
      * Initialise the detector: reset the filter
      */
     void init( const SVMConfig &svm_conf);
     
     /**
      * Detect sonar-features in a sonar-image
      * @param frame: sonar-image
      * @param debug_frame: empty frame for the debug-image
      * @param config: Configuration of the detector
      * @param dd: empty debug-data struct. To be filed
      * @return: Vector of sonar-features
      */
     SonarFeatures detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd);
     
     std::vector<LabeledCluster> label_cluster(base::samples::SonarScan &input, DetectorConfig &config, DebugData &dd, Label label);
     
     /**
      * Distance functions, to be used by the clustering algorithm
      */
     static double distance(SonarPeak *p1, SonarPeak *p2);
     static double mahalanobis_distance(SonarPeak *p1, SonarPeak *p2);
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_