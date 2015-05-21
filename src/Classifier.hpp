#ifndef _SONARIMAGEFEATURE_CLASSIFIER_HPP_
#define _SONARIMAGEFEATURE_CLASSIFIER_HPP_

#include "DetectorTypes.hpp"
#include <vector>
#include <list>


namespace sonar_image_feature_extractor
{
  
  class Classifier{
   private:

     
     
   public:
     
     bool classify(Cluster c, DetectorConfig config);
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_