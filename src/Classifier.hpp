#ifndef _SONARIMAGEFEATURE_CLASSIFIER_HPP_
#define _SONARIMAGEFEATURE_CLASSIFIER_HPP_

#include "DetectorTypes.hpp"
#include "libsvm/svm.h"
#include <vector>
#include <list>


namespace sonar_image_feature_extractor
{
  
  class Classifier{
   private:

     
     
   public:
     
     bool classify(Cluster c, SVMConfig config);
     
     bool learn( std::vector<Cluster> positives, std::vector<Cluster> negatives, SVMConfig config);
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_