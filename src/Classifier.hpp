#ifndef _SONARIMAGEFEATURE_CLASSIFIER_HPP_
#define _SONARIMAGEFEATURE_CLASSIFIER_HPP_

#include "DetectorTypes.hpp"
#include "ClassificationTypes.hpp"
#include "libsvm/svm.h"
#include <vector>
#include <list>



namespace sonar_image_feature_extractor
{
  
  class Classifier{
   private:
      SVMConfig config;
      svm_parameter param;
      svm_model *model;
    
      std::vector<svm_node> getNodes( Cluster &c);
     
   public:
     
     Classifier();
     ~Classifier();
     
     void init(SVMConfig config);
     
     bool classify(Cluster &c);
     
     bool learn( std::vector<Cluster> &clusters, std::vector<Label> &labels);
     
     
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_