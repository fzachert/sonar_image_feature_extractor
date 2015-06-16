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
     
     int classify(Cluster &c);
     
     SVMConfig learn( std::vector<Cluster> &clusters, std::vector<Label> &labels);
     
     void cross_validate_params(svm_problem &prob);
     
     void cross_validate_result(svm_problem &prob);
     
     void validate_model(svm_problem &prob);
     
     void validate_model( std::vector<Cluster> &clusters, std::vector<Label> &labels);
     
     void calculateScalling( std::vector<std::vector<svm_node> >  &nodes );
     
     void scale(std::vector< svm_node> &nodes);
     
     void scaleAllNodes( std::vector< std::vector< svm_node> > &nodes);
     
     static void print_null(const char *s) {};
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_