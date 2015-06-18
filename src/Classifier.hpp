/* ----------------------------------------------------------------------------
 * Classifier.hpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This class provides a svm-classifier of sonar-cluster
 * ----------------------------------------------------------------------------
*/

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
      
     void cross_validate_params(svm_problem &prob);
     
     void cross_validate_result(svm_problem &prob);
     
     void validate_model(svm_problem &prob);
     
     void validate_model( std::vector<Cluster> &clusters, std::vector<Label> &labels);
     
     void calculateScalling( std::vector<std::vector<svm_node> >  &nodes );
     
     void scale(std::vector< svm_node> &nodes);
     
     void scaleAllNodes( std::vector< std::vector< svm_node> > &nodes);      
    
     
   public:
     
     Classifier();
     ~Classifier();
     
     /**
      * Initialize the classifier with a configuration
      */
     void init(SVMConfig config);
     
     /**
      * Classify a cluster, using a svm
      * @param c: input-cluster
      * @return: label of the input-cluster
      */
     int classify(Cluster &c);
     
     /**
      * Train a svm. 
      * After training, the svm is saved in the defined filepath of the config
      * @param clusters: vector of training-cluster
      * @param labels: vector of training-labels. size(cluster) == size(labels)
      * @return configuration with learned svm-parameters and caluclated feature-scalling
      */
     SVMConfig learn( std::vector<Cluster> &clusters, std::vector<Label> &labels);
     

     
     static void print_null(const char *s) {};
     
   };
  

} // end namespace sonar_image_feature_extractor

#endif // _DUMMYPROJECT_DUMMY_HPP_