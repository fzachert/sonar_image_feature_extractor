/* ----------------------------------------------------------------------------
 * ClassificationTypes.hpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This file provides types for the svm-classifier
 * ----------------------------------------------------------------------------
*/

#ifndef _SONARIMAGEFEATURE_CLASSTYPES_HPP_
#define _SONARIMAGEFEATURE_CLASSTYPES_HPP_

#include "DetectorTypes.hpp"
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Time.hpp>
#include <base/samples/SonarScan.hpp>
#include <vector>
#include <math.h>
#include "libsvm/svm.h"

namespace sonar_image_feature_extractor
{
 
  /**
   * A linear scale-factor
   * Need, to scale the svm-input-features in a range betwwen 0 and 1
   */
  struct ScaleFactor{
    
    double offset;
    double scalling;    
  };  
  
  /**
   * Configuration of the svm
   * For training and callsification
   */
  struct SVMConfig{
    
    //General properties for learning and training
    std::string svm_path; //Filepath for loading and saving the svm
    
    SVMType svm_type;
    KERNEL_TYPE kernel_type; //Right now, only RBF is full-supported
    int kernel_degree;
    double rbf_gamma; //Gamma-variance of the rbf-kernel
    double coef0; //Offset-value of the rbf and polynomial-kernel
    bool use_moments; //Use the cluster-moments as features
    
    bool learn; //if classiffier is used for training
    
    //Training parameters    
    double cache_size;
    double stopping_eps;
    double C;
    double nu;
    std::vector<double> weights;
    std::vector<int> weight_labels;
    int use_shrinking;
    int use_probability;   
    
    //Cross-validation training parameters
    bool cross_validation; //Use cross-validation, to find the best C and rbf_gamma value
    int number_of_folds;
    //In the cross-validation, diffeerent C and gamma value are used
    //The values are used as the power of 2, in the range, defined below
    int start_gamma_exp;
    int end_gamma_exp;
    int start_C_exp;
    int end_C_exp;
    
    //Calculated featre-scalling
    std::vector<ScaleFactor> scales;
   
  };
  
  /**
   * Label for the svm-training
   */
  struct Label{
    base::Vector2d pos;
    double label_id;    
  };

  /**
   * Labeled cluster, as input for the svm-training
   */
  struct LabeledCluster{
    Cluster cluster;
    Label label;    
  };  
  

} // end namespace sonar_image_feature_extractor

#endif