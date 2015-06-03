#include "Classifier.hpp"

using namespace sonar_image_feature_extractor;

Classifier::Classifier(){
  
  model = 0;
}

Classifier::~Classifier(){
  delete model;
}

void Classifier::init(SVMConfig config){
  
  this->config = config;
  
  param.svm_type = config.svm_type;
  param.kernel_type = config.kernel_type;
  param.degree = config.kernel_degree;
  param.gamma = config.rbf_gamma;
  param.coef0 = config.coef0;
  
  param.cache_size = config.cache_size;
  param.eps = config.stopping_eps;
  param.C = config.C;
  param.nr_weight = config.weights.size();
  param.weight_label = config.weight_labels.data();
  param.weight = config.weights.data();
  param.shrinking = config.use_shrinking;
  param.probability = config.use_probability;
  
  if(! config.learn){
    
    model = svm_load_model( config.svm_path.c_str());
    
  }
  
  
}


bool Classifier::classify(Cluster &c){
  
  if(model){
  
    std::vector<svm_node> nodes = getNodes(c);
     
    double prediction = svm_predict(model, nodes.data());
    
    if(prediction > 0)
      return true;   
  
  } 
  
 
  return false;
}

bool Classifier::learn( std::vector<Cluster> &clusters, std::vector<Label> &labels){
  
  svm_problem prob;
  prob.l = clusters.size();
  
  std::vector<double> results;
  
  for(std::vector<Label>::iterator it = labels.begin(); it != labels.end(); it++){
    results.push_back(it->label_id);    
  }
  
  
  std::vector<std::vector<svm_node> >  all_nodes;
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++){
    all_nodes.push_back(  getNodes(*it) );
  } 
  
  svm_node **data = new svm_node*[all_nodes.size()];
  
  for( int i = 0; i < all_nodes.size(); i++){
    
    data[i] = all_nodes[i].data();    
  }
  
  prob.y = results.data();
  prob.x = data;
  
  model = svm_train(&prob, &param);
  
  svm_save_model( config.svm_path.c_str() , model);
  
  return false;
}

std::vector<svm_node> Classifier::getNodes( Cluster &c){
    std::vector<svm_node> nodes;
    
    svm_node node;
    node.index = 0;
    node.value = c.range_size;
    nodes.push_back(node);    
    
    node.index = 1;
    node.value = c.angle_size;
    nodes.push_back(node);
    
    node.index = 2;
    node.value = c.variance;
    nodes.push_back(node);
    
    node.index = 3;
    node.value = c.contrast;
    nodes.push_back(node);
        
    return nodes;
}

