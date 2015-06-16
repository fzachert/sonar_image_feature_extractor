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


int Classifier::classify(Cluster &c){
  
  if(model){
  
    std::vector<svm_node> nodes = getNodes(c);
    scale(nodes);
    
    double prediction = svm_predict(model, nodes.data());
    
    return (int) round(prediction);  
  
  } 
 
  return 0;
}

SVMConfig Classifier::learn( std::vector<Cluster> &clusters, std::vector<Label> &labels){
  
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
  
  calculateScalling( all_nodes);
  scaleAllNodes( all_nodes);
  
  svm_node **data = new svm_node*[all_nodes.size()];
  
  for( int i = 0; i < all_nodes.size(); i++){
    
    data[i] = all_nodes[i].data();    
  }
  
  prob.y = results.data();
  prob.x = data;
  
  const char *message = svm_check_parameter(&prob, &param); 
  
  if(message == NULL){
    
    if(config.cross_validation){
      
      cross_validate_params( prob);
      
    }
    
    std::cout << "Start training with C=" << param.C << " and gamma=" << param.gamma << std::endl;
    
    model = svm_train(&prob, &param);
    
    validate_model(clusters, labels);
    
    cross_validate_result( prob);
    
    if(svm_save_model( config.svm_path.c_str() , model) == -1)
      std::cout  << "Could not save svm" << std::endl;
    
  }else{
    
    std::cout << "Parameter error: " << message << std::endl;
  }
  
  return config;
}


void Classifier::validate_model(svm_problem &prob){
 
  int error_count = 0;
  
  std::vector<int> false_labels, true_labels;  
  
  for( int i = 0; i < prob.l; i++){
    
    double prediction = svm_predict( model, prob.x[i] );
    int prediction_label = (int)round(prediction);    
    
    if( prob.y[i] >= true_labels.size() ){
      true_labels.resize( prob.y[i], 0);
      false_labels.resize( prob.y[i], 0);
    }
    
    
    if(  std::fabs(prediction - prob.y[i]) > 0.1 ){
      error_count++;
      
      false_labels[ prob.y[i] - 1]++;      
    }
    else{
      true_labels[ prob.y[i] - 1]++;
    }
      
  }
  
  std::cout << error_count << " errors in " << prob.l << " samples. " << std::endl;
  
  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << " Label | positives | negatives " << std::endl;
  
  for( int i = 0; i < true_labels.size(); i++){
    std::cout << " " << i +1 << "        " << true_labels[i] << "         " << false_labels[i] << std::endl;
    
  }
  
  std::cout << "----------------------------------------------------" << std::endl;
  
}

void Classifier::validate_model( std::vector<Cluster> &clusters, std::vector<Label> &labels){
 
  int error_count = 0;
  std::vector<svm_node> nodes;
  
  std::vector<int> false_labels, true_labels; 
  std::vector< std::vector<int> > predictions(3, std::vector<int>(3, 0)) ;
  
  for( int i = 0; i < clusters.size(); i++){
    
    nodes = getNodes(clusters[i]);
    scale(nodes);
    
    double prediction = svm_predict( model, nodes.data() );
    int prediction_label = (int)round(prediction);    
    
    if( labels[i].label_id >= true_labels.size() ){
      true_labels.resize( labels[i].label_id, 0);
      false_labels.resize( labels[i].label_id, 0);
      
      predictions.resize( labels[i].label_id, std::vector<int>( labels[i].label_id, 0)  );
    }
    
    if( prediction >= predictions[ labels[i].label_id -1].size() ){
      
      predictions[ labels[i].label_id - 1].resize( prediction, 0);
    }
    
    predictions[ labels[i].label_id -1][ prediction -1]++;
    
    if(  std::fabs(prediction - labels[i].label_id) > 0.1 ){
      error_count++;
      
      false_labels[ labels[i].label_id  -1 ]++;      
    }
    else{
      true_labels[ labels[i].label_id - 1]++;
    }
      
  }
  
  std::cout << error_count << " errors in " << clusters.size() << " samples. " << std::endl;
  
  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << " Label | positives | negatives " << std::endl;
  
  for( int i = 0; i < true_labels.size(); i++){
    std::cout << " " << i +1 << "        " << true_labels[i] << "         " << false_labels[i] << std::endl;
    
  }
  
  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << "Confusion matrix of learning-error" << std::endl;
  
  std::cout << "     ";
  for(int i = 0 ; i < predictions.size(); i++){
    std::cout << "| " << i +1 << "   " ;
  }
  std::cout << std::endl;
  
  for(int i = 0; i < predictions.size(); i++){
    std::cout << i + 1 << "   " ; 
    for(int j = 0; j < predictions[i].size(); j++){
        std::cout << " " << predictions[i][j] << "  ";   
    }
    std::cout << std::endl;
  }
  
} 


void Classifier::cross_validate_result(svm_problem &prob){
  
    int total_error = 0;
    double *target = new double[prob.l];	
    std::vector< std::vector<int> > predictions(3, std::vector<int>(3, 0));
    
    svm_set_print_string_function( print_null) ;
    svm_cross_validation(&prob, &param, config.number_of_folds, target );
	
    for(int i=0;i<prob.l;i++)
    {

      if( prob.y[i] >= predictions.size() ){
	
	predictions.resize( prob.y[i], std::vector<int>( prob.y[i], 0)  );
      }
      
      if( target[i] >= predictions[ prob.y[i] -1].size() ){
	
	predictions[ prob.y[i] - 1].resize( target[i], 0);
      }
      
      predictions[ prob.y[i] -1][ target[i] -1]++;      
      
	
    }

  std::cout << "----------------------------------------------------" << std::endl;
  std::cout << "Confusion matrix of cross-validation" << std::endl;
  
  std::cout << "     ";
  for(int i = 0 ; i < predictions.size(); i++){
    std::cout << "| " << i +1 << "   " ;
  }
  std::cout << std::endl;
  
  for(int i = 0; i < predictions.size(); i++){
    std::cout << i + 1 << "   " ; 
    for(int j = 0; j < predictions[i].size(); j++){
        std::cout << " " << predictions[i][j] << "  ";   
    }
    std::cout << std::endl;
  }    
    
    
 
}


void Classifier::cross_validate_params(svm_problem &prob){
  
  std::cout << "Cross validation" << std::endl;
  
  svm_set_print_string_function( print_null) ;
  
  double best_gamma, best_C;
  int min_error = prob.l+1;
  double *target = new double[prob.l];
  
  for( int g = config.start_gamma_exp; g <= config.end_gamma_exp; g++){
    
    for( int c = config.start_C_exp; c <= config.end_C_exp; c++){
	
	int total_error = 0;	

	param.gamma = std::pow(2.0,g);	
	
	if( param.svm_type == C_SVC)	
	  param.C = std::pow(2.0,c);
	
	if( param.svm_type == NU_SVC)
	  param.nu = std::pow(2.0, c);
	
	svm_cross_validation(&prob, &param, config.number_of_folds, target );
	
	for(int i=0;i<prob.l;i++)
	{
	    if( std::fabs(target[i] - prob.y[i]) > 0.1  )
	      total_error++;
	
	}
	
	std::cout << "gamma: " << param.gamma << " C: " << param.C << " Error-rate: " << ((double)total_error) / ((double)prob.l) <<std::endl;
	
	if(total_error < min_error){
	  min_error = total_error;
	  best_gamma = param.gamma;
	  best_C = param.C;
	}	
      
    }
    
  }
  
  param.gamma = best_gamma;
  
  if( param.svm_type == C_SVC)
    param.C = best_C;
  if( param.svm_type == NU_SVC)
    param.nu = best_C;
  
  config.rbf_gamma = best_gamma;
  config.C = best_C;
  
  svm_set_print_string_function( NULL);
  
}


std::vector<svm_node> Classifier::getNodes( Cluster &c){
    std::vector<svm_node> nodes;
    
    svm_node node;
    node.index = 1;
    node.value = c.range_size;
    nodes.push_back(node);    
    
    node.index = 2;
    node.value = c.angle_size;
    nodes.push_back(node);
    
     node.index = 3;
     node.value = c.variation_coefficient;
     nodes.push_back(node);
     
     node.index = 4;
     node.value = c.contrast;
     nodes.push_back(node);
     
     node.index = 5;
     node.value = c.min_range;
     nodes.push_back(node);
     
     if(config.use_moments){

       for( int i = 1; i < c.moments.moments.size(); i++){
	 
	 node.index = 6 + i;
	 node.value = c.moments.moments[i].value;
	 nodes.push_back(node);
	 
       }
       
     }     
    
    node.index = -1;
    node.value = 0;
    nodes.push_back(node);
        
    return nodes;
}

void Classifier::calculateScalling( std::vector<std::vector<svm_node> > &nodes){
  
  std::vector<double> mins, maxs;
  config.scales.clear();
  
  for(std::vector< std::vector<svm_node> >::iterator it = nodes.begin(); it != nodes.end(); it++){
   
    for( std::vector< svm_node>::iterator it_node = it->begin(); it_node != it->end(); it_node++){
      
      if( it_node->index != -1){
      
	if( mins.size() < it_node->index || maxs.size() < it_node->index){
	  mins.resize( it_node->index, std::numeric_limits<double>::max());
	  maxs.resize( it_node->index, -std::numeric_limits<double>::max());      
	}
	  
	if( it_node->value < mins[ it_node->index -1]){
	  mins[ it_node->index -1] = it_node->value;
	}
	
	if( it_node->value > maxs[ it_node->index -1]){
	  maxs[ it_node->index -1] = it_node->value;
	}      
	
      }
      
    }
  
  }
  
  for( int i = 0; i < mins.size() && i < maxs.size(); i++){
    
    ScaleFactor sf;
    sf.offset = - mins[i];
    sf.scalling = 1.0 / ( maxs[i] - mins[i] );
    
    std::cout << "Min: " << mins[i] << " Max: " << maxs[i] << " offset: " << sf.offset
      << " scalling: " << sf.scalling << std::endl;
    
    config.scales.push_back( sf );
  }
  
}


void Classifier::scale( std::vector< svm_node> &nodes){
  
  if( config.scales.size() < nodes.size() - 1)
    return;
  
  for(std::vector< svm_node>::iterator it = nodes.begin(); it != nodes.end(); it++){
      
    if(it->index != -1){
    
      it->value = it->value * config.scales[ it->index -1 ].scalling + config.scales[ it->index -1].offset; 
    }
     
  }
  
}

void Classifier::scaleAllNodes( std::vector< std::vector< svm_node> > &nodes){
  
  for( std::vector< std::vector< svm_node> >::iterator it = nodes.begin(); it != nodes.end(); it++){
  
    scale( *it); 
    
  }
}

