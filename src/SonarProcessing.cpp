#include "SonarProcessing.hpp"
#include "ProcessingTools.hpp"

using namespace cv;
using namespace sonar_image_feature_extractor;

static double angular_resolution = 0.5;
static double range_resolution = 0.008;
static double sin_angular_resolution = 0.00872653549;

void SonarProcessing::init(){
  
}


std::vector<LabeledCluster> SonarProcessing::label_cluster(base::samples::SonarScan &input, DetectorConfig &config, DebugData &dd, Label label)
{
  std::vector<LabeledCluster> result;
  LabeledCluster lc;
  lc.label = label;
  
  cv::Mat threshold_mat;
  
  config.debug_mode = NO_DEBUG;
  
  std::vector<SonarPeak> peaks = process(input, input, config, dd, threshold_mat);  
    
  std::vector<Cluster> clusters = cluster(peaks, config, input, input);
  
  process_points(clusters, input, threshold_mat, config);
  
  dd.cluster = clusters;
  
  double min_dist = std::numeric_limits<double>::max();
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    
    if(label.label_id > 0.0){
      double dist =  (it->middle - label.pos).norm();
    
      if( dist < min_dist){
	lc.cluster = *it;
	min_dist = dist;
      }
      
    }else{
      lc.cluster = *it;
      result.push_back(lc);
    }
    
  }
  
  if(clusters.size() > 0 && label.label_id > 0.0){
    result.push_back(lc);
  }
  
  return result;
} 


SonarFeatures SonarProcessing::detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd)
{
  
  SonarFeatures result;
  cv::Mat threshold_mat;
  
  std::vector<SonarPeak> peaks = process(input, debug, config, dd, threshold_mat);
  
  
  std::cout << "Start clustering with " << peaks.size() << " peaks." << std::endl;
  base::Time start = base::Time::now();
  
  std::vector<Cluster> clusters = cluster(peaks, config, input, debug);
  std::cout << "Clustering time: " << base::Time::now().toSeconds() - start.toSeconds() << std::endl;
  
  process_points(clusters, input, threshold_mat, config);
  
  dd.cluster = clusters;
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    
//     if(classifier.classify(*it, config)){
//       //TODO ad feature to result
//     }
    
  }
  
  return result;
} 

std::vector<SonarPeak> SonarProcessing::process(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd, cv::Mat &threshold_mat)
{
  SonarFeatures feat;
  std::vector<SonarPeak> peaks;
  
  if(input.memory_layout_column)
  {
    input.toggleMemoryLayout();
  }

  if(config.debug_mode != NO_DEBUG)
  {
    debug = input;
  }
  
  Mat sonar_mat;
  sonar_mat = Mat(input.number_of_beams, input.number_of_bins, CV_8U, (void*) input.data.data() );
  sonar_mat.copyTo(threshold_mat);
  
  smooth( threshold_mat, config.blur, config.smooth_mode);
  
  if(config.debug_mode == SMOOTHING){
    debug.data.assign(threshold_mat.datastart, threshold_mat.dataend);
  }
    
  
  threshold( threshold_mat, config.threshold_mode, config.threshold, config.adaptive_threshold_neighborhood);  
  
  if(config.debug_mode == THRESHOLD){
    debug.data.assign(threshold_mat.datastart, threshold_mat.dataend);
  }  
  
  std::vector<cv::Point2i> locations;  
  
  if( cv::countNonZero( threshold_mat) > 0)
    cv::findNonZero(threshold_mat, locations);
  
  peaks.reserve(locations.size());
  
  double range_scale = config.sonar_max_range / input.number_of_bins;  
  dd.points.points.clear();
  
  for(std::vector<cv::Point2i>::iterator it = locations.begin(); it != locations.end(); it++){
    
    SonarPeak peak;
    
    if(config.gemini)
      peak.angle = getRadGemini(it->y);
    else    
      peak.angle = ((input.angular_resolution * it->y) - input.start_bearing ).rad;
    
    peak.range = range_scale * it->x;  
    
    peak.range_id = it->x;
    peak.angle_id = it->y;
    peak.pos.x() = cos(peak.angle) * peak.range;
    peak.pos.y() = sin(peak.angle) * peak.range;
    
    base::Vector3d p;
    p.x() = peak.pos.x();
    p.y() = peak.pos.y();
    p.z() = 0.0;
    dd.points.points.push_back(p);
    
    //std::cout << "point: " << it->x << " " << it->y << std::endl;
    //std::cout << "Angle: " << peak.angle << " range: " << peak.range << "Pos: " << peak.pos.x() << " " << peak.pos.y() << std::endl;
    
    peaks.push_back(peak);
  }
  
  
return peaks;  
  
}

std::vector<Cluster> SonarProcessing::cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config, base::samples::SonarScan &sonar_scan, base::samples::SonarScan &debug_scan)
{
  std::vector<Cluster> analysedCluster;
  std::list<SonarPeak*> pPeaks;

  for(int i = 0; i < peaks.size(); i++){
    
    pPeaks.push_back( &peaks[i]);    
  }
  
  Mat debug_mat;
  
  machine_learning::DBScan<SonarPeak> *scan;
  
  if(config.distance_mode == EUKLIDIAN)
    scan = new machine_learning::DBScan<SonarPeak>(&pPeaks, 1, config.cluster_noise, false, 1.0, distance, true);
  else if(config.distance_mode == MAHALANOBIS)
    scan = new machine_learning::DBScan<SonarPeak>(&pPeaks, 1, config.cluster_noise, false, 1.0, mahalanobis_distance, true);
    
  //std::cout << "Before scan" << std::endl;
  std::map< SonarPeak*, int> clusteredPoints = scan->scan();
  //std::cout << "After scan" << std::endl;
  
  analysedCluster.resize(scan->getClusterCount());
  
  double range_scale = config.sonar_max_range / sonar_scan.number_of_bins;
  uint8_t color_step = 255 / scan->getClusterCount();
  if(config.debug_mode == FEATURES){
    
    debug_mat = Mat(sonar_scan.number_of_beams, sonar_scan.number_of_bins, CV_8U, (void*) debug_scan.data.data() );
    debug_mat.setTo(0);   
  
  }
  
  for(std::map< SonarPeak* , int> ::iterator it = clusteredPoints.begin(); it != clusteredPoints.end(); it++){
    
    int id = it->second;
    
    if(id >= 0){
      
      SonarPeak p = *(it->first);
      
      analysedCluster[id].number_of_points++;
      
      if( p.pos.x() < analysedCluster[id].minX)
	analysedCluster[id].minX = p.pos.x();
      if( p.pos.x() > analysedCluster[id].maxX)
	analysedCluster[id].maxX = p.pos.x();
      if( p.pos.y() < analysedCluster[id].minY)
	analysedCluster[id].minY = p.pos.y();
      if( p.pos.y() > analysedCluster[id].maxY)
	analysedCluster[id].maxY = p.pos.y();
      
      if( p.range < analysedCluster[id].min_range)
	analysedCluster[id].min_range = p.range;
      if( p.range > analysedCluster[id].max_range)
	analysedCluster[id].max_range = p.range;
      if( p.angle < analysedCluster[id].min_angle)
	analysedCluster[id].min_angle = p.angle;
      if( p.angle > analysedCluster[id].max_angle)
	analysedCluster[id].max_angle = p.angle;
      
      if( p.range_id < analysedCluster[id].min_range_id)
	analysedCluster[id].min_range_id = p.range_id;
      if( p.range_id > analysedCluster[id].max_range_id)
	analysedCluster[id].max_range_id = p.range_id;
      if( p.angle_id < analysedCluster[id].min_angle_id)
	analysedCluster[id].min_angle_id = p.angle_id;
      if( p.angle_id > analysedCluster[id].max_angle_id)
	analysedCluster[id].max_angle_id = p.angle_id;      
      
	analysedCluster[id].sum_pos += p.pos;
      
      if(config.debug_mode == FEATURES){
	debug_mat.at<uint8_t>(p.angle_id, p.range_id) = (id +1) * color_step;
	//std::cout << "Angle idx: " << angle_index << " Range idx: " << range_index << std::endl;
      }
      
      
    }  
        
  }
  
  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end();){

    if(it->number_of_points < config.cluster_min_size || it->number_of_points > config.cluster_max_size){
      it = analysedCluster.erase(it);
      
    }else{

      it->middle.x() = (it->maxX + it->minX) * 0.5;
      it->middle.y() = (it->maxY + it->minY) * 0.5;
      it->angle_size = (it->max_angle - it->min_angle);
      it->range_size = (it->max_range - it->min_range);
      
      it->avg_pos = it->sum_pos * (1.0 / it->number_of_points);
      
      it++;
    }
  }
  
  
  return analysedCluster;  
}


void SonarProcessing::process_points(std::vector<Cluster> &analysedCluster, base::samples::SonarScan &sonar_scan, cv::Mat threshold_mat, const DetectorConfig &config){
  
  //std::cout << "Threshold_mat " << threshold_mat.size().width << " " << threshold_mat.size().height << std::endl; 
  
  
  cv::Mat sonar_mat = cv::Mat(sonar_scan.number_of_beams, sonar_scan.number_of_bins, CV_8U, (void*) sonar_scan.data.data());
  
  double temp_sum = 0.0;
  int temp_points = 0;
  
  //Calculate avg-singnal in the cluster-areas  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sum = 0.0;
      temp_points = 0;
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
// 	std::cout << "angle_id: " << angle_index << " range_id: " << range_index << std::endl;
// 	std::cout << "angle: " << angle << " range: " << range << std::endl;
// 	std::cout << "min_angle: " << it->min_angle << " max_angle: " << it->max_angle << std::endl;
	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){

	  sum += temp_sum + sonar_mat.at<uint8_t>(angle, range);
	  num_points += temp_points + 1;
	  temp_sum = 0.0;
	  temp_points = 0;
	  
	}else{
	  temp_sum += sonar_mat.at<uint8_t>(angle, range);
	  temp_points++;
	}
      
      }
    
    } 
    
    if(num_points > 0)
      it->avg_signal= sum / num_points;    
    else
      it->avg_signal= 0;
    
  }
  
  
  //Calculate variance in cluster-areas
  temp_sum = 0.0;
  temp_points = 0;

  
  //Calculate avg-singnal in the cluster-areas  
  for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    double avg = it->avg_signal;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      temp_sum = 0.0;
      temp_points = 0;
      
      for(int range = it->min_range_id; range <= it->max_range_id; range++){

	
	if(threshold_mat.at<uint8_t>(angle, range) > 0){
	  
	  sum += temp_sum + std::pow(sonar_mat.at<uint8_t>(angle, range) - avg  , 2.0 );
	  num_points += temp_points + 1;
	  temp_sum = 0.0;
	  temp_points = 0;
	  
	}else{
	  temp_sum += std::pow(sonar_mat.at<uint8_t>(angle, range) - avg, 2.0)  ;
	  temp_points++;
	}
      
      }
    
    } 
    
    if(num_points > 0){
      it->variance = sum / num_points;
      it->variation_coefficient = std::sqrt( it->variance) / 127.5; 
    }else{
      it->variance = 0;
      it->variation_coefficient = 0;
    }
  }  
  
  
  
    //Calculate contrast
    for(std::vector<Cluster>::iterator it = analysedCluster.begin(); it != analysedCluster.end(); it++){
    
    double sum = 0.0;
    int num_points = 0;
    
    for(int angle = it->min_angle_id; angle <= it->max_angle_id; angle++){
      
      for(int range = it->min_range_id - 10; range <= it->min_range_id; range++){
	
	if(range > 0){
	  sum += sonar_mat.at<uint8_t>(angle, range);
	  num_points++;	  
	}
      
      }
      
      for(int range = it->max_range_id; range <= it->max_range_id + 10; range++){
	
	if(range < sonar_scan.number_of_bins){
	  sum += sonar_mat.at<uint8_t>(angle, range);
	  num_points++;	  
	}
      
      }      
      
    
    } 
    
    double avg_env = sum / num_points;
    
    if(avg_env > 0)
      it->contrast = it->avg_signal / avg_env;
    else
      it->contrast = 0;
    
    
  } 
  
  
  
}

double SonarProcessing::mahalanobis_distance(SonarPeak *p1, SonarPeak *p2)
{
  double cross_resolution = (std::fabs( sin_angular_resolution * p1->range) 
	+ std::fabs(sin_angular_resolution * p2->range)) * 0.5;
	
  base::Matrix2d cov = base::Matrix2d::Zero();
  cov(0,0) = 1.0;
  cov(1,1) = cross_resolution / range_resolution;
  Eigen::Rotation2D<double> rot( (p1->angle + p2->angle) * 0.5  );
  
  cov = rot * cov;
  
  return std::sqrt(  (p1->pos - p2->pos).transpose() * cov.inverse() * (p1->pos - p2->pos) ) ;   
}


double SonarProcessing::distance(SonarPeak *p1, SonarPeak *p2)
{
  return (p1->pos - p2->pos).norm();
}


