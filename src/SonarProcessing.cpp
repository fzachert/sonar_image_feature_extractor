/* ----------------------------------------------------------------------------
 * SonarProcessing.cpp
 * written by Fabio Zachert, June 2015
 * University of Bremen
 * 
 * This class provides a featue-detector on sonar-scans
 * ----------------------------------------------------------------------------
*/


#include "SonarProcessing.hpp"
#include "ProcessingTools.hpp"
#include <base/logging.h>

using namespace cv;
using namespace sonar_image_feature_extractor;

static double angular_resolution = 0.5;
static double range_resolution = 0.008;
static double sin_angular_resolution = 0.00872653549;

void SonarProcessing::init( const SVMConfig &svm_conf){
  
  this->svm_conf = svm_conf;
  
  classifier.init( svm_conf);  
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
  
  process_points(clusters, input, threshold_mat);
  calculate_moments(clusters, input, threshold_mat);
  
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
  result.time = input.time;
  cv::Mat threshold_mat;
  base::Time total_start_time = base::Time::now();
  
  LOG_INFO_S << "Start processing";
  base::Time start = base::Time::now();
  std::vector<SonarPeak> peaks = process(input, debug, config, dd, threshold_mat);
  LOG_INFO_S << "Processing time: " << base::Time::now().toSeconds() - start.toSeconds();
  
  
  LOG_INFO_S << "Start clustering with " << peaks.size() << " peaks.";
  start = base::Time::now();  
  std::vector<Cluster> clusters = cluster(peaks, config, input, debug);
  LOG_INFO_S << "Clustering time: " << base::Time::now().toSeconds() - start.toSeconds();
  dd.time_clustering = base::Time::now().toSeconds() - start.toSeconds();
  
  start = base::Time::now();
  process_points(clusters, input, threshold_mat);
  calculate_moments(clusters, input, threshold_mat);
  dd.time_extraction = base::Time::now().toSeconds() - start.toSeconds();
  
  dd.cluster = clusters;
  
  start = base::Time::now();
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    
    //std::cout << "Start classification" << std::endl;
    start = base::Time::now();
    int label = classifier.classify(*it);
    //std::cout << "Classification time: " << base::Time::now().toSeconds() - start.toSeconds() << std::endl;  
    
    
    if( label != 0){
      
      Feature f;
      f.confidence = 1;
      f.position.x() = cos( (it->min_angle + it->max_angle) * 0.5 ) * it->min_range;
      f.position.y() = sin( (it->min_angle + it->max_angle) * 0.5 ) * it->min_range;
      f.size.x() = it->maxX - it->minX;
      f.size.y() = it->maxY - it->minY;
      f.range = it->min_range;
      f.angle_h = (it->min_angle + it->max_angle) * 0.5;
      
      f.desc.label = label;      
      
      if( f.size.norm() < config.feature_max_size){
	result.features.push_back(f);
      }
	
    }  
    
  }
  dd.time_classification = base::Time::now().toSeconds() - start.toSeconds();
  dd.time_total = base::Time::now().toSeconds() - total_start_time.toSeconds();
  
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
  
  base::Time start_time = base::Time::now();
  
  Mat sonar_mat;
  sonar_mat = Mat(input.number_of_beams, input.number_of_bins, CV_8U, (void*) input.data.data() );
  sonar_mat.copyTo(threshold_mat);
  
  smooth( threshold_mat, config.blur, config.smooth_mode);
  
  dd.time_preprocessing = base::Time::now().toSeconds() - start_time.toSeconds();
  
  if(config.debug_mode == SMOOTHING){
    debug.data.assign(threshold_mat.datastart, threshold_mat.dataend);
  }
    
  start_time = base::Time::now();
  
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
  dd.time_segmentation = base::Time::now().toSeconds() - start_time.toSeconds();
  
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

void SonarProcessing::print_detected_features( const SonarFeatures &features, base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config){
  
  debug = input;
  cv::Mat sonar_mat = cv::Mat(debug.number_of_beams, debug.number_of_bins, CV_8U, (void*) debug.data.data());
  double range_scale = config.sonar_max_range / debug.number_of_bins;
  
  
  for(std::vector<Feature>::const_iterator it = features.features.begin(); it != features.features.end(); it++){
    
    double size = it->size.norm();
    double angular_size = std::asin( (size * 0.5) / it->range); 
    
    int min_range_id = it->range / range_scale;
    int max_range_id = (it->range + size) / range_scale;
    int avg_range_id = (it->range + (size * 0.5)) / range_scale;
    int avg_angle_id = debug.number_of_beams + std::fabs(it->angle_h - input.start_bearing.rad) / input.angular_resolution.rad;
    int min_angle_id = avg_angle_id + (angular_size / input.angular_resolution.rad);
    int max_angle_id = avg_angle_id - (angular_size / input.angular_resolution.rad);

/*    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Print feature with : " << std::endl;
    std::cout << "range: " << it->range << ", size: " << it->size.transpose() << " and angle: " << it->angle_h << std::endl;
    std::cout << "min_range_id: " << min_range_id << " max_range_id: " << max_range_id << " avg_range_id: " << avg_angle_id << std::endl;
    std::cout << "min_angle_id: " << min_angle_id << " max_angle_id: " << max_angle_id << " avg_angle_id: " << avg_angle_id << std::endl;
 */   
    for(int range_id = min_range_id; range_id < max_range_id; range_id++){
      sonar_mat.at<uint8_t>(avg_angle_id, range_id) = 255;      
    }
    
    for(int angle_id = min_angle_id; angle_id < max_angle_id; angle_id++){
      sonar_mat.at<uint8_t>(angle_id, avg_range_id) = 255;      
    }    
    
    
  }
  
}

