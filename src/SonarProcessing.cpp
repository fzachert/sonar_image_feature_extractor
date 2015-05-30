#include "SonarProcessing.hpp"
#include "ProcessingTools.hpp"

using namespace cv;
using namespace sonar_image_feature_extractor;

static double angular_resolution = 0.5;
static double range_resolution = 0.008;
static double sin_angular_resolution = 0.00872653549;

void SonarProcessing::init(){
  
}

SonarFeatures SonarProcessing::detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd)
{
  
  SonarFeatures result;
  
  std::vector<SonarPeak> peaks = process(input, debug, config, dd);
  
  
  std::cout << "Start clustering with " << peaks.size() << " peaks." << std::endl;
  base::Time start = base::Time::now();
  
  std::vector<Cluster> clusters = cluster(peaks, config, input);
  std::cout << "Clustering time: " << base::Time::now().toSeconds() - start.toSeconds() << std::endl;
  
  if(config.debug_mode == FEATURES)
    debug = input;
  
  dd.cluster = clusters;
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    
    if(classifier.classify(*it, config)){
      //TODO ad feature to result
    }
    
  }
  
  return result;
} 

std::vector<SonarPeak> SonarProcessing::process(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config, DebugData &dd)
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
  
  smooth( sonar_mat, config.blur, config.smooth_mode);
  
  if(config.debug_mode == SMOOTHING){
    debug.data.assign(sonar_mat.datastart, sonar_mat.dataend);
  }
    
  
  threshold( sonar_mat, config.threshold_mode, config.threshold, config.adaptive_threshold_neighborhood);  
  
  if(config.debug_mode == THRESHOLD){
    debug.data.assign(sonar_mat.datastart, sonar_mat.dataend);
  }  
  
  std::vector<cv::Point2i> locations;  
  
  if( cv::countNonZero( sonar_mat) > 0)
    cv::findNonZero(sonar_mat, locations);
  
  peaks.reserve(locations.size());
  
  double range_scale = config.sonar_max_range / input.number_of_bins;  
  dd.points.points.clear();
  
  for(std::vector<cv::Point2i>::iterator it = locations.begin(); it != locations.end(); it++){
    
    SonarPeak peak;
    peak.angle = ((input.angular_resolution * it->y) - input.start_bearing ).rad;
    peak.range = range_scale * it->x;
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

std::vector<Cluster> SonarProcessing::cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config, base::samples::SonarScan &sonar_scan)
{
  std::vector<Cluster> analysedCluster;
  std::list<SonarPeak*> pPeaks;

  for(int i = 0; i < peaks.size(); i++){
    
    pPeaks.push_back( &peaks[i]);    
  }
  
  Mat debug_mat;
  
  machine_learning::DBScan<SonarPeak> *scan;
  
  if(config.distance_mode == EUKLIDIAN)
    scan = new machine_learning::DBScan<SonarPeak>(&pPeaks, config.cluster_min_size, config.cluster_noise, false, 1.0, distance);
  else if(config.distance_mode == MAHALANOBIS)
    scan = new machine_learning::DBScan<SonarPeak>(&pPeaks, config.cluster_min_size, config.cluster_noise, false, 1.0, mahalanobis_distance);
    
  std::map< SonarPeak*, int> clusteredPoints = scan->scan();
  
  analysedCluster.resize(scan->getClusterCount());
  
  double range_scale = config.sonar_max_range / sonar_scan.number_of_bins;
  uint8_t color_step = 255 / scan->getClusterCount();
  if(config.debug_mode == FEATURES){
    
    debug_mat = Mat(sonar_scan.number_of_beams, sonar_scan.number_of_bins, CV_8U, (void*) sonar_scan.data.data() );
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
      
      
      if(config.debug_mode == FEATURES){
	int angle_index =  abs(int((p.angle-sonar_scan.start_bearing.rad)/sonar_scan.angular_resolution.rad));
	int range_index = p.range / range_scale;
	debug_mat.at<uint8_t>(angle_index, range_index) = (id +1) * color_step;
	//std::cout << "Angle idx: " << angle_index << " Range idx: " << range_index << std::endl;
      }
      
      
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


