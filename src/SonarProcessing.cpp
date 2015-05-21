#include "SonarProcessing.hpp"

using namespace cv;
using namespace sonar_image_feature_extractor;


void SonarProcessing::init(){
  
}

SonarFeatures SonarProcessing::detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config)
{
  
  SonarFeatures result;
  
  std::vector<SonarPeak> peaks = process(input, debug, config);
  
  std::vector<Cluster> clusters = cluster(peaks, config);
  
  for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    
    if(classifier.classify(*it, config)){
      //TODO ad feature to result
    }
    
  }
  
  return result;
} 

std::vector<SonarPeak> SonarProcessing::process(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config)
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
  sonar_mat = Mat(input.number_of_bins, input.number_of_beams, CV_8U, (void*) input.data.data() );
  
  int blur = config.blur;
  if(!(blur%2)){
    blur += 1;
  }  
  //Smoothing
  if(config.smooth_mode == AVG){
    cv::blur( sonar_mat, sonar_mat, Size(blur, 1), Point(-1,-1));
  }else if(config.smooth_mode == GAUSSIAN){
    cv::GaussianBlur( sonar_mat, sonar_mat, Size(blur, 1) ,0);
  }else if(config.smooth_mode == MEDIAN){
    cv::medianBlur(sonar_mat, sonar_mat, blur);
  }
  
  if(config.debug_mode == SMOOTHING){
    debug.data.assign(sonar_mat.datastart, sonar_mat.dataend);
  }
    
  double min,max;
  cv::minMaxLoc( sonar_mat, &min, &max);
  
  //Thresholding
  if(config.threshold_mode == ABSOLUTE)
    threshold(sonar_mat, sonar_mat, config.threshold * max, 255, CV_THRESH_BINARY);
  else if(config.threshold_mode == ADAPTIVE_MEAN)
    adaptiveThreshold(sonar_mat, sonar_mat, 255, ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 15 ,config.threshold * max);
  else if(config.threshold_mode == ADAPTIVE_GAUSSIAN)
    adaptiveThreshold(sonar_mat, sonar_mat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 15, config.threshold * max);
  else if(config.threshold_mode == OTSU)
    threshold(sonar_mat, sonar_mat, 0 , 255 , CV_THRESH_BINARY + CV_THRESH_OTSU);
  
  if(config.debug_mode == THRESHOLD){
    debug.data.assign(sonar_mat.datastart, sonar_mat.dataend);
  }  
  
  std::vector<cv::Point2i> locations;
  
  
  if( cv::countNonZero( sonar_mat) > 0)
    cv::findNonZero(sonar_mat, locations);
  
  peaks.reserve(locations.size());
  
  double range_scale = config.sonar_max_range / input.number_of_bins;  
  
  for(std::vector<cv::Point2i>::iterator it = locations.begin(); it != locations.end(); it++){
    
    SonarPeak peak;
    peak.angle = (input.start_bearing + (input.angular_resolution * it->y) ).rad;
    peak.range = range_scale * it->x;
    peak.pos.x() = cos(peak.angle) * peak.range;
    peak.pos.y() = sin(peak.angle) * peak.range;
    
    peaks.push_back(peak);
  }
  
  
/*    
  for( int i = 0; i < input.number_of_beams; i++){
    base::samples::SonarBeam beam;
        
    base::Angle angle = base::Angle::fromRad( input.start_bearing.getRad() - (i * input.angular_resolution.getRad())  );
    //std::cout << "Angle: " << angle << " " << input.start_bearing.getRad() << " " << input.angular_resolution.getRad() <<  std::endl;
    
    input.getSonarBeam( angle,  beam);
    base::samples::SonarBeam filtered(beam);
    base::samples::SonarBeam derivative(beam);
    
    //filtered.beam[ (int)( filtered.beam.size() * 0.6)  ] = 255;
    
    dsp::movingAverageFilterSym<std::vector<uint8_t>::iterator, std::vector<uint8_t>::iterator, uint8_t>(beam.beam.begin(), beam.beam.end(), filtered.beam.begin() , config.blur);
    //dsp::flipSignal<std::vector<uint8_t>::iterator, std::vector<uint8_t>::iterator>(beam.beam.begin(), beam.beam.end(), fifilltered.beam.begin());
    dsp::derivativeSignal<std::vector<uint8_t>::iterator, std::vector<uint8_t>::iterator>(filtered.beam.begin(), filtered.beam.end(), derivative.beam.begin());
    
    if(config.debug_mode == SMOOTHING){
      debug.addSonarBeam(filtered, false);
    }
    if(config.debug_mode == SOBEL){
      debug.addSonarBeam(derivative, false);
    }
    
  }*/ 
  
return peaks;  
  
}

std::vector<Cluster> SonarProcessing::cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config)
{
std::vector<Cluster> cluster;



return cluster;  
}
