#include "SonarProcessing.hpp"

using namespace sonar_image_feature_extractor;


void SonarProcessing::init(){
  
}

SonarFeatures SonarProcessing::detect(base::samples::SonarScan &input, base::samples::SonarScan &debug, const DetectorConfig &config)
{
  SonarFeatures feat;
  std::vector<SonarPeak> peaks;
  input.memory_layout_column = false;
  
  if(input.memory_layout_column)
  {
    input.toggleMemoryLayout();
  }

  if(config.debug_mode != NO_DEBUG)
  {
    debug = input;
  }
  

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
    
  } 
  
  debug.memory_layout_column = true;
  
return feat;  
  
}

SonarFeatures SonarProcessing::cluster(std::vector<SonarPeak> &peaks, const DetectorConfig &config)
{
SonarFeatures feat;






  
return feat;  
}
