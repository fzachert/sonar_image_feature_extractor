rock_library(sonar_image_feature_extractor
    SOURCES Detector.cpp Filter.cpp SonarProcessing.cpp
    HEADERS Detector.hpp DetectorTypes.hpp Filter.hpp SonarProcessing.hpp
    DEPS_PKGCONFIG base-types opencv frame_helper machine_learning dsp_acoustics)



