rock_library(sonar_image_feature_extractor
    SOURCES Detector.cpp Filter.cpp SonarProcessing.cpp Classifier.cpp libsvm/svm.cpp
    HEADERS Detector.hpp DetectorTypes.hpp ClassificationTypes.hpp Filter.hpp SonarProcessing.hpp Classifier.hpp ProcessingTools.hpp libsvm/svm.h
    DEPS_PKGCONFIG base-types opencv frame_helper machine_learning dsp_acoustics)



