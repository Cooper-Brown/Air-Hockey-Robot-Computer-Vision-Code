#ifndef GAME_STATE_TABLE_CLASS
#define GAME_STATE_TABLE_CLASS

#include <opencv2/opencv.hpp>

#include "AirHockeyTable.hpp"
#include "Puck.hpp"

class GameState {
    public:
        
        class Reflection {
            public:
                Coordinate averagePosition;
                Coordinate mostRecentReflectionPosition;
                std::string reflectedSurface;
                Vector reflectedVector;
        };

        AirHockeyTable pixelSpaceTable;
        Puck greenPuck;
        Reflection firstOrderReflection;
        GameState(cv::Size rescaledSize);
        void registerLostPuck();
        void updatePuckPosition(cv::Vec3f positionalData);
        void updateLogic(cv::Mat imageToDrawOn);
    private:
        void computeFirstOrderPuckReflection(cv::Mat imageToDrawOn);
        
        
};

#endif
