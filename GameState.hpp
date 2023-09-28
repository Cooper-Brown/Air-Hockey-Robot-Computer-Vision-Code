#ifndef GAME_STATE_TABLE_CLASS
#define GAME_STATE_TABLE_CLASS

#include <opencv2/opencv.hpp>

#include "AirHockeyTable.hpp"
#include "Puck.hpp"

class GameState {
    public:
        AirHockeyTable pixelSpaceTable;
        Puck greenPuck;
        GameState(cv::Size rescaledSize);
        void registerLostPuck();
        void updatePuckPosition(cv::Vec3f positionalData, cv::Mat imageToDrawOn);
    private:
        void computeFirstOrderPuckReflection(cv::Mat imageToDrawOn);
        
};

#endif
