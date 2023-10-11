#ifndef GAME_STATE_TABLE_CLASS
#define GAME_STATE_TABLE_CLASS

#include <opencv2/opencv.hpp>

#include "AirHockeyTable.hpp"
#include "Puck.hpp"
#include "StmCommunicator.hpp"

#define STATE_STANDBY 0
#define STATE_DEFEND 1
#define STATE_ATTACK 2

#define TABLE_X_BOUNDARY_MIN 2000
#define TABLE_X_BOUNDARY_MAX 15100
#define TABLE_Y_BOUNDARY_MIN 2000
#define TABLE_Y_BOUNDARY_MAX 17300

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
        int gameState;
        bool resetTrackingAverage;
        StmCommunicator* stmComms;
        GameState(cv::Size rescaledSize, StmCommunicator* stmCommsIn);
        void registerLostPuck();
        void updatePuckPosition(cv::Vec3f positionalData);
        void updateLogic(cv::Mat imageToDrawOn);
    private:
        void computeFirstOrderPuckReflection(cv::Mat imageToDrawOn);
        void standbyProcedure(cv::Mat imageToDrawOn);
        void attackProcedure(cv::Mat imageToDrawOn);
        void defendProcedure(cv::Mat imageToDrawOn);
        
        
};

#endif
