#ifndef TRAINING_H
#define TRAINING_H

#include <Arduino.h>

class Training {
public:
    struct StepResult
    {
        int actionIndex;
        int targetDownAngle;
        int targetUpAngle;
        float reward;
    };

    Training();
    void begin();

    void startTraining();
    void stopTraining();
    bool isTraining();
    StepResult step(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                    int downAngleDeg, int upAngleDeg);
    
    void executeLearnedBehavior();
    bool hasLearnedBehavior();
    
    void saveModel();
    void loadModel();
    void resetModel();

private:
    static constexpr int kDownActionCount = 3;
    static constexpr int kUpActionCount = 3;
    static const int kDownAngleOptions[kDownActionCount];
    static const int kUpAngleOptions[kUpActionCount];
    static constexpr int kNumActions = kDownActionCount + kUpActionCount;
    static constexpr int kNumStates = kDownActionCount * kUpActionCount;

    static constexpr float kGamma = 0.95f;
    static constexpr float kAlpha = 1.0f / kNumActions;
    static constexpr float kEpsilonStart = 1.0f;
    static constexpr float kEpsilonMin = 0.1f;
    static constexpr float kEpsilonDecay = 0.9995f;
    
    bool trainingActive;
    bool modelLoaded;
    bool hasLastStep;
    int lastAction;
    int lastState;
    int kNumSteps = 0;
    float currentEpsilon;
    float qTable[kNumStates][kNumActions];
    uint32_t visitCounts[kNumStates][kNumActions];

    void resetQTable();
    int getStateIndex(int downAngleDeg, int upAngleDeg) const;
    int findDownIndex(int downAngleDeg) const;
    int findUpIndex(int upAngleDeg) const;
    float computeQ(int stateIndex, int actionIndex) const;
    float computeMaxQ(int stateIndex) const;
    int selectAction(int stateIndex);
    void decayEpsilon();
    void decodeAction(int actionIndex, int currentDownAngle, int currentUpAngle,
                      int &targetDownAngle, int &targetUpAngle) const;
    float computeReward(float deltaDistanceCm) const;
};

#endif // TRAINING_H
