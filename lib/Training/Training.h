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
    static constexpr int kDownActionCount = 5;
    static constexpr int kUpActionCount = 7;
    static const int kTargetOptionsDown[kDownActionCount];
    static const int kTargetOptionsUp[kUpActionCount];
    static constexpr int kNumActions = kDownActionCount * kUpActionCount;
    static constexpr int kNumFeatures = 5;

    static constexpr float kAlpha = 0.4f;
    static constexpr float kGamma = 0.95f;
    static constexpr float kEpsilonStart = 1.0f;
    static constexpr float kEpsilonMin = 0.1f;
    static constexpr float kEpsilonDecay = 0.9995f;

    static constexpr float kDistScale = 10.0f;
    static constexpr float kSpeedScale = 10.0f;
    static constexpr float kAccelScale = 5.0f;
    static constexpr float kAngleScale = 180.0f;

    static constexpr float kRewardDistWeight = 1.0f;
    static constexpr float kRewardSpeedWeight = 1.0f;
    static constexpr float kRewardAccelWeight = 1.0f;

    bool trainingActive;
    bool modelLoaded;
    bool hasLastStep;
    int lastAction;
    float currentEpsilon;
    float lastFeatures[kNumFeatures];
    float weights[kNumActions][kNumFeatures];

    void resetWeights();
    void buildFeatures(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                       int downAngleDeg, int upAngleDeg, float *featuresOut) const;
    float normalizeFeature(float value, float scale) const;
    float computeQ(int actionIndex, const float *features) const;
    float computeMaxQ(const float *features) const;
    int selectAction(const float *features);
    void decayEpsilon();
    void decodeAction(int actionIndex, int &targetDownAngle, int &targetUpAngle) const;
    float computeReward(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2) const;
};

#endif // TRAINING_H
