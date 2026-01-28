#include "Training.h"
#include <string.h>

const int Training::kTargetOptionsDown[Training::kDownActionCount] = {140, 130, 120, 110, 100};
const int Training::kTargetOptionsUp[Training::kUpActionCount] = {0, 15, 30, 45, 60, 75, 90};

Training::Training()
    : trainingActive(false),
      modelLoaded(false),
      hasLastStep(false),
      lastAction(0),
      currentEpsilon(kEpsilonStart)
{
    resetWeights();
}

void Training::begin()
{
    randomSeed(micros());
    resetWeights();
    Serial.println("Training module initialized");
}

void Training::startTraining()
{
    Serial.println("Training started");
    trainingActive = true;
    hasLastStep = false;
    currentEpsilon = kEpsilonStart;
}

void Training::stopTraining()
{
    Serial.println("Training stopped");
    trainingActive = false;
}

bool Training::isTraining()
{
    return trainingActive;
}

Training::StepResult Training::step(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                                    int downAngleDeg, int upAngleDeg)
{
    StepResult result = {};

    if (!trainingActive)
    {
        return result;
    }

    float features[kNumFeatures];
    buildFeatures(deltaDistanceCm, avgSpeedCms, avgAccelerationMps2, downAngleDeg, upAngleDeg, features);
    float reward = computeReward(deltaDistanceCm, avgSpeedCms, avgAccelerationMps2);

    if (hasLastStep)
    {
        float maxQ = computeMaxQ(features);
        float lastQ = computeQ(lastAction, lastFeatures);
        float tdError = reward + (kGamma * maxQ) - lastQ;

        kAlpha = 1 / kNumActions;
        if (kAlpha < kAlphaMin) {
            kAlpha = kAlphaMin;
        }
        for (int i = 0; i < kNumFeatures; ++i)
        {
            weights[lastAction][i] += kAlpha * tdError * lastFeatures[i];
        }
    }

    int actionIndex = selectAction(features);
    int targetDownAngle = 0;
    int targetUpAngle = 0;
    decodeAction(actionIndex, targetDownAngle, targetUpAngle);
    decayEpsilon();

    memcpy(lastFeatures, features, sizeof(lastFeatures));
    lastAction = actionIndex;
    hasLastStep = true;

    result.actionIndex = actionIndex;
    result.targetDownAngle = targetDownAngle;
    result.targetUpAngle = targetUpAngle;
    result.reward = reward;

    kNumSteps++;

    return result;
}

void Training::executeLearnedBehavior()
{
    Serial.println("Executing learned behavior (implementation pending)");
}

bool Training::hasLearnedBehavior()
{
    return modelLoaded;
}

void Training::saveModel()
{
    Serial.println("Saving model (implementation pending)");
}

void Training::loadModel()
{
    Serial.println("Loading model (implementation pending)");
    modelLoaded = false;
}

void Training::resetModel()
{
    resetWeights();
    Serial.println("Resetting model (implementation pending)");
    modelLoaded = false;
    hasLastStep = false;
}

void Training::resetWeights()
{
    for (int action = 0; action < kNumActions; ++action)
    {
        for (int i = 0; i < kNumFeatures; ++i)
        {
            weights[action][i] = 0.0f;
        }
    }
}

void Training::buildFeatures(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                             int downAngleDeg, int upAngleDeg, float *featuresOut) const
{
    featuresOut[0] = normalizeFeature(deltaDistanceCm, kDistScale);
    featuresOut[1] = normalizeFeature(avgSpeedCms, kSpeedScale);
    featuresOut[2] = normalizeFeature(avgAccelerationMps2, kAccelScale);
    featuresOut[3] = normalizeFeature(static_cast<float>(downAngleDeg), kAngleScale);
    featuresOut[4] = normalizeFeature(static_cast<float>(upAngleDeg), kAngleScale);
}

float Training::normalizeFeature(float value, float scale) const
{
    if (scale <= 0.0f)
    {
        return 0.0f;
    }

    float normalized = value / scale;
    if (normalized > 1.0f)
    {
        normalized = 1.0f;
    }
    else if (normalized < -1.0f)
    {
        normalized = -1.0f;
    }

    return normalized;
}

float Training::computeQ(int actionIndex, const float *features) const
{
    float sum = 0.0f;
    for (int i = 0; i < kNumFeatures; ++i)
    {
        sum += weights[actionIndex][i] * features[i];
    }
    return sum;
}

float Training::computeMaxQ(const float *features) const
{
    float bestQ = computeQ(0, features);
    for (int action = 1; action < kNumActions; ++action)
    {
        float qValue = computeQ(action, features);
        if (qValue > bestQ)
        {
            bestQ = qValue;
        }
    }
    return bestQ;
}

int Training::selectAction(const float *features)
{
    int roll = random(0, 10000);
    if (roll < static_cast<int>(currentEpsilon * 10000.0f))
    {
        return random(0, kNumActions);
    }

    float bestQ = computeQ(0, features);
    int bestAction = 0;
    for (int action = 1; action < kNumActions; ++action)
    {
        float qValue = computeQ(action, features);
        if (qValue > bestQ)
        {
            bestQ = qValue;
            bestAction = action;
        }
    }

    return bestAction;
}

void Training::decayEpsilon()
{
    if (currentEpsilon > kEpsilonMin)
    {
        currentEpsilon *= kEpsilonDecay;
        if (currentEpsilon < kEpsilonMin)
        {
            currentEpsilon = kEpsilonMin;
        }
    }
}

void Training::decodeAction(int actionIndex, int &targetDownAngle, int &targetUpAngle) const
{
    int downIndex = actionIndex / kUpActionCount;
    int upIndex = actionIndex % kUpActionCount;
    targetDownAngle = kTargetOptionsDown[downIndex];
    targetUpAngle = kTargetOptionsUp[upIndex];
}

float Training::computeReward(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2) const
{
    float dist = normalizeFeature(deltaDistanceCm, kDistScale);
    float speed = normalizeFeature(avgSpeedCms, kSpeedScale);
    float accel = normalizeFeature(avgAccelerationMps2, kAccelScale);

    return (kRewardDistWeight * dist) + (kRewardSpeedWeight * speed) + (kRewardAccelWeight * accel);
}
