#include "Training.h"
#include <FS.h>
#include <SPIFFS.h>
#include <string.h>

namespace
{
    const char kModelPath[] = "/training.bin";
    const uint32_t kModelMagic = 0x524C4D31; // "RLM1"
    const uint32_t kModelVersion = 1;

    struct ModelHeader
    {
        uint32_t magic;
        uint32_t version;
        uint32_t actionCount;
        uint32_t featureCount;
        uint32_t payloadSize;
    };
}

const int Training::kTargetOptionsDown[Training::kDownActionCount] = {140, 130, 120, 110, 100};
const int Training::kTargetOptionsUp[Training::kUpActionCount] = {0, 15, 30, 45, 60, 75, 90};

Training::Training()
    : trainingActive(false),
      modelLoaded(false),
      fsReady(false),
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
    fsReady = SPIFFS.begin(true);
    if (!fsReady)
    {
        Serial.println("Training FS mount failed");
    }
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

    return result;
}

Training::StepResult Training::infer(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                                     int downAngleDeg, int upAngleDeg)
{
    StepResult result = {};
    if (!modelLoaded)
    {
        return result;
    }

    float features[kNumFeatures];
    buildFeatures(deltaDistanceCm, avgSpeedCms, avgAccelerationMps2, downAngleDeg, upAngleDeg, features);

    int actionIndex = selectBestAction(features);
    int targetDownAngle = 0;
    int targetUpAngle = 0;
    decodeAction(actionIndex, targetDownAngle, targetUpAngle);

    result.actionIndex = actionIndex;
    result.targetDownAngle = targetDownAngle;
    result.targetUpAngle = targetUpAngle;
    result.reward = computeReward(deltaDistanceCm, avgSpeedCms, avgAccelerationMps2);

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
    if (!fsReady)
    {
        Serial.println("Model save skipped (filesystem unavailable)");
        modelLoaded = false;
        return;
    }

    File file = SPIFFS.open(kModelPath, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open model file for writing");
        modelLoaded = false;
        return;
    }

    ModelHeader header = {};
    header.magic = kModelMagic;
    header.version = kModelVersion;
    header.actionCount = kNumActions;
    header.featureCount = kNumFeatures;
    header.payloadSize = sizeof(weights);

    size_t headerBytes = file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    size_t dataBytes = file.write(reinterpret_cast<const uint8_t *>(weights), sizeof(weights));
    file.close();

    if (headerBytes != sizeof(header) || dataBytes != sizeof(weights))
    {
        Serial.println("Failed to write training model");
        modelLoaded = false;
        return;
    }

    Serial.println("Training model saved");
    modelLoaded = true;
}

bool Training::modelFileExists()
{
    if (!fsReady)
    {
        return false;
    }

    return SPIFFS.exists(kModelPath);
}

bool Training::loadModel()
{
    if (!fsReady)
    {
        Serial.println("Model load skipped (filesystem unavailable)");
        modelLoaded = false;
        return false;
    }

    if (!SPIFFS.exists(kModelPath))
    {
        Serial.println("No training model file found");
        modelLoaded = false;
        return false;
    }

    File file = SPIFFS.open(kModelPath, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open model file for reading");
        modelLoaded = false;
        return false;
    }

    if (file.size() < (sizeof(ModelHeader) + sizeof(weights)))
    {
        Serial.println("Training model file is incomplete");
        file.close();
        modelLoaded = false;
        return false;
    }

    ModelHeader header = {};
    size_t headerBytes = file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    if (headerBytes != sizeof(header))
    {
        Serial.println("Failed to read model header");
        file.close();
        modelLoaded = false;
        return false;
    }

    if (header.magic != kModelMagic || header.version != kModelVersion ||
        header.actionCount != kNumActions || header.featureCount != kNumFeatures ||
        header.payloadSize != sizeof(weights))
    {
        Serial.println("Training model header mismatch");
        file.close();
        modelLoaded = false;
        return false;
    }

    size_t dataBytes = file.read(reinterpret_cast<uint8_t *>(weights), sizeof(weights));
    file.close();

    if (dataBytes != sizeof(weights))
    {
        Serial.println("Failed to read model weights");
        modelLoaded = false;
        return false;
    }

    hasLastStep = false;
    modelLoaded = true;
    Serial.println("Training model loaded");
    return true;
}

void Training::resetModel()
{
    resetWeights();
    if (fsReady && SPIFFS.exists(kModelPath))
    {
        SPIFFS.remove(kModelPath);
    }
    Serial.println("Training model reset");
    modelLoaded = false;
    hasLastStep = false;
}

bool Training::isEpsilonMin() const
{
    return currentEpsilon <= kEpsilonMin;
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

    return selectBestAction(features);
}

int Training::selectBestAction(const float *features) const
{
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
