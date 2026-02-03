#include "Training.h"
#include <FS.h>
#include <SPIFFS.h>

namespace
{
    const char kModelPath[] = "/training.bin";
    const uint32_t kModelMagic = 0x524C4D31; // "RLM1"
    const uint32_t kModelVersion = 2;

    struct ModelHeader
    {
        uint32_t magic;
        uint32_t version;
        uint32_t stateCount;
        uint32_t actionCount;
        uint32_t payloadSize;
    };
}

const int Training::kDownAngleOptions[Training::kDownActionCount] = {140, 90, 45};
const int Training::kUpAngleOptions[Training::kUpActionCount] = {40, 90, 125};

Training::Training()
    : trainingActive(false),
      modelLoaded(false),
      fsReady(false),
      hasLastStep(false),
      lastAction(0),
      lastState(0),
      totalEpisodes(0),
      trainingStartMs(0),
      accumulatedTrainingMs(0),
      currentEpsilon(kEpsilonStart)
{
    resetQTable();
}

void Training::begin()
{
    randomSeed(micros());
    resetQTable();
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
    totalEpisodes = 0;
    accumulatedTrainingMs = 0;
    trainingStartMs = millis();
}

void Training::stopTraining()
{
    Serial.println("Training stopped");
    if (trainingActive)
    {
        accumulatedTrainingMs += millis() - trainingStartMs;
    }
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

    (void)avgSpeedCms;
    (void)avgAccelerationMps2;

    if (!trainingActive)
    {
        return result;
    }

    int currentState = getStateIndex(downAngleDeg, upAngleDeg);
    float reward = computeReward(deltaDistanceCm);

    if (hasLastStep)
    {
        float maxQ = computeMaxQ(currentState);
        float lastQ = computeQ(lastState, lastAction);
        float tdError = reward + (kGamma * maxQ) - lastQ;

        float alpha = 1.0f / (1.0f + static_cast<float>(visitCounts[lastState][lastAction]));
        qTable[lastState][lastAction] += alpha * tdError;
        visitCounts[lastState][lastAction] += 1;
    }

    int actionIndex = selectAction(currentState);
    int targetDownAngle = 0;
    int targetUpAngle = 0;
    decodeAction(actionIndex, downAngleDeg, upAngleDeg, targetDownAngle, targetUpAngle);
    decayEpsilon();

    lastState = currentState;
    lastAction = actionIndex;
    hasLastStep = true;

    result.actionIndex = actionIndex;
    result.targetDownAngle = targetDownAngle;
    result.targetUpAngle = targetUpAngle;
    result.reward = reward;

    totalEpisodes++;

    return result;
}

Training::StepResult Training::infer(float deltaDistanceCm, float avgSpeedCms, float avgAccelerationMps2,
                                     int downAngleDeg, int upAngleDeg)
{
    StepResult result = {};

    (void)avgSpeedCms;
    (void)avgAccelerationMps2;

    if (!modelLoaded)
    {
        return result;
    }

    int currentState = getStateIndex(downAngleDeg, upAngleDeg);
    int actionIndex = selectBestAction(currentState);
    int targetDownAngle = 0;
    int targetUpAngle = 0;
    decodeAction(actionIndex, downAngleDeg, upAngleDeg, targetDownAngle, targetUpAngle);

    result.actionIndex = actionIndex;
    result.targetDownAngle = targetDownAngle;
    result.targetUpAngle = targetUpAngle;
    result.reward = computeReward(deltaDistanceCm);

    return result;
}

uint32_t Training::getTotalEpisodes() const
{
    return totalEpisodes;
}

float Training::getTotalTrainingSeconds() const
{
    unsigned long totalMs = accumulatedTrainingMs;
    if (trainingActive)
    {
        totalMs += millis() - trainingStartMs;
    }
    return static_cast<float>(totalMs) / 1000.0f;
}

const char *Training::getActionLabel(int actionIndex) const
{
    if (actionIndex < 0 || actionIndex >= kNumActions)
    {
        return "Unknown";
    }
    return (actionIndex < kDownActionCount) ? "Down" : "Up";
}

bool Training::isDownAction(int actionIndex) const
{
    return actionIndex >= 0 && actionIndex < kDownActionCount;
}

int Training::getDownActionCount() const
{
    return kDownActionCount;
}

int Training::getUpActionCount() const
{
    return kUpActionCount;
}

int Training::getDownAngleOption(int index) const
{
    if (index < 0 || index >= kDownActionCount)
    {
        return kDownAngleOptions[0];
    }
    return kDownAngleOptions[index];
}

int Training::getUpAngleOption(int index) const
{
    if (index < 0 || index >= kUpActionCount)
    {
        return kUpAngleOptions[0];
    }
    return kUpAngleOptions[index];
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
    header.stateCount = kNumStates;
    header.actionCount = kNumActions;
    header.payloadSize = sizeof(qTable) + sizeof(visitCounts);

    size_t headerBytes = file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    size_t qBytes = file.write(reinterpret_cast<const uint8_t *>(qTable), sizeof(qTable));
    size_t countBytes = file.write(reinterpret_cast<const uint8_t *>(visitCounts), sizeof(visitCounts));
    file.close();

    if (headerBytes != sizeof(header) || qBytes != sizeof(qTable) || countBytes != sizeof(visitCounts))
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

    if (file.size() < (sizeof(ModelHeader) + sizeof(qTable) + sizeof(visitCounts)))
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
        header.stateCount != kNumStates || header.actionCount != kNumActions ||
        header.payloadSize != (sizeof(qTable) + sizeof(visitCounts)))
    {
        Serial.println("Training model header mismatch");
        file.close();
        modelLoaded = false;
        return false;
    }

    size_t qBytes = file.read(reinterpret_cast<uint8_t *>(qTable), sizeof(qTable));
    size_t countBytes = file.read(reinterpret_cast<uint8_t *>(visitCounts), sizeof(visitCounts));
    file.close();

    if (qBytes != sizeof(qTable) || countBytes != sizeof(visitCounts))
    {
        Serial.println("Failed to read training model data");
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
    resetQTable();
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

void Training::resetQTable()
{
    for (int state = 0; state < kNumStates; ++state)
    {
        for (int action = 0; action < kNumActions; ++action)
        {
            qTable[state][action] = 0.0f;
            visitCounts[state][action] = 0;
        }
    }
}

int Training::getStateIndex(int downAngleDeg, int upAngleDeg) const
{
    int downIndex = findDownIndex(downAngleDeg);
    int upIndex = findUpIndex(upAngleDeg);
    return (downIndex * kUpActionCount) + upIndex;
}

int Training::findDownIndex(int downAngleDeg) const
{
    for (int i = 0; i < kDownActionCount; ++i)
    {
        if (kDownAngleOptions[i] == downAngleDeg)
        {
            return i;
        }
    }
    return 0;
}

int Training::findUpIndex(int upAngleDeg) const
{
    for (int i = 0; i < kUpActionCount; ++i)
    {
        if (kUpAngleOptions[i] == upAngleDeg)
        {
            return i;
        }
    }
    return 0;
}

float Training::computeQ(int stateIndex, int actionIndex) const
{
    return qTable[stateIndex][actionIndex];
}

float Training::computeMaxQ(int stateIndex) const
{
    float bestQ = computeQ(stateIndex, 0);
    for (int action = 1; action < kNumActions; ++action)
    {
        float qValue = computeQ(stateIndex, action);
        if (qValue > bestQ)
        {
            bestQ = qValue;
        }
    }
    return bestQ;
}

int Training::selectAction(int stateIndex)
{
    int roll = random(0, 10000);
    if (roll < static_cast<int>(currentEpsilon * 10000.0f))
    {
        return random(0, kNumActions);
    }

    return selectBestAction(stateIndex);
}

int Training::selectBestAction(int stateIndex) const
{
    float bestQ = computeQ(stateIndex, 0);
    int bestAction = 0;
    for (int action = 1; action < kNumActions; ++action)
    {
        float qValue = computeQ(stateIndex, action);
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

void Training::decodeAction(int actionIndex, int currentDownAngle, int currentUpAngle,
                            int &targetDownAngle, int &targetUpAngle) const
{
    if (actionIndex < kDownActionCount)
    {
        targetDownAngle = kDownAngleOptions[actionIndex];
        targetUpAngle = currentUpAngle;
        return;
    }

    int upIndex = actionIndex - kDownActionCount;
    if (upIndex >= kUpActionCount)
    {
        upIndex = 0;
    }
    targetDownAngle = currentDownAngle;
    targetUpAngle = kUpAngleOptions[upIndex];
}

float Training::computeReward(float deltaDistanceCm) const
{
    return deltaDistanceCm;
}
