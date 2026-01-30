#include "Training.h"

const int Training::kDownAngleOptions[Training::kDownActionCount] = {140, 150, 160};
const int Training::kUpAngleOptions[Training::kUpActionCount] = {40, 90, 125};

Training::Training()
    : trainingActive(false),
      modelLoaded(false),
      hasLastStep(false),
      lastAction(0),
      lastState(0),
      currentEpsilon(kEpsilonStart)
{
    resetQTable();
}

void Training::begin()
{
    randomSeed(micros());
    resetQTable();
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

        qTable[lastState][lastAction] += kAlpha * tdError;
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
    resetQTable();
    Serial.println("Resetting model (implementation pending)");
    modelLoaded = false;
    hasLastStep = false;
}

void Training::resetQTable()
{
    for (int state = 0; state < kNumStates; ++state)
    {
        for (int action = 0; action < kNumActions; ++action)
        {
            qTable[state][action] = 0.0f;
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
