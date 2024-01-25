from bisect import bisect
from enum import IntEnum
import random
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt


class RgvMovementType(IntEnum):
    Wait = 0
    Straight = 1
    ArcLeft = 2
    ArcRight = 3
    UTurnLeft = 4
    UTurnRight = 5
    

class RGV:
    speed = 1
    turningRadius = 4
    uTurnRadius = 2
    waitTimeMin = 90
    waitTimeMax = 100
    straightTimeMin = 1
    straightTimeMax = 2
    arcTimeMin = 2
    arcTimeMax = 3
    uTurnBonusTimeMax = 4
    waitChanceWeight = 0.1
    straightChanceWeight = 5
    arcChanceWeight = 1
    
    safeDistanceFromEdge = uTurnRadius*2
    turningSpeed = speed/turningRadius
    uTurnSpeed = speed/uTurnRadius
    uTurnTurnTime = np.pi/uTurnSpeed
    uTurnStraightMinTime = 2*uTurnRadius/speed
    uTurnTimeMin = uTurnTurnTime + uTurnStraightMinTime
    uTurnTimeMax = uTurnTimeMin + uTurnBonusTimeMax
    weightSum = waitChanceWeight + straightChanceWeight + arcChanceWeight*2
    waitProbCutoff = waitChanceWeight/weightSum
    straightProbCutoff = (waitChanceWeight+straightChanceWeight)/weightSum
    arcLeftCutoff = (waitChanceWeight+straightChanceWeight+arcChanceWeight)/weightSum
    
    def __init__(self, seed: int, vec_startPos_local_ne: np.ndarray, startYawAngle: float, duration: float):
        # Set the seed of the random number generator so that the RGV
        # generation is consistent
        random.seed(seed)
        
        vec_time = [0.0]
        trix_vec_position_local_ne = [vec_startPos_local_ne]
        vec_yawAngle = [startYawAngle]
        vec_movementType = [getRandomRgvMovementType()]
        
        t = 0
        while t <= duration:
            # Determine how long this step takes based on the known movement
            # type for the step and the known amounts of time that that type of
            # movement can take
            match vec_movementType[-1]:
                case RgvMovementType.Straight:
                    deltaTime = random.random()*(RGV.straightTimeMax-RGV.straightTimeMin)+RGV.straightTimeMin
                case RgvMovementType.ArcLeft | RgvMovementType.ArcRight:
                    deltaTime = random.random()*(RGV.arcTimeMax-RGV.arcTimeMin)+RGV.arcTimeMin
                case RgvMovementType.UTurnLeft | RgvMovementType.UTurnRight:
                    deltaTime = random.random()*(RGV.uTurnTimeMax-RGV.uTurnTimeMin)+RGV.uTurnTimeMin
                case _:
                    deltaTime = random.random()*(RGV.waitTimeMax-RGV.waitTimeMin)+RGV.waitTimeMin
            
            # Take a step
            newYawAngle, vec_newPos_local_ne = moveRgv(t + deltaTime, t, trix_vec_position_local_ne[-1], vec_yawAngle[-1], vec_movementType[-1])
            if (distanceToBoundary(vec_newPos_local_ne) < RGV.safeDistanceFromEdge):
                # If the step would take the RGV out of the safe region, don't
                # let it. Instead, only move until you hit the safe region,
                # then note that the next step needs to be some kind of u-turn
                temp = fsolve(lambda x: edgeFinderHelper(t, x, trix_vec_position_local_ne[-1], vec_yawAngle[-1], vec_movementType[-1]), [deltaTime])
                deltaTime = temp[0]
                newYawAngle, vec_newPos_local_ne = moveRgv(t + deltaTime, t, trix_vec_position_local_ne[-1], vec_yawAngle[-1], vec_movementType[-1])
                
                vec_dir_local_ne = np.array([np.cos(newYawAngle),np.sin(newYawAngle)])
                signedAngle = np.arctan2(vec_dir_local_ne[1]*vec_newPos_local_ne[0]-vec_dir_local_ne[0]*vec_newPos_local_ne[1],vec_dir_local_ne[0]*vec_newPos_local_ne[0]+vec_dir_local_ne[1]*vec_newPos_local_ne[1])
                if signedAngle > 0:
                    vec_movementType.append(RgvMovementType.UTurnLeft)
                else:
                    vec_movementType.append(RgvMovementType.UTurnRight)
            else:
                # If the step wouldn't take the RGV out of the safe region,
                # keep the full step and choose a random movement type for the
                # next step
                vec_movementType.append(getRandomRgvMovementType())
            t += deltaTime
            vec_time.append(t)
            trix_vec_position_local_ne.append(vec_newPos_local_ne)
            vec_yawAngle.append(newYawAngle)
        
        self.vec_time = np.array(vec_time)
        self.trix_vec_position_local_ne = np.stack(trix_vec_position_local_ne)
        self.vec_yawAngle = np.array(vec_yawAngle)
        self.vec_movementType = np.array(vec_movementType)

def edgeFinderHelper(t: float, x: np.ndarray, vec_position_local_ne: np.ndarray, yawAngle: float, movementType: RgvMovementType):
    return distanceToBoundary(moveRgv(t + x[0], t, vec_position_local_ne, yawAngle, movementType)[1]) - RGV.safeDistanceFromEdge

def getRgvStateAtTime(rgv: RGV, t: float) -> tuple[float, np.ndarray]:
    prevStateIndex = bisect(rgv.vec_time, t) - 1
    startTime = rgv.vec_time[prevStateIndex]
    vec_startPos_local_ne = rgv.trix_vec_position_local_ne[prevStateIndex,:]
    startYawAngle = rgv.vec_yawAngle[prevStateIndex]
    movementType = rgv.vec_movementType[prevStateIndex]
    return moveRgv(t, startTime, vec_startPos_local_ne, startYawAngle, movementType)

def moveRgv(t: float, startTime: float, vec_startPos_local_ne: np.ndarray, startYawAngle: float, movementType: RgvMovementType) -> tuple[float, np.ndarray]:
    match movementType:
        case RgvMovementType.Straight:
            progressTime = t - startTime
            vec_dir_local_ne = np.array([np.cos(startYawAngle),np.sin(startYawAngle)])
            vec_newPos_local_ne = vec_startPos_local_ne + RGV.speed * progressTime * vec_dir_local_ne
            newYawAngle = startYawAngle
        case RgvMovementType.ArcLeft:
            vec_radiusVectorIn_local_ne = RGV.turningRadius * np.array([-np.sin(startYawAngle),np.cos(startYawAngle)])
            progressTime = t - startTime
            addedYawAngle = RGV.turningSpeed * progressTime
            newYawAngle = startYawAngle + addedYawAngle
            vec_radiusVectorOut_local_ne = RGV.turningRadius * np.array([np.sin(newYawAngle),-np.cos(newYawAngle)])
            vec_newPos_local_ne = vec_startPos_local_ne + vec_radiusVectorIn_local_ne + vec_radiusVectorOut_local_ne
        case RgvMovementType.ArcRight:
            vec_radiusVectorIn_local_ne = RGV.turningRadius * np.array([np.sin(startYawAngle),-np.cos(startYawAngle)])
            progressTime = t - startTime
            addedYawAngle = -RGV.turningSpeed * progressTime
            newYawAngle = startYawAngle + addedYawAngle
            vec_radiusVectorOut_local_ne = RGV.turningRadius * np.array([-np.sin(newYawAngle),np.cos(newYawAngle)])
            vec_newPos_local_ne = vec_startPos_local_ne + vec_radiusVectorIn_local_ne + vec_radiusVectorOut_local_ne
        case RgvMovementType.UTurnLeft:
            # U-turn movement has two parts
            vec_radiusVectorIn_local_ne = RGV.uTurnRadius * np.array([-np.sin(startYawAngle),np.cos(startYawAngle)])
            progressTime = t - startTime
            if (progressTime < RGV.uTurnTurnTime):
                # The first part is an arc
                addedYawAngle = RGV.uTurnSpeed  * progressTime
                newYawAngle = startYawAngle + addedYawAngle
                vec_radiusVectorOut_local_ne = RGV.uTurnRadius * np.array([np.sin(newYawAngle),-np.cos(newYawAngle)])
                vec_newPos_local_ne = vec_startPos_local_ne + vec_radiusVectorIn_local_ne + vec_radiusVectorOut_local_ne
            else:
                # The second part is straight
                vec_startPos_local_ne = vec_startPos_local_ne + 2*vec_radiusVectorIn_local_ne
                newYawAngle = startYawAngle + np.pi
                progressTime = progressTime - RGV.uTurnTurnTime
                vec_dir_local_ne = np.array([np.cos(newYawAngle),np.sin(newYawAngle)])
                vec_newPos_local_ne = vec_startPos_local_ne + RGV.speed * progressTime * vec_dir_local_ne
        case RgvMovementType.UTurnRight:
            # U-turn movement has two parts
            vec_radiusVectorIn_local_ne = RGV.uTurnRadius * np.array([np.sin(startYawAngle),-np.cos(startYawAngle)])
            progressTime = t - startTime
            if (progressTime < RGV.uTurnTurnTime):
                # The first part is an arc
                addedYawAngle = RGV.uTurnSpeed  * progressTime
                newYawAngle = startYawAngle - addedYawAngle
                vec_radiusVectorOut_local_ne = RGV.uTurnRadius * np.array([-np.sin(newYawAngle),np.cos(newYawAngle)])
                vec_newPos_local_ne = vec_startPos_local_ne + vec_radiusVectorIn_local_ne + vec_radiusVectorOut_local_ne
            else:
                # The second part is straight
                vec_startPos_local_ne = vec_startPos_local_ne + 2*vec_radiusVectorIn_local_ne
                newYawAngle = startYawAngle + np.pi
                progressTime = progressTime - RGV.uTurnTurnTime
                vec_dir_local_ne = np.array([np.cos(newYawAngle),np.sin(newYawAngle)])
                vec_newPos_local_ne = vec_startPos_local_ne + RGV.speed * progressTime * vec_dir_local_ne
        case _:
            # This case is for "Waiting" or if another movement type is
            # added later but not implemented (it will just stand still)
            newYawAngle = startYawAngle
            vec_newPos_local_ne = vec_startPos_local_ne
    return (newYawAngle,vec_newPos_local_ne)

def getRandomRgvMovementType() -> RgvMovementType:
    randVal = random.random()
    if randVal < RGV.waitProbCutoff:
        return RgvMovementType.Wait
    elif randVal < RGV.straightProbCutoff:
        return RgvMovementType.Straight
    elif randVal < RGV.arcLeftCutoff:
        return RgvMovementType.ArcLeft
    else:
        return RgvMovementType.ArcRight

def distanceToBoundary(vec_pos_local_ne: np.ndarray) -> float:
    missionAreaHalfWidth = 22.86
    return min(missionAreaHalfWidth + vec_pos_local_ne[0], missionAreaHalfWidth - vec_pos_local_ne[0], missionAreaHalfWidth + vec_pos_local_ne[1], missionAreaHalfWidth - vec_pos_local_ne[1])

def main():
    rgv = RGV(int(random.random()*10000), np.array([0,0]), 0, 900)
    xs = []
    ys = []
    for t in np.arange(0, 900, 1/32):
        _, vec_newPos_local_ne = getRgvStateAtTime(rgv, t)
        xs.append(vec_newPos_local_ne[0])
        ys.append(vec_newPos_local_ne[1])
    plt.plot(xs, ys)
    stop_xs: list[float] = []
    stop_ys: list[float] = []
    for i in range(len(rgv.vec_movementType)):
        if rgv.vec_movementType[i] == RgvMovementType.Wait:
            stop_xs.append(rgv.trix_vec_position_local_ne[i,0])
            stop_ys.append(rgv.trix_vec_position_local_ne[i,1])
    plt.scatter(stop_xs,stop_ys)
    plt.axis('equal')
    plt.grid(which='both')
    plt.show()

if __name__ == "__main__":
    main()