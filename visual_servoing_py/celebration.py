import sys
import time
import argparse
import motion
import almath
from naoqi import ALProxy

from nao_driver import NaoDriver


def celebration_arms():
    print("arms")

    # Enable Whole Body Balancer
    isEnabled = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # end initialize whole body, define arms motions

    useSensorValues = False

    frame = motion.FRAME_ROBOT

    # Arms motion
    effectorList = ["LArm", "RArm"]

    # pathLArm
    pathLArm = []
    currentTf = motionProxy.getTransform("LArm", frame, useSensorValues)
    # 1
    target1Tf = almath.Transform(currentTf)
    target1Tf.r2_c4 += 0.08  # y
    target1Tf.r3_c4 += 0.14  # z

    # 2
    target2Tf = almath.Transform(currentTf)
    target2Tf.r2_c4 -= 0.05  # y
    target2Tf.r3_c4 -= 0.07  # z

    pathLArm.append(list(target1Tf.toVector()))
    pathLArm.append(list(target2Tf.toVector()))
    pathLArm.append(list(target1Tf.toVector()))
    pathLArm.append(list(target2Tf.toVector()))
    pathLArm.append(list(target1Tf.toVector()))

    # pathRArm
    pathRArm = []
    currentTf = motionProxy.getTransform("RArm", frame, useSensorValues)
    # 1
    target1Tf = almath.Transform(currentTf)
    target1Tf.r2_c4 += 0.05  # y
    target1Tf.r3_c4 -= 0.07  # z

    # 2
    target2Tf = almath.Transform(currentTf)
    target2Tf.r2_c4 -= 0.08  # y
    target2Tf.r3_c4 += 0.14  # z

    pathRArm.append(list(target1Tf.toVector()))
    pathRArm.append(list(target2Tf.toVector()))
    pathRArm.append(list(target1Tf.toVector()))
    pathRArm.append(list(target2Tf.toVector()))
    pathRArm.append(list(target1Tf.toVector()))
    pathRArm.append(list(target2Tf.toVector()))

    pathList = [pathLArm, pathRArm]

    axisMaskList = [almath.AXIS_MASK_VEL,  # for "LArm"
                    almath.AXIS_MASK_VEL]  # for "RArm"

    coef = 0.5
    timesList = [[coef * (i + 1) for i in range(5)],  # for "LArm" in seconds
                 [coef * (i + 1) for i in range(6)]]  # for "RArm" in seconds

    # called cartesian interpolation
    motionProxy.transformInterpolations(
        effectorList, frame, pathList, axisMaskList, timesList)

    # end define arms motions, define torso motion


def celebration_torso():
    print("torso")

    # Enable Whole Body Balancer
    isEnabled = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # end initialize whole body, define arms motions
    useSensorValues = False

    frame = motion.FRAME_ROBOT

    # Torso Motion
    effectorList = ["Torso", "LArm", "RArm"]

    dy = 0.06
    dz = 0.06

    # pathTorso
    currentTf = motionProxy.getTransform("Torso", frame, useSensorValues)
    # 1
    target1Tf = almath.Transform(currentTf)
    target1Tf.r2_c4 += dy
    target1Tf.r3_c4 -= dz

    # 2
    target2Tf = almath.Transform(currentTf)
    target2Tf.r2_c4 -= dy
    target2Tf.r3_c4 -= dz

    pathTorso = []
    for i in range(3):
        pathTorso.append(list(target1Tf.toVector()))
        pathTorso.append(currentTf)
        pathTorso.append(list(target2Tf.toVector()))
        pathTorso.append(currentTf)

    pathLArm = [motionProxy.getTransform("LArm", frame, useSensorValues)]
    pathRArm = [motionProxy.getTransform("RArm", frame, useSensorValues)]

    pathList = [pathTorso, pathLArm, pathRArm]

    axisMaskList = [almath.AXIS_MASK_ALL,  # for "Torso"
                    almath.AXIS_MASK_VEL,  # for "LArm"
                    almath.AXIS_MASK_VEL]  # for "RArm"

    coef = 0.5
    timesList = [
        [coef * (i + 1) for i in range(12)],  # for "Torso" in seconds
        [coef * 12],  # for "LArm" in seconds
        [coef * 12]  # for "RArm" in seconds
    ]

    motionProxy.transformInterpolations(
        effectorList, frame, pathList, axisMaskList, timesList)


def celebration_dab(side="L"):
    print("dab")

    dab_angles = {
        "HeadYaw": 0.5 * (2 * (side=="L") - 1),
        "HeadPitch": 0.5,
        "LShoulderPitch": 0.5 * (side=="L"),
        "LShoulderRoll": 1.5 - 2 * (side=="R"),
        "LElbowYaw": 0.0,
        "LElbowRoll": -2.0 * (side=="R"),
        "LWristYaw": 0.0,
        "RShoulderPitch": 0.5 * (side=="R"),
        "RShoulderRoll": 0.5 - 2 * (side=="R"),
        "RElbowYaw": 0.0,
        "RElbowRoll": 2.0 * (side=="L"),
        "RWristYaw": 0.0,
    }

    fractionMaxSpeed = 0.5
    motionProxy.setAngles(list(dab_angles.keys()),
                          list(dab_angles.values()),
                          fractionMaxSpeed)

    motionProxy.waitUntilMoveIsFinished()


if __name__ == '__main__':
    robotIp = "localhost"
    # robotIp = "172.17.0.1"
    robotPort = 11212

    motionProxy = ALProxy("ALMotion", robotIp, robotPort)
    postureProxy = ALProxy("ALRobotPosture", robotIp, robotPort)

    # Wake up robot
    motionProxy.wakeUp()

    postureProxy.goToPosture("StandInit", 1.)

    celebration_dab("L")
    time.sleep(1)
    celebration_dab("R")
    time.sleep(1)
    postureProxy.goToPosture("StandInit", 1.)
    celebration_torso()
    postureProxy.goToPosture("StandInit", 1.)
    celebration_arms()

    # Deactivate whole body
    isEnabled = False
    motionProxy.wbEnable(isEnabled)
    # Go to rest position
    motionProxy.rest()

    # end script
