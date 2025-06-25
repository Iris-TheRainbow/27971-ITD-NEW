package org.firstinspires.ftc.teamcode.pathing

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.hardware.gobilda.Pinpoint
import kotlin.math.PI


@JvmField
val otosOffsetX = -3.0998
@JvmField
val otosOffsetY = -3.0397
@JvmField
val otosOffsetH = -1.5672

@JvmField
val offset = SparkFunOTOS.Pose2D(otosOffsetX, otosOffsetY, otosOffsetH)

@JvmField
val otosLinearScalar = .985
@JvmField
val otosAngularScalar = .995



@JvmField
val pinpointXOffsetMM = -55.325
@JvmField
val pinpointYOffsetMM = -15

@JvmField
val pinpointXDirection = Pinpoint.EncoderDirection.REVERSED
@JvmField
val pinpointYDirection = Pinpoint.EncoderDirection.REVERSED

@JvmField
val axialGain = -.5
@JvmField
val lateralGain = -axialGain
@JvmField
val headingGain = 1
@JvmField
val axialVelGain = 0.15
@JvmField
val lateralVelGain = 0.15
@JvmField
val headingVelGain = 0.15

@JvmField
var inPerTick = 1.0
@JvmField
var lateralInPerTick = .65
@JvmField
var trackWidthTicks = 12.0

@JvmField
var kS = 1.9
@JvmField
var kV = .145
@JvmField
var kA = .05

@JvmField
var maxWheelVel = 70.0
@JvmField
var minProfileAccel = -50.0
@JvmField
var maxProfileAccel = 50.0

@JvmField
var maxAngVel = 2 * PI
@JvmField
var maxAngAccel = 1 * PI
