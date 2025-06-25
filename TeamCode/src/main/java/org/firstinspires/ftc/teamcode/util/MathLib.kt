package org.firstinspires.ftc.teamcode.util

import java.util.function.UnaryOperator
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

@JvmName("MathLib")

fun distance(x1: Double, y1: Double, x2: Double, y2: Double): Double {
    return (sqrt((x2 - x1).pow(2.0) + (y2 - y1).pow(2.0)))
}

fun signedFunc(func: UnaryOperator<Double>, value: Double): Double {
    return sign(value) * func.apply(abs(value))
}

fun cube(x: Double): Double {
    return x * x * x
}

fun arcLength(func: UnaryOperator<Double>, start: Double, end: Double, subInts: Int): Double {
    var lastInput = start
    var lastOutput = func.apply(start)
    var dist = 0.0
    for (i in 0..subInts) {
        val input = i * (end - start) + start
        val output = func.apply(input)
        dist += distance(lastInput, lastOutput, input, output)
        lastOutput = output
        lastInput = input
    }
    return dist
}

class Spline(
    private val point0: OrderedPair<Double>,
    private val point1: OrderedPair<Double>,
    private val tangent0: Double,
    private val tangent1: Double) {

    private val vecTan0 = normVec(cos(tangent0), sin(tangent0))
    private val vecTan1 = normVec(cos(tangent1), sin(tangent1))
    private val magnitude = distance(point0.x, point0.y, point1.x, point1.y)
    private fun spline(t: Double, point0: Double, point1: Double, tangent0: Double, tangent1: Double): Double{
        return (2*t.pow(3) - 3*t.pow(2) + 1) * point0 + (-2*t.pow(3) + 3*t.pow(2))*point1 + (t.pow(3)- 2 * t.pow(2) + t) * magnitude * tangent0 + (-t.pow(3)+ t.pow(2)) * magnitude * tangent1
    }
    private fun tangent(t: Double, point0: Double, point1: Double, tangent0: Double, tangent1: Double): Double {
        return (6*t.pow(3) - 6*t) * point0 + (-6*t.pow(2) + 6*t) * point1 + (3*t.pow(2) - 4*t + 1) * magnitude * tangent0 + (-3*t.pow(2) + 2*t) * magnitude * tangent1
    }

    val splinePoints: MutableList<OrderedPair<Double>> = ArrayList()
    val splineTangents: MutableList<Double> = ArrayList()
    fun generateTangents(numTangents: Int): Spline{
        splineTangents.add(tangent0)

        var i = 0.0
        repeat(numTangents){
            i += 1.0 / numTangents
            splineTangents.add(
                atan2(
                    tangent(i * 1.0 / numTangents, point0.x, point1.x, vecTan0.x, vecTan1.x),
                    tangent(i * 1.0 / numTangents, point0.y, point1.y, vecTan0.y, vecTan1.y)
                )
            )
        }

        splineTangents.apply {
            removeAt(lastIndex)
            add(tangent1)
        }
        return this
    }
    fun generateSpline(numPoints: Int): Spline {
        splineTangents.add(tangent0)
        if (splineTangents.isEmpty()){
            generateTangents(numPoints)
        }
        var i = 0.0;
        repeat(numPoints){
            i += 1.0 / numPoints
            splinePoints.add(
                OrderedPair(
                    spline(i * 1.0 / numPoints, point0.x, point1.x, vecTan0.x, vecTan1.x),
                    spline(i * 1.0 / numPoints, point0.y, point1.y, vecTan0.y, vecTan1.y),
                )
            )
        }

        return this
    }
}

data class OrderedPair<T>(val x: T, val y: T)
class normVec(x: Double, y: Double){
    val x = x / max(x, y)
    val y = y / max(x, y)
}