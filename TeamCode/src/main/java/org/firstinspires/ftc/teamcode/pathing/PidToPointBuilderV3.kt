package org.firstinspires.ftc.teamcode.pathing

import com.acmerobotics.roadrunner.Pose2d
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commandbase.CommandUtil
import org.firstinspires.ftc.teamcode.util.OrderedPair
import org.firstinspires.ftc.teamcode.util.Spline
import org.firstinspires.ftc.teamcode.util.distance
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import java.util.function.UnaryOperator
import kotlin.math.abs
import kotlin.math.tan

typealias p2pCommand = (Supplier<Pose2d>, translationalAccuracy: Double, headingAccuracy: Double) -> Command
open class PidToPointBuilderV3(private var pose: Pose2d, private val p2pCommand: p2pCommand, private val currentPose: Supplier<Pose2d>) {
    private var commands = ArrayList<Command>()
    private var path = Path(pose)
    private var openPath = true;
    private var startTangent = tan(pose.heading.toDouble())
    private fun internalSetTangent(pose: Pose2d){ startTangent = pose.heading.toDouble() }

    private fun closePath(){
        if (openPath) {
            commands.add(path.command())
            openPath = false;
            pose = path.pose
        }
    }

    private fun checkPath(){
        if (!openPath){
            path = Path(pose)
        }
    }

    fun stopAndAdd(vararg command: Command): PidToPointBuilderV3{
        closePath()
        commands.add(Parallel(command.asList()))
        return this
    }

    fun stopAndAdd(vararg command: Runnable): PidToPointBuilderV3{
        closePath()
        val parallel = ArrayList<Command>()
        for (runnable: Runnable in command){
            parallel.add(Lambda("instant command").setInit(runnable))
        }
        commands.add(Parallel(parallel))
        return this
    }

    fun waitSeconds(seconds: Double): PidToPointBuilderV3{
        closePath()
        commands.add(Wait(seconds))
        return this
    }

    @JvmOverloads
    fun pidAfter(seconds: Double, target: Pose2d, translationalAccuracy: Double = 3.0, headingAccuracy: Double = 1.0): PidToPointBuilderV3 {
        return waitSeconds(seconds).pidTo(target, translationalAccuracy, headingAccuracy)
    }

    @JvmOverloads
    fun pidTo(target: Pose2d, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderV3 {
        checkPath()
        path.add(
            PathComponent(
                p2pCommand(
                    Supplier<Pose2d>{ target },
                    translationalAccuracy,
                    headingAccuracy
                ),
                target
            )
        )
        return this
    }

    @JvmOverloads
    fun pidRealtive(x: DoubleSupplier, y: DoubleSupplier, h: DoubleSupplier, translationalAccuracy: Double = 1.0 , headingAccuracy: Double = 3.0): PidToPointBuilderV3 {
        closePath()
        commands.add(
            p2pCommand(
                Supplier<Pose2d>{ Pose2d(
                    pose.position.x + x.asDouble,
                    pose.position.y + y.asDouble,
                    pose.heading.toDouble() + h.asDouble ) },
                translationalAccuracy,
                headingAccuracy
            )
        )

        return this
    }

    @JvmOverloads
    fun splineTo(target: Pose2d, endTangent: Double = target.heading.toDouble(),
                 numInt: Int = 4,
                 translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0,
                 headingInteroplation: HeadingInterpolation = HeadingInterpolation.target): PidToPointBuilderV3 {
        checkPath()
        val spline = Spline(OrderedPair(this.pose.position.x, this.pose.position.y), OrderedPair(target.position.x, target.position.y), startTangent, Math.PI + endTangent)
        val curvePoints: MutableList<OrderedPair<Double>> = spline.generateSpline(numInt).splinePoints
        val tangents = spline.splineTangents

        val heading: UnaryOperator<Double> = when (headingInteroplation) {
            HeadingInterpolation.tangential -> UnaryOperator{ x: Double -> tangents[x.toInt()] }
            HeadingInterpolation.linear -> UnaryOperator{ x: Double -> x * ((this.pose.heading.toDouble() - target.heading.toDouble()) / numInt.toDouble()) + this.pose.heading.toDouble() }
            else -> UnaryOperator{ this.pose.heading.toDouble() }
        }

        this.startTangent = endTangent
        for (i in 0 until numInt) {
            val point = curvePoints[i]
            path.add(
                PathComponent(
                    p2pCommand(
                        { Pose2d(point.x, point.y, heading.apply(i.toDouble()))},
                        8.0,
                        999.0
                    ),
                    target
                )
            )
        }
        this.pidTo(Pose2d(curvePoints[numInt - 1].x, curvePoints[numInt - 1].y, heading.apply(numInt - 1.0)), translationalAccuracy, headingAccuracy)
        this.pose = target
        return this
    }

    @JvmOverloads
    fun splineToLinearHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderV3 {
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy,
            HeadingInterpolation.linear
        )
    }

    @JvmOverloads
    fun splineToTargetHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderV3 {
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy,
            HeadingInterpolation.target
        )
    }

    @JvmOverloads
    fun splineToTangentialHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderV3 {
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy,
            HeadingInterpolation.tangential
        )
    }
    fun setTangent(angle: Double): PidToPointBuilderV3 { this.startTangent = angle; return this}

    fun startRepeat(): PidToPointBuilderV3{
        closePath()
        return ChildPidToPointBuilderV3(this.path.pose, this.p2pCommand, currentPose, this)

    }

    fun afterTime(seconds: Double, vararg command: Command): PidToPointBuilderV3{
        path.timeMarker(seconds, Parallel(command.asList()))
        return this;
    }
    fun afterTime(seconds: Double, runnable: Runnable): PidToPointBuilderV3{
        return afterTime(seconds, Lambda("instant command").setInit(runnable))
    }
    fun duringLast(vararg command: Command): PidToPointBuilderV3 { return afterTime(0.0, Parallel(command.asList())) }
    fun duringLast(runnable: Runnable): PidToPointBuilderV3 { return afterTime(0.0, runnable) }
    fun addCommands(command: Command){
        commands.add(command)
    }
    fun build(): Command{
        closePath()
        commands.replaceAll(CommandUtil::proxiedCommand)
        return Sequential(commands)
    }

}

class Path(start: Pose2d){
    private var commands = ArrayList<Command>()
    var pose = start
    private var length = 0.0
    private val markerCommands = ArrayList<Command>()
    fun timeMarker(time: Double, command: Command){
        markerCommands.add(command)
        commands.add(Sequential(Wait(time), Lambda("instantProxy").setInit(command::schedule)))
    }
    fun add(component: PathComponent){
        length += abs(distance(pose.position.x, pose.position.y,component.endPos.position.x, component.endPos.position.y))
        commands.add(component.command)
        pose = component.endPos;
    }
    fun command(): Command{
        val commands = ArrayList<Command>()
        for (command: Command in commands){
            commands.add(command)
        }
        commands.add(
            Lambda("wait for commands")
                .setFinish {
                    markerCommands.any {Mercurial.isScheduled(it)}
                }
        )
        commands.replaceAll(CommandUtil::proxiedCommand)

        return Sequential(commands)
    }
    fun prepend(path: Path): Path{
        this.markerCommands.addAll(path.markerCommands)
        val commands = path.commands
        commands.addAll(this.commands)
        this.commands = commands
        this.length += path.length
        return this;
    }
    fun append(path: Path): Path{
        this.markerCommands.addAll(path.markerCommands)
        this.commands.addAll(path.commands)
        this.length += path.length
        this.pose = path.pose
        return this;
    }
}
class ChildPidToPointBuilderV3(pose: Pose2d, p2pCommand: p2pCommand, poseSupplier: Supplier<Pose2d>, private val parent: PidToPointBuilderV3):  PidToPointBuilderV3(pose, p2pCommand, poseSupplier){
    fun endRepeatBlock(number: Int): PidToPointBuilderV3{
        repeat(number){
            parent.addCommands(this.build())
        }
        return parent;
    }
}

class PathComponent(val command: Command, @JvmField val endPos: Pose2d)


enum class HeadingInterpolation{
    target,
    linear,
    tangential,
    start
}