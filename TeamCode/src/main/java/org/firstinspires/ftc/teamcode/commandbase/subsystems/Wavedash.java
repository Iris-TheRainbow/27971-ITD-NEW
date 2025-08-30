package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.LazyPose2d;
import org.firstinspires.ftc.teamcode.util.MathLibKt;
import org.firstinspires.ftc.teamcode.util.PidToPointBuilder;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Wavedash implements Subsystem {

    private static Pose2d initialPose = new Pose2d(0, 0, 0);
    public static PinpointDrive RRDrive;


    private static HardwareMap hwmap;
    private static double turnNerf = 1;

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        hwmap = opMode.getOpMode().hardwareMap;
        RRDrive = GenerateRRDrive(hwmap);
        if (opMode.getOpModeType() == OpModeMeta.Flavor.TELEOP){
            setDefaultCommand(TeleDrive());
        }else {
            setDefaultCommand(PIDToLast());
        }
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opmode){
        RRDrive.updatePoseEstimate();
    }

    public static Pose2d getPose(){
        return RRDrive.pose;
    }

    public static void setInitialPose(Pose2d initialPose){
        Wavedash.initialPose = initialPose;
        RRDrive.pose = initialPose;
        RRDrive.setPinpointPose(initialPose);
    }

    private static PinpointDrive GenerateRRDrive(HardwareMap hwmap){
        return new PinpointDrive(hwmap, Wavedash.initialPose);
    }
    @NonNull
    public static Lambda TeleDrive(){
        return new Lambda("Tele Driving")
                .addRequirements(INSTANCE)
                .setExecute(() ->{
                    RRDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    MathLibKt.cube(Mercurial.gamepad1().leftStickY().state()),
                                    MathLibKt.cube(Mercurial.gamepad1().leftStickX().state())
                            ),
                            turnNerf * MathLibKt.signedFunc((val) -> (5/4)*Math.pow(val*5/4, 2), Mercurial.gamepad1().rightStickX().state())
                    ));
                })
                .setFinish(() ->false)
                .setEnd((interupted) -> RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0)));
    }

    public static Lambda fc(){
        return new Lambda("fc")
                .setInit(() -> INSTANCE.setDefaultCommand(TeleDriveButFc()));
    }

    @NonNull
    public static Lambda TeleDriveButFc(){
        return new Lambda("Tele Driving")
                .addRequirements(INSTANCE)
                .setExecute(() ->{
                    double x = MathLibKt.cube(Mercurial.gamepad1().leftStickY().state());
                    double y = MathLibKt.cube(Mercurial.gamepad1().leftStickX().state());
                    double rotx = x*Math.cos(-Wavedash.getPose().heading.toDouble()) - y*Math.sin(-Wavedash.getPose().heading.toDouble());
                    double roty = x*Math.sin(-Wavedash.getPose().heading.toDouble()) + y*Math.cos(-Wavedash.getPose().heading.toDouble());
                    RRDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    rotx,
                                    roty
                            ),
                            turnNerf * MathLibKt.signedFunc((val) -> (5/4)*Math.pow(val*5/4, 2), Mercurial.gamepad1().rightStickX().state())
                    ));
                })
                .setFinish(() ->false)
                .setEnd((interupted) -> RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0)));
    }

    @NonNull
    public static Command DriveVectorForTime(PoseVelocity2d driveVector, double seconds){
        return new Sequential(new Lambda("Drive vector for set time")
                .setRequirements(INSTANCE)
                .setInit(() -> RRDrive.setDrivePowers(driveVector)),
                new Wait(seconds),
                new Lambda("instant")
                        .setInit(() -> RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
    }

    @NonNull
    public static Lambda PIDToLast() {
        return new Lambda("PID to last set target")
                .addRequirements(INSTANCE)
                .setExecute(() -> {
                    if (RRDrive.getLastTxWorldTarget() != null){
                        Pose2dDual<Time> target = RRDrive.getLastTxWorldTarget();
                        TelemetryPacket p = Telem.packet;
                        Canvas canvas = new Canvas();
                        double translationalError = target.value().minusExp(RRDrive.pose).position.norm();
                        double headingError = Math.abs(Math.toDegrees(target.value().minusExp(RRDrive.pose).heading.toDouble()));
                        if (!(translationalError < .3 && headingError <  1)) {
                            RRDrive.goToTarget(target, Telem.packet.fieldOverlay());
                        }else {
                            RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                });
    }
    @NonNull
    public static Lambda nerfDrive(double multiplier){
        return new Lambda("nerf turn speed")
                .setExecute(() -> { turnNerf = multiplier; });
    }

    @NonNull
    public static Lambda PIDToPoint(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy) {
        LazyPose2d pose = new LazyPose2d(x, y, h);
        return new Lambda("PID to last set target")
                .addRequirements(INSTANCE)
                .setInit(pose::evaluate)
                .setExecute(() -> {
                    Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                    double translationalError = target.value().minusExp(RRDrive.pose).position.norm();
                    double headingError = Math.abs(Math.toDegrees(target.value().minusExp(RRDrive.pose).heading.toDouble()));
                    if (!(translationalError < .5 && headingError <  1)) {
                        RRDrive.goToTarget(target, Telem.packet.fieldOverlay());
                    } else {
                        RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    }
                })
                .setFinish(() -> {
                    Pose2dDual<Time> target = Pose2dDual.constant(pose.value(), 3);
                    double translationalError = target.value().minusExp(RRDrive.pose).position.norm();
                    double headingError = Math.abs(Math.toDegrees(target.value().minusExp(RRDrive.pose).heading.toDouble()));
                    return  translationalError < translationalAccuracy && headingError <  headingAccuracy;
                })
                .setEnd((interupted) -> RRDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0)));
    }

    @NonNull
    public static Lambda PIDToPoint(Pose2d pose, double translationalAccruacy, double headingAccuracy){
        return PIDToPoint(() -> pose.position.x, () -> pose.position.y, pose.heading::toDouble, translationalAccruacy, headingAccuracy);

    }

    public static PidToPointBuilder p2pBuilder(Pose2d pose){
        setInitialPose(pose);
        return new PidToPointBuilder(pose);
    }

    public static final Wavedash INSTANCE = new Wavedash();
    private Wavedash() { }
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}