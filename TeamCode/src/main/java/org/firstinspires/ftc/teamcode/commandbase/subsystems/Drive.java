package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pathing.Drawing;
import org.firstinspires.ftc.teamcode.util.OrderedPair;
import org.firstinspires.ftc.teamcode.util.controllers.HolonomicRobotCentricController;
import org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt;
import org.firstinspires.ftc.teamcode.pathing.Evaluation;
import org.firstinspires.ftc.teamcode.util.LazyPose2d;
import org.firstinspires.ftc.teamcode.util.MathLibKt;
import org.firstinspires.ftc.teamcode.pathing.PidToPointBuilderKt;
import com.qualcomm.hardware.gobilda.Pinpoint;
import org.firstinspires.ftc.teamcode.util.features.Telem;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Drive implements Subsystem {
        public static final Drive INSTANCE = new Drive();
        public static DcMotorEx leftFront, leftBack, rightFront, rightBack;
        public static Pinpoint pinpoint;
        public static Pose2d pose = new Pose2d(0,0,0);
        private static Pose2d lastTarget;
        private static boolean tele = false;

        private static HolonomicRobotCentricController controller;
        private static double turnNerf = 1;
        private static final LinkedList<Pose2d> poseHistory = new LinkedList<>();
        private Drive() { }

        @Override
        public void preUserInitHook(@NonNull Wrapper opMode) {
                HardwareMap hwmap = opMode.getOpMode().hardwareMap;
                leftBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftBack"));
                leftFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "leftFront"));
                rightBack = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightBack"));
                rightFront = new CachingDcMotorEx(hwmap.get(DcMotorEx.class, "rightFront"));
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                pinpoint = hwmap.get(Pinpoint.class, "pinpoint");
                pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(PathingConstantsKt.pinpointXDirection, PathingConstantsKt.pinpointYDirection);
                pinpoint.setOffsets(PathingConstantsKt.pinpointXOffsetMM, PathingConstantsKt.pinpointYOffsetMM, DistanceUnit.MM);
                pinpoint.resetPosAndIMU();
                lastTarget = pose;
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.position.x, pose.position.y, AngleUnit.RADIANS, pose.heading.toDouble()));
                preUserLoopHook(opMode);
                if (opMode.getOpModeType() == OpModeMeta.Flavor.TELEOP) {
                        setDefaultCommand(driveCommand());
                        tele = true;
                }
                if (opMode.getOpModeType() == OpModeMeta.Flavor.AUTONOMOUS){
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        setDefaultCommand(PIDToLast());
                        tele = false;
                }
                controller = new HolonomicRobotCentricController(
                        PathingConstantsKt.axialGain, PathingConstantsKt.lateralGain, PathingConstantsKt.headingGain,
                        PathingConstantsKt.axialVelGain, PathingConstantsKt.lateralVelGain, PathingConstantsKt.headingVelGain
                );
                poseHistory.clear();
        }
        @Override
        public void preUserLoopHook(@NonNull Wrapper opMode){
                pinpoint.update();
                pose = Pinpoint.pinpointToRRPose(pinpoint.getPosition());
                poseHistory.add(pose);
                Drawing.drawPoseHistory(Telem.packet.fieldOverlay(), poseHistory);
                drawRobot("#3F51B5", pose);
        }
        public static void driveUpdate(){
                double x = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().leftStickX().state());
                double y = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().leftStickY().state());
                double rot = MathLibKt.signedFunc(MathLibKt::cube, Mercurial.gamepad1().rightStickX().state()) * turnNerf;

                double newTargetX = pose.position.x;
                double newTargetY = pose.position.y;
                double newTargetHead = pose.heading.toDouble();

                lastTarget = new Pose2d(newTargetX, newTargetY, newTargetHead);
                drawRobot("#4CAF50", lastTarget);
                setDrivePower(new Vector2d(x, y), rot);
        }

        public static void setDrivePower(Vector2d vector, Double rotate) {
                drawVector("#4CAF50", pose, vector);
                drawVector("#4CAF50", new Pose2d(pose.position.x + 15, pose.position.y, 0), new Vector2d(0, rotate*5));

                // Do the kinematics math
                double x = vector.x;
                double y = vector.y;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotate), 1);
                double lfPower = (y + x + rotate) / denominator;
                double lbPower = (y - x + rotate) / denominator;
                double rfPower = (y - x - rotate) / denominator;
                double rbPower = (y + x - rotate) / denominator;
                // Set the Motor Power
                leftBack.setPower(lbPower);
                leftFront.setPower(lfPower);
                rightBack.setPower(rbPower);
                rightFront.setPower(rfPower);
        }
        public static void setDrivePower(PoseVelocity2d input) {
                setDrivePower(new Vector2d(input.linearVel.x, input.linearVel.y), input.angVel);
        }
        private static void drawRobot(String stroke, Pose2d pose){
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                Drawing.drawRobot(canvas, pose);
        }
        private static void drawVector(String stroke, Pose2d pose, Vector2d vector){
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                OrderedPair<Double> endPoint = new OrderedPair<>(pose.position.x + 20 * vector.x,pose.position.y + 20 * vector.y);
                canvas.strokeLine(pose.position.x, pose.position.y, endPoint.getX(), endPoint.getY());
        }
        private static void drawArrowNormalized(String stroke, Pose2d pose, Vector2d vector){
                double max = Math.max(vector.x, vector.y);
                Canvas canvas = Telem.packet.fieldOverlay();
                canvas.setStroke(stroke);
                canvas.strokeLine(pose.position.x, pose.position.y, pose.position.x + 3 * vector.x/max, pose.position.y + 3 * vector.y/max);
        }
        private static PoseVelocity2d computeCommand(Pose2d poseTarget){
                return controller.compute(poseTarget, pose);
        }

        public static void goToTarget(Pose2d poseTarget){
                setDrivePower(computeCommand(poseTarget));
                drawRobot("#4CAF50", poseTarget);
        }
        @NonNull
        public static Lambda nerfDrive(double multiplier){
                return new Lambda("nerf turn speed")
                        .setExecute(() -> { turnNerf = multiplier; });
        }

        @NonNull
        public static Lambda driveCommand() {
            return new Lambda("driveCommand")
                    .addRequirements(INSTANCE)
                    .setExecute(Drive::driveUpdate);
        }
        @NonNull
        public static Lambda PIDToLast() {
                if(lastTarget != null){
                        return PIDToPoint(
                                () -> lastTarget.position.x,
                                () -> lastTarget.position.y,
                                () -> lastTarget.heading.toDouble(), .5, 1,
                                Evaluation.onInit,
                                "PID TO LAST"
                        );
                }
                else return new Lambda("null");
        }
        public static Lambda PIDToPoint(DoubleSupplier x, DoubleSupplier y , DoubleSupplier h, double translationalAccuracy, double headingAccuracy, Evaluation evaluate){
                return PIDToPoint(x, y, h, translationalAccuracy, headingAccuracy, evaluate, "PID TO POINT");
        }
        @NonNull
        public static Lambda PIDToPoint(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy, Evaluation evaluate, String name) {
                LazyPose2d targetPos = new LazyPose2d(x, y, h);
                return new Lambda(name)
                        .addRequirements(INSTANCE)
                        .setInit(targetPos::evaluate)
                        .setExecute(() -> {
                                lastTarget = targetPos.value();
                                double translationalError = targetPos.value().minusExp(Drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(targetPos.value().minusExp(Drive.pose).heading.toDouble()));
                                if (!(translationalError < .5 && headingError <  1)) {
                                        goToTarget(targetPos.value());
                                } else {
                                        setDrivePower(new Vector2d(0, 0), 0.0);
                                }
                        })
                        .setFinish(() -> {
                                Pose2dDual<Time> target = Pose2dDual.constant(targetPos.value(), 3);
                                double translationalError = target.value().minusExp(Drive.pose).position.norm();
                                double headingError = Math.abs(Math.toDegrees(target.value().minusExp(Drive.pose).heading.toDouble()));
                                return  translationalError < translationalAccuracy && headingError <  headingAccuracy;
                        })
                        .setEnd((interrupted) -> setDrivePower(new Vector2d(0, 0), 0.0));
        }
        public static Pose2d getPose(){
                return pose;
        }
        public static PidToPointBuilderKt p2p(Pose2d pose){
                pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));
                Drive.pose = pose;
                return new PidToPointBuilderKt(pose, Drive::PIDToPoint, Drive::getPose);
        }

        @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
        public @interface Attach { }

        private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

        @NonNull @Override
        public Dependency<?> getDependency() { return dependency; }

        @Override
        public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
    }