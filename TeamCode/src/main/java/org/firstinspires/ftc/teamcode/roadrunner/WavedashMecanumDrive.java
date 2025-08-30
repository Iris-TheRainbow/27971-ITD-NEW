package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class WavedashMecanumDrive {
    public Pose2d pose;
    public static Params PARAMS = new Params();
    private Pose2dDual<Time> lastTxWorldTarget;
    public Pose2dDual<Time> getLastTxWorldTarget(){
        return this.lastTxWorldTarget;
    }

    public void goToTarget(Pose2dDual<Time> txWorldTarget, Canvas canvas){
        PoseVelocity2d robotVelRobot = updatePoseEstimate();
        Pose2d error = txWorldTarget.value().minusExp(this.pose);
        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain - 2, PARAMS.lateralGain - 2, PARAMS.headingGain - 2,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, pose, robotVelRobot);

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));
        RobotLog.a("FUCKFUCKFUCK");
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        drawPoseHistory(canvas);

        canvas.setStroke("#4CAF50");
        Drawing.drawRobot(canvas, txWorldTarget.value());

        canvas.setStroke("#3F51B5");
        Drawing.drawRobot(canvas, pose);
        lastTxWorldTarget = txWorldTarget;
    }

    public static class Params {

        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
        public double lateralInPerTick =  .65;
        public double trackWidthTicks = 12;

        // feedforward parameters (in tick units)
        public double kS =  1.9;
        public double kV = .145;
        public double kA = .05;

        // path profile parameters (in inches)
        public double maxWheelVel = 70;
        public double minProfileAccel = -50;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = 2 * Math.PI; // shared with path
        public double maxAngAccel = 1* Math.PI;

        // path controller gains
        public double axialGain = 6;
        public double lateralGain = 6;
        public double headingGain = 7; // shared with turn

        public double axialVelGain = .15;
        public double lateralVelGain = .15;
        public double headingVelGain = .15; // shared with turn
    }

    public final MecanumKinematics kinematics;

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;

    public final AccelConstraint defaultAccelConstraint;

    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx rightFront;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public WavedashMecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
        leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightBack = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
         class DriveLocalizer implements Localizer {

            public DriveLocalizer() {

            }

            @Override
             public Twist2dDual<Time> update(){
                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
        }
        localizer = new DriveLocalizer();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

        defaultAccelConstraint =
                new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    }

    public void setDrivePowers(@NonNull PoseVelocity2d powers) {
        updatePoseEstimate();
        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double h = powers.angVel;
        double normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(h), 1);
        leftFront.setPower((x + y + h ) / normalizer);
        leftBack.setPower((x - y + h ) / normalizer);
        rightFront.setPower((x - y - h ) / normalizer);
        rightBack.setPower((x + y - h ) / normalizer);
    }


    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

}
