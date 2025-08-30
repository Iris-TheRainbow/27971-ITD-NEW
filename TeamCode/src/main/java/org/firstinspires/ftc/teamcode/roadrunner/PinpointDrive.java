package org.firstinspires.ftc.teamcode.roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.gobilda.Pinpoint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;


@Config
public class PinpointDrive extends WavedashMecanumDrive {

    public Pinpoint pinpoint;
    private Pose2d lastPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    public void setPinpointPose(Pose2d pose){
        pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));
    }
    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        pinpoint = hardwareMap.get(Pinpoint.class, "pinpoint");
        pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PathingConstantsKt.pinpointXDirection, PathingConstantsKt.pinpointYDirection);
        pinpoint.setOffsets(PathingConstantsKt.pinpointXOffsetMM, PathingConstantsKt.pinpointYOffsetMM, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));

    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPose != pose) {
            pinpoint.setPosition(Pinpoint.rrToPinpointPose(pose));
        }

        pinpoint.update();

        pose = Pinpoint.pinpointToRRPose(pinpoint.getPosition());
        lastPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return new PoseVelocity2d(
                new Vector2d(
                    pinpoint.getVelX(DistanceUnit.INCH),
                    pinpoint.getVelY(DistanceUnit.INCH)
                ),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }


}
