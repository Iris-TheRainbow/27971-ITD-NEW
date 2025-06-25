package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.Pinpoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.CommandGroups;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.pathing.Evaluation;
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Mercurial.Attach
@DepositArm.Attach
@DepositWrist.Attach
@DepositClaw.Attach
@IntakeWrist.Attach
@IntakeClaw.Attach
@IntakeRotate.Attach
@Lift.Attach
@Extendo.Attach
@Drive.Attach
@BulkRead.Attach
@LoopTimes.Attach
@Telem.Attach
public class FiveSpecAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(10, -62.5, Math.toRadians(90));
    private Command driveCommand;
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), new Wait(.1), IntakeClaw.closeClaw(), new Wait(.2));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), new Wait(.25), IntakeClaw.closeClaw(), new Wait(.45));
    }

    @Override
    public void init() {
        driveCommand = Drive.p2p(initialPose)
                .stopAndAdd(DepositClaw.closeClaw())
                .waitSeconds(.025)
                .pidTo(new Pose2d(10, -32, Math.toRadians(90)))
                .duringLast(liftMedium())
                .pidTo(new Pose2d(10, -36, Math.toRadians(90)))
                .stopAndAdd(DepositClaw.closeClaw(), retract())
                //strafe left
                .pidTo(new Pose2d(33, -40, Math.toRadians(90)), 2, 9999)
                .duringLast(retract())
                .pidTo(new Pose2d(33, -18, Math.toRadians(90)), 8, 9999)
                .pidTo(new Pose2d(44, -18, Math.toRadians(90)), 6, 9999)
                //first push
                .pidTo(new Pose2d(46, -52, Math.toRadians(90)), 4, 9999)
                .pidTo(new Pose2d(46, -14, Math.toRadians(90)), 8, 9999)
                .pidTo(new Pose2d(53, -14, Math.toRadians(90)), 6, 9999)
                //second push
                .pidTo(new Pose2d(52, -52, Math.toRadians(90)), 4, 9999)
                .pidTo(new Pose2d(52, -14, Math.toRadians(90)), 8, 9999)
                .pidTo(new Pose2d(64, -14, Math.toRadians(90)), 6, 9999)
                .pidTo(new Pose2d(64, -52, Math.toRadians(90)), 4, 9999)
                .build();
    }
    @Override
    public void start(){
        //driveCommand.schedule();
        Drive.pose = new Pose2d(12, -62.5, Math.toRadians(0));
        Drive.pinpoint.setPosition(Pinpoint.rrToPinpointPose(Drive.pose));
        Drive.PIDToPoint(() -> 12, () -> -62.5, ()-> Math.toRadians(180), 6, 0, Evaluation.onInit);
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
