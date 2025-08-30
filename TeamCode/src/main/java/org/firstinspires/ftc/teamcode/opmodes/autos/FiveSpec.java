package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.instakeSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intake;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;
import static org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt.specIntakePose;
import static org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt.specStartPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.CommandUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
import org.firstinspires.ftc.teamcode.util.features.Telem;


import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Telem.Attach
@Mercurial.Attach
@DepositArm.Attach
@DepositWrist.Attach
@DepositClaw.Attach
@IntakeWrist.Attach
@IntakeClaw.Attach
@IntakeRotate.Attach
@Lift.Attach
@Extendo.Attach
@Wavedash.Attach
@BulkRead.Attach
@LoopTimes.Attach
public class FiveSpec extends OpMode {

    private final Pose2d specDepoRelay = new Pose2d(7.5, -45, Math.toRadians(90));
    @Override
    public void init() {
}

    @Override
    public void start(){
        Wavedash.p2pBuilder(specStartPose)
                //preload
                .stopAndAdd(DepositClaw.closeClaw())
                .waitSeconds(.05)
                .pidTo(new Pose2d(0-4, -37, Math.toRadians(90)))
                .duringLast(liftMedium(), Extendo.goTo(0))
                .stopAndAdd(depositSpec())
                .pidTo(new Pose2d(35, -43, Math.toRadians(90)), 6, 99)
                .duringLast(retract())
                .pidTo(new Pose2d(35, -16, Math.toRadians(90)), 6, 99)
                .pidTo(new Pose2d(48, -16, Math.toRadians(90)), 6, 99)
                .pidTo(new Pose2d(48, -52, Math.toRadians(90)), 6, 99)
                .pidTo(new Pose2d(38, -16, Math.toRadians(90)), 6, 99)
                .pidTo(new Pose2d(56, -16, Math.toRadians(90)), 6, 99)
                .pidTo(new Pose2d(56, -52, Math.toRadians(90)), 6, 99)

                //score
                .pidTo(new Pose2d(48, -48, Math.toRadians(90)), 4, 99)
                .pidTo(specIntakePose)
                .duringLast(instakeSpec())
                .waitSeconds(.1)
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.1)
                .pidTo(specDepoRelay, 4, 99)
                .duringLast(new Sequential(DepositClaw.openClaw(), IntakeWrist.wristIntermediate(), IntakeRotate.center(), new Wait(.15), IntakeWrist.wristTransfer(), DepositClaw.closeClaw(), new Wait(.1), IntakeClaw.openClaw()))
                .stopAndAdd(liftMedium())
                .pidTo(new Pose2d(0-3, -37, Math.toRadians(90)), 1, .5)
                .stopAndAdd(depositSpec())
                .pidTo(specDepoRelay, 6, 999)

                //score 2
                .pidTo(new Pose2d(48, -48, Math.toRadians(90)), 4, 99)
                .pidTo(specIntakePose)
                .duringLast(instakeSpec())
                .waitSeconds(.1)
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.1)
                .pidTo(specDepoRelay, 4, 99)
                .duringLast(new Sequential(DepositClaw.openClaw(), IntakeWrist.wristIntermediate(), IntakeRotate.center(), new Wait(.15), IntakeWrist.wristTransfer(), DepositClaw.closeClaw(), new Wait(.1), IntakeClaw.openClaw()))
                .stopAndAdd(liftMedium())
                .pidTo(new Pose2d(0-2, -37, Math.toRadians(90)), 1, .5)
                .stopAndAdd(depositSpec())
                .pidTo(specDepoRelay, 6, 999)


                //score 3
                .pidTo(new Pose2d(48, -48, Math.toRadians(90)), 4, 99)
                .pidTo(specIntakePose)
                .duringLast(instakeSpec())
                .waitSeconds(.1)
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.1)
                .pidTo(specDepoRelay, 4, 99)
                .duringLast(new Sequential(DepositClaw.openClaw(), IntakeWrist.wristIntermediate(), IntakeRotate.center(), new Wait(.15), IntakeWrist.wristTransfer(), DepositClaw.closeClaw(), new Wait(.1), IntakeClaw.openClaw()))
                .stopAndAdd(liftMedium())
                .pidTo(new Pose2d(0-1, -37, Math.toRadians(90)), 1, .5)
                .stopAndAdd(depositSpec())
                .stopAndAdd(DepositClaw.openClaw())

                .pidTo(new Pose2d(34, -45.5, Math.toRadians(145)))
                .duringLast(Extendo.goTo(475),DepositArm.armTransfer())
                .build().schedule();
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
