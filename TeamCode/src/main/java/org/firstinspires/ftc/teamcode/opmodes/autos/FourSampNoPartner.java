package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intake;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intakeAutoShort;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;
import static org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt.sampleDepoSpot;
import static org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt.sampleStartPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandbase.CommandGroups;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.pathing.PathingConstantsKt;
import org.firstinspires.ftc.teamcode.util.OffsetThing;
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import static org.firstinspires.ftc.teamcode.commandbase.CommandUtil.*;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import kotlin.coroutines.intrinsics.IntrinsicsKt;

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
public class FourSampNoPartner extends OpMode {
    private Command driveCommand;
    private OffsetThing cycleOneOffset = new OffsetThing();
    private OffsetThing cycleTwoOffset = new OffsetThing();
    enum configState{
        cycleOne,
        cycleTwo,
        done
    }
    configState state = configState.cycleOne;
    @Override
    public void init() {

    }

    @Override
    public void init_loop(){
        switch(state){
            case cycleOne:
                telemetry.addLine("CycleOne");
                telemetry.addData("x", cycleOneOffset.x);
                telemetry.addData("y", cycleOneOffset.y);
                Mercurial.gamepad1().cross().onTrue(instant(() -> state = configState.done));
                Mercurial.gamepad1().dpadUp().onTrue(instant(() -> cycleOneOffset.y += .0001));
                Mercurial.gamepad1().dpadDown().onTrue(instant(() -> cycleOneOffset.y -= .0001));
                Mercurial.gamepad1().dpadLeft().onTrue(instant(() -> cycleOneOffset.x -= .0001));
                Mercurial.gamepad1().dpadRight().onTrue(instant(() -> cycleOneOffset.y += .0001));
                break;

            case done:
                telemetry.addLine("Done!");
                Mercurial.gamepad1().circle().onTrue(instant(() -> state = configState.cycleOne));
                break;
        }
    }


    @Override
    public void start(){
        Wavedash.p2pBuilder(sampleStartPose)
                //preload
                .waitSeconds(.01)
                .stopAndAdd(DepositClaw.closeClaw())
                .waitSeconds(.01)
                .stopAndAdd(Wavedash.PIDToPoint(sampleDepoSpot, 1, 3).with(new Sequential(CommandGroups.liftHigh().then(DepositArm.armUp()), DepositClaw.openClaw())))
                .waitSeconds(.01)

                //partner sample
                .pidTo(new Pose2d(-31, -64, Math.toRadians(180)))
                .duringLast(Lift.goTo(0), DepositArm.armTransfer(), DepositWrist.wristTransfer(), Extendo.goTo(475), IntakeClaw.openClaw(), IntakeWrist.wristIntake(), IntakeRotate.center())
                .waitSeconds(.1)
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.15)
                .pidTo(sampleDepoSpot)
                .duringLast(retract().then(transfer()).then(liftHigh()).then(DepositArm.armUp()))
                .stopAndAdd(DepositClaw.openClaw())

                //spike 1
                .pidTo(new Pose2d(-54.25, -50.5, Math.toRadians(260)))
                .duringLast(intakeAuto().with(Lift.goTo(0)))
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.1)
                .pidTo(sampleDepoSpot)
                .duringLast(retract().then(transfer()).then(liftHigh()).then(DepositArm.armUp()))
                .stopAndAdd(DepositClaw.openClaw())

                //spike 2
                .pidTo(new Pose2d(-60.5, -41, Math.toRadians(270)))
                .duringLast(intakeAutoShort().with(Lift.goTo(0)))
                .waitSeconds(.1)
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.15)
                .pidTo(sampleDepoSpot)
                .duringLast(retract().then(transfer()).then(liftHigh()).then(DepositArm.armUp()))
                .stopAndAdd(DepositClaw.openClaw())

                //spike 3
                .pidTo(new Pose2d(-51, -36, Math.toRadians(335)))
                .duringLast(intakeAuto().with(Lift.goTo(0)), IntakeRotate.center().then(IntakeRotate.previous()).then(IntakeRotate.previous()))
                .stopAndAdd(IntakeClaw.closeClaw())
                .waitSeconds(.15)
                .pidTo(sampleDepoSpot)
                .duringLast(retract().then(transfer()).then(liftHigh()).then(DepositArm.armUp()))
                .stopAndAdd(DepositClaw.openClaw())

                //park
                .pidTo(new Pose2d(-60, -15, Math.toRadians(0)), 10, 99)
                .pidTo(new Pose2d(-24, -11, Math.toRadians(0)))
                .duringLast(Lift.goTo(250))
                .build().schedule();
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}