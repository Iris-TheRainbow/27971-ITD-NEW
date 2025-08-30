package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.instakeSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.prepDepo;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;
import static org.firstinspires.ftc.teamcode.commandbase.CommandUtil.proxiedCommand;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.CommandGroups;
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


import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.IfElse;
import dev.frozenmilk.mercurial.commands.util.Wait;

@TeleOp
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
public class Tele extends OpMode {
    @Override
    public void init() {
        Lift.liftOffset = 0;
        //lift to high basket
        Mercurial.gamepad1().y().onTrue(new Sequential(Wavedash.nerfDrive(1), retract(), transfer(), liftHigh()));
        //lift to high bar
        //retract extendo and transfer
        Mercurial.gamepad1().a().onTrue(new Sequential(Wavedash.nerfDrive(1), retract(), transfer(), prepDepo(), DepositWrist.wristSepc()));
        //extend
        Mercurial.gamepad1().b().onTrue(new Sequential(Wavedash.nerfDrive(.5), CommandGroups.intake()));
                                                                                                                                      //toggle for claw
        //Mercurial.gamepad1().rightBumper().onTrue(new Parallel(IntakeClaw.toggleClaw(), DepositClaw.toggleClaw()));
        Mercurial.gamepad1().rightBumper().onTrue(new IfElse(() -> Lift.getTarget() > 300, proxiedCommand(new Sequential(DepositClaw.openClaw(), new Wait(.05), new Parallel(DepositArm.armExtend(), new Sequential(new Wait(.1), retract())))), proxiedCommand(new Parallel(IntakeClaw.toggleClaw(), DepositClaw.toggleClaw()).then(new Wait(.1)).then(new Parallel(DepositArm.armTransfer(), DepositWrist.wristTransfer())))));
        //hang extend
        Mercurial.gamepad1().dpadUp().onTrue(Lift.offset(10));
        //hang retract
        Mercurial.gamepad1().dpadDown().onTrue(Lift.offset(-10));
        Mercurial.gamepad1().dpadRight().onTrue(Lift.hang());
        Mercurial.gamepad1().dpadLeft().onTrue(CommandGroups.instakeSpec());
        Mercurial.gamepad1().leftTrigger().conditionalBindState().greaterThan(.2).bind().onTrue(IntakeRotate.next());
        Mercurial.gamepad1().rightTrigger().conditionalBindState().greaterThan(.2).bind().onTrue(IntakeRotate.previous());
        Mercurial.gamepad1().rightStickButton().and(Mercurial.gamepad1().dpadDown()).onTrue(new Parallel(Lift.goTo(1000), DepositArm.armTransfer(), Lift.disableSkippy()));
        Mercurial.gamepad1().leftBumper().onTrue(new IfElse(() -> Extendo.getTarget() > 100, Extendo.goTo(0).with(IntakeWrist.wristSpec()), new Parallel(Extendo.goTo(475), IntakeWrist.wristSpec())));
        gamepad1.setLedColor(255, 0, 0, 999999999);
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.addData("left", Mercurial.gamepad1().leftTrigger().state());
        telemetry.addData("right", Mercurial.gamepad1().rightTrigger().state());
    }
}
