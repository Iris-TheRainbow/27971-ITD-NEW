package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;
import static org.firstinspires.ftc.teamcode.commandbase.CommandUtil.proxiedCommand;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
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
@Drive.Attach
@BulkRead.Attach
@LoopTimes.Attach
public class TestingOpMode extends OpMode {
    @Override
    public void init(){
       Mercurial.gamepad1().cross().onTrue(
               Drive.PIDToPoint(
                   () -> 20,
                   () -> 0,
                   () -> Math.toRadians(90),
                   3,
                   3,
                   Evaluation.onInit
               )
       );
       Mercurial.gamepad1().circle().onTrue(
               Drive.PIDToPoint(
                   () -> 0,
                   () -> 0,
                   () -> Math.toRadians(0),
                   3,
                   3,
                   Evaluation.onInit
       ));
       Mercurial.gamepad1().triangle().onTrue(
               new Parallel(
                       new Sequential(
                               retract(), transfer(), liftHigh()
                       ),
                       Drive.p2p(new Pose2d(0, 0, Math.toRadians(90)))
                               .pidTo(new Pose2d(0, 10, Math.toRadians(30)))
                               .pidTo(new Pose2d(0,0,0))
                               .build()
               ).then(proxiedCommand(new Sequential(DepositClaw.openClaw(), new Wait(.05), new Parallel(DepositArm.armExtend(), new Sequential(new Wait(.1), retract())))))
       );
       Mercurial.gamepad1().b().onTrue(new Sequential(Drive.nerfDrive(.5), CommandGroups.intake()));

    }
    @Override
    public void loop(){
    }
}
