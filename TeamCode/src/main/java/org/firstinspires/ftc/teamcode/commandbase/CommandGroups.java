package org.firstinspires.ftc.teamcode.commandbase;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Wavedash;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class CommandGroups {
    public static Command transfer(){
        return new Sequential(new Parallel(DepositArm.armTransfer()), new Wait(.1), DepositClaw.closeClaw(),new Wait(.1), IntakeClaw.openClaw());
    }
    public static Command transferOld(){
        return new Sequential(new Parallel(DepositWrist.wristTransfer(),new Wait(.1), DepositArm.armTransfer()),new Wait(.05), DepositClaw.closeClaw(),new Wait(.05), IntakeClaw.openClaw());
    }
    public static Command intake(){
        return new Sequential(new Parallel(Lift.goTo(0), Extendo.goTo(475), IntakeClaw.openClaw(), DepositWrist.wristTransfer(), DepositArm.armTransfer(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command instakeSpec(){
        return new Parallel(Lift.goTo(0), IntakeClaw.openClaw(), DepositWrist.wristTransfer(),DepositArm.armTransfer(), DepositClaw.openClaw(), IntakeWrist.wristSpec(), IntakeRotate.wristRotate(0));
    }
    public static Command intakeAuto(){
        return new Sequential(new Parallel(Extendo.goTo(420), IntakeClaw.openClaw(), DepositWrist.wristTransfer(), DepositArm.armWait(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command intakeAutoShort(){
        return new Sequential(new Parallel(Lift.goTo(0), Extendo.goTo(200), IntakeClaw.openClaw(), DepositWrist.wristWait(), DepositArm.armWait(), IntakeWrist.wristExtend()), IntakeWrist.wristIntake());
    }
    public static Command retract(){
        return new Sequential(IntakeRotate.center(), IntakeWrist.wristTransfer(), IntakeClaw.closeClaw(), DepositClaw.openClaw(), DepositWrist.wristTransfer(), DepositArm.armTransfer(), Lift.goTo(0).with(Extendo.goTo(75)).then(Extendo.goTo(0)), new Wait(.1));
    }
    public static Command prepDepo(){
        return new Parallel(DepositWrist.wristDeposit(), DepositArm.armExtend(), DepositClaw.closeClaw());
    }
    private static Command liftGoTo(int ticks){
        return new Sequential(new Parallel(prepDepo(), Lift.goTo(ticks)), DepositArm.armUp());
    }
    public static Command liftHigh(){
        return liftGoTo(1580).then(DepositWrist.wristDepo());
    }
    public static Command liftMedium(){
        return new Parallel(Lift.goTo(450), DepositWrist.wristSepc(), DepositArm.armUp());
    }
    public static Command depositSpec(){
        return new Sequential(
                Wavedash.PIDToPoint(() -> Wavedash.RRDrive.pose.position.x, () -> Wavedash.RRDrive.pose.position.y - 5, () -> Wavedash.RRDrive.pose.heading.toDouble(), .5, 3),
                DepositClaw.openClaw()
        );
    }
}
