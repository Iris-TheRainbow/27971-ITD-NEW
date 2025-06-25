package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@Drive.Attach
@TeleOp
public class DriveDirectionDebugger extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.triangle){
            Drive.leftFront.setPower(1);
        }else{
            Drive.leftFront.setPower(0);
        }
        if (gamepad1.square){
            Drive.leftBack.setPower(1);
        }else{
            Drive.leftBack.setPower(0);
        }
        if (gamepad1.cross){
            Drive.rightBack.setPower(1);
        }else{
            Drive.rightBack.setPower(0);
        }
        if (gamepad1.circle){
            Drive.rightFront.setPower(1);
        }else{
            Drive.rightFront.setPower(0);
        }
    }
}
