package org.firstinspires.ftc.teamcode.util.controllers;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.opmodes.Tele;

import java.util.Currency;

public class HolonomicRobotCentricController {
    private final double axialGain,  lateralGain,  headingGain,  axialVelGain,  lateralVelGain,  headingVelGain;
    private double lastX = 0;
    private double lastY = 0;
    public HolonomicRobotCentricController(double axialGain, double lateralGain, double headingGain, double axialVelGain, double lateralVelGain, double headingVelGain){
        this.axialGain = axialGain;
        this.lateralGain = lateralGain;
        this.headingGain = headingGain;
        this.axialVelGain = axialVelGain;
        this.lateralVelGain = lateralVelGain;
        this.headingVelGain = headingVelGain;
    }
    public PoseVelocity2d compute(Pose2d target, Pose2d current){
        double headingErrorRad = (target.heading.toDouble() - current.heading.toDouble()) % (2 * Math.PI);
        if (headingErrorRad < 0) {
            headingErrorRad += 2 * Math.PI;
        }
        if (headingErrorRad > Math.PI) {
            headingErrorRad -= 2 * Math.PI;
        } else if (headingErrorRad < -Math.PI) {
            headingErrorRad += 2 * Math.PI;
        }
        double h = -headingGain * headingErrorRad;


        Vector2d translationalError = new Vector2d(target.position.x - current.position.x, target.position.y - current.position.y);

        double heading = current.heading.toDouble();

        double y = axialGain * translationalError.y;
        double x = lateralGain * translationalError.x;

        double rotX = x * cos(heading) - y * sin(heading);
        double rotY = x * sin(heading) + y * sin(heading);

        return new PoseVelocity2d(
                new Vector2d(
                        -rotX,
                        rotY),
                h);
    }
}
