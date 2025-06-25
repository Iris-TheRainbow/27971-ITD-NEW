package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class IntakeRotate implements Subsystem {
    private static Servo rotation;
    public static final IntakeRotate INSTANCE = new IntakeRotate();

    private IntakeRotate() { }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        rotation = hwmap.get(Servo.class, "intakeRotate");
        rotation.setDirection(Servo.Direction.REVERSE);
    }

    private static void setWristRotation(double pos){
        rotation.setPosition(pos);
    }
    @NonNull
    public static Lambda wristRotate(double pos){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(() -> { setWristRotation(pos);});
    }
    @NonNull
    public static Lambda wristRotateSupplier(DoubleSupplier pos){
        return new Lambda("rotate the wrist")
                .setRequirements(INSTANCE)
                .setExecute(() -> { setWristRotation(pos.getAsDouble());})
                .setInterruptible(true);
    }
    private static int index = 0;
    public static List<Double> poses = Arrays.asList(.98, .83, .68, .53, .38);

    @NonNull
    public static Lambda next(){
        return new Lambda("next wrist pos")
                .setRequirements(INSTANCE)
                .setInit(() -> {
                    if (index < 4){
                        index++;
                    }
                })
                .setExecute(() -> setWristRotation(poses.get(index)));
    }
    @NonNull
    public static Lambda previous(){
        return new Lambda("previous wrist pos")
                .setRequirements(INSTANCE)
                .setInit(() -> {
                    if (index > 0){
                        index--;
                    }
                })
                .setExecute(() -> setWristRotation(poses.get(index)));
    }

    public static Lambda center(){
        return new Lambda("center wrist")
                .setRequirements(INSTANCE)
                .setInit(() -> index = 2)
                .setExecute(() -> setWristRotation(poses.get(index)));
    }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}