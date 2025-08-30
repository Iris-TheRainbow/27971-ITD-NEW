package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class DepositWrist implements Subsystem {
    public static final DepositWrist INSTANCE = new DepositWrist();
    public static Servo depositWrist;
    private DepositWrist() {
    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        depositWrist = new CachingServo(hwmap.get(Servo.class, "depositWrist"));
        depositWrist.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){ wristTransfer().schedule(); }

    public static void setPosition(double position) {
        depositWrist.setPosition(position);
    }

    @NonNull
    public static Command wristDeposit() {
        return new Sequential(new Lambda("wrist deposit")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.43)),
                new Wait(.075));
    }
    @NonNull
    public static Command wristWait() {
        return new Sequential(new Lambda("wrist wait")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.24)),
                new Wait(.075));
    }
    @NonNull
    public static Command wristTransfer() {
        return new Sequential(new Lambda("wrist transfer")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.24)),
                new Wait(.075));
    }
    @NonNull
    public static Command wristSepc() {
        return new Sequential(new Lambda("wrist spec")
                .addRequirements(INSTANCE)
                .setInit(() -> setPosition(.43)),
                new Wait(.075));
    }

    @NonNull
    public static Command wristDepo(){
        return new Lambda("wrist sample depo")
                .setRequirements(INSTANCE)
                .setInit(() -> setPosition(.5));
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }
}