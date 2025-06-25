package org.firstinspires.ftc.teamcode.util.features;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.dependency.resolution.DependencyResolutionException;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;

public class DefaultMessage implements Feature {
    private DefaultMessage() {}

    public static final DefaultMessage INSTANCE = new DefaultMessage();
    private static String message = "Hello from inside 27971's robot :3";

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
        opMode.getOpMode().telemetry.clear();
        opMode.getOpMode().telemetry.addData("", message);
    }

    private Dependency<?> dependency = (opMode, thing, alsoThing) -> {
        if (!(opMode.getOpMode() instanceof OpModeManagerImpl.DefaultOpMode))throw new DependencyResolutionException("Not a defaultOpMode");
        return null;
    };

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }


    @Target(ElementType.TYPE)
    @Retention(RetentionPolicy.RUNTIME)
    @Documented
    @Inherited
    public @interface Attach {}
}