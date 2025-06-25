package org.firstinspires.ftc.teamcode.util.features;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.Feature;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;

public class Telem implements Feature {
    private Telem() {}

    public static final Telem INSTANCE = new Telem();
    public static TelemetryPacket packet = new TelemetryPacket();
    private static FtcDashboard dash;
    @Override
    public void preUserInitHook(@NonNull Wrapper opMode){
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }
    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    private Dependency<?> dependency = new SingleAnnotation<>(Attach.class);

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