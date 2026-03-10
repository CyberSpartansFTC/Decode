package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.FilterBuilder;
import dev.nextftc.control.interpolators.ConstantInterpolator;
import dev.nextftc.control.interpolators.FirstOrderEMAInterpolator;
import dev.nextftc.control.interpolators.FirstOrderEMAParameters;
import dev.nextftc.control.interpolators.InterpolatorElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Launcher implements Subsystem {
    public static final Launcher INSTANCE = new Launcher();

    private final MotorEx launcher = new MotorEx("launcher");
    private final MotorEx launcherSupport = new MotorEx("launcherSupport").reversed();
    private final MotorGroup launcherMotors = new MotorGroup(launcher, launcherSupport);
    public static ControlSystem launcherController;
    public static double power;
    Command spinLaunchWheels;
    Command stopLaunchWheels;
    Command kickBack;
    public InterpolatorElement interpolator;
    public static double kP;
    public static double kI;
    public static double kD;
    public static double alpha;


    @Override
    public void initialize() {
        ActiveOpMode.telemetry().addData("Controller Initialized", true);
        ActiveOpMode.telemetry().update();
        alpha = 0.015;
        interpolator = new FirstOrderEMAInterpolator(new FirstOrderEMAParameters(alpha));
        kP = 0.0008;
        kI = 0;
        kD = 0;
        launcherController = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(0.00045, 0.01, 0.0)
                .interpolator(interpolator)
                .build();

        launcherController.setGoal(new KineticState(0, 0));
        kickBack = new RunToVelocity(launcherController, -500.0, 25).requires(this);
        stopLaunchWheels = new RunToVelocity(launcherController, 0.0, 1).requires(this);
        spinLaunchWheels = new RunToVelocity(launcherController, 2400.0, 50).requires(this);

    }


    public void periodic() {
        power = launcherController.calculate(launcherMotors.getState());
        launcherMotors.setPower(power);
        TelemetryEx.telemetry.addData("Launcher velocity", launcherMotors.getVelocity());
        TelemetryEx.telemetry.addData("Intended velocity", launcherController.getReference().getVelocity());
        TelemetryEx.telemetry.addData("Launcher power", power);
        ActiveOpMode.telemetry().addData("Launcher velocity", launcherMotors.getVelocity());
        ActiveOpMode.telemetry().addData("Intended velocity", launcherController.getReference().getVelocity());
        ActiveOpMode.telemetry().addData("Launcher power", power);

    }




}
