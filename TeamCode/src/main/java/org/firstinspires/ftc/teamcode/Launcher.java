package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Launcher implements Subsystem {
    public static final Launcher INSTANCE = new Launcher();
    private final MotorEx launcher = new MotorEx("launcher");
    private final MotorEx launcherSupport = new MotorEx("launcherSupport").reversed();
    private final MotorGroup launcherMotors = new MotorGroup(launcher, launcherSupport);
    Command spinLaunchWheels = new SetPower(launcherMotors, 0.5).requires(this);
    Command stopLaunchWheels = new SetPower(launcherMotors, 0.0).requires(this);
}
