package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private final MotorEx intake = new MotorEx("intake");
    Command spinIntakeWheel = new SetPower(intake, 1.0).requires(this);
    Command stopIntakeWheel = new SetPower(intake, 0.0).requires(this);
}
