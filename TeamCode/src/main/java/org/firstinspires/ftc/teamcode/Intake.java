package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
        public static final org.firstinspires.ftc.teamcode.Intake INSTANCE = new org.firstinspires.ftc.teamcode.Intake();
        private final MotorEx intake = new MotorEx("intake");
        Command spinIntakeWheel = new SetPower(intake, 0.4).requires(this);
        Command reverseIntakeWheel = new SetPower(intake, -0.4).requires(this);
        Command stopIntakeWheel = new SetPower(intake, 0.0).requires(this);
}

