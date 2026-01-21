package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class Chassis implements Subsystem {
    public static final Chassis INSTANCE = new Chassis();
    private final MotorEx frontLeftMotor = new MotorEx("frontLeft");
    private final MotorEx frontRightMotor = new MotorEx("frontRight").reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeft");
    private final MotorEx backRightMotor = new MotorEx("backRight");

    @Override
    public void initialize() {
        frontLeftMotor.brakeMode();
        frontRightMotor.brakeMode();
        backLeftMotor.brakeMode();
        backRightMotor.brakeMode();
    }

    Command driverControlled = new MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX(),
            Gamepads.gamepad1().rightStickX()
    );
}
