package org.firstinspires.ftc.teamcode;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift() {

    }
    private MotorEx motor = new MotorEx("flag");
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005,0,0)
            .elevatorFF(0)
            .build();

}
