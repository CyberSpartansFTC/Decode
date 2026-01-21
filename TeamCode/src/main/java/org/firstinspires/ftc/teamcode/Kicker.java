package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Kicker implements Subsystem {

    public static Kicker INSTANCE = new Kicker();
    private ServoEx kicker = new ServoEx("kicker");
    Command kick = new SetPosition(kicker, 0);
    Command retract = new SetPosition(kicker, 0.7);

}
