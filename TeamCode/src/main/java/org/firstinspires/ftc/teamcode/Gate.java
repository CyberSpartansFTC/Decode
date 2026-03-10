package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Gate implements Subsystem  {
    public static Gate INSTANCE = new Gate();
    private ServoEx gate = new ServoEx("gate");
    Command close = new SetPosition(gate, 0.03);
    Command open = new SetPosition(gate, 0.8);
}
