package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class TelemetryEx implements Component {
    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static TelemetryEx INSTANCE = new TelemetryEx();

    @Override
    public void postUpdate() {
        telemetry.update();
        ActiveOpMode.telemetry().update();
    }
}
