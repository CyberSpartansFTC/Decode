package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import dev.nextftc.core.components.Component;

public class TelemetryEx implements Component {
    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static TelemetryEx INSTANCE = new TelemetryEx();

    @Override
    public void postUpdate() {
        telemetry.update();
    }
}
