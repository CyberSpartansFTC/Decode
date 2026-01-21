package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@TeleOp(name="Decode TeleOp")
public class DecodeTeleOp extends NextFTCOpMode {

    public DecodeTeleOp() {
        addComponents(
                // new SubsystemComponent(Chassis.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                PedroPathingTeleOp.INSTANCE,
                // Limelight.INSTANCE,
                TelemetryEx.INSTANCE,
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        Button autoAlign = button(() -> gamepad1.a);
        Button cancelAlign = button(() -> gamepad1.b);
        Button spinIntake = button(() -> gamepad2.x);
        Button spinLauncher = button(() -> gamepad2.dpad_up);
        Button kick = button(() -> gamepad2.y);

        spinIntake
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.spinIntakeWheel)
                .whenBecomesFalse(Intake.INSTANCE.stopIntakeWheel);

        spinLauncher
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Launcher.INSTANCE.spinLaunchWheels)
                .whenBecomesFalse(Launcher.INSTANCE.stopLaunchWheels);

        kick
                .whenBecomesTrue(Kicker.INSTANCE.kick)
                .whenBecomesFalse(Kicker.INSTANCE.retract);

        autoAlign.whenBecomesTrue(PedroPathingTeleOp.INSTANCE.autoPosition);
        cancelAlign.whenBecomesTrue(PedroPathingTeleOp.INSTANCE.teleOpDrive);

    }
    @Override
    public void onUpdate() {
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

}
