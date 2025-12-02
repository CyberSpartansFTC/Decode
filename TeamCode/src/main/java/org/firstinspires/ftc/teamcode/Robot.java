package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@TeleOp(name="TeleOp")
public class Robot extends NextFTCOpMode {
    // Initialize hardware and controller bindings
    public static final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.LEFT);

    public Robot() {
        addComponents(
                new SubsystemComponent(Chassis.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        Button headingReset = button(() -> gamepad1.y);
        Button spinIntake = button(() -> gamepad1.a);
        headingReset.whenBecomesTrue(imu::zero);
        spinIntake
            .toggleOnBecomesTrue()
            .whenBecomesTrue(Intake.INSTANCE.spinIntakeWheel)
            .whenBecomesFalse(Intake.INSTANCE.stopIntakeWheel);

        imu.zero();
        Chassis.INSTANCE.driverControlled.schedule();
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
