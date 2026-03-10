package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="Decode Auto Blue")
public class DecodeAutoBlue extends NextFTCOpMode {
    public SequentialGroup kickback;
    public ParallelGroup intake;
    public ParallelGroup travel;
    public static SequentialGroup kickSequence;
    public DecodeAutoBlue() {
        addComponents(
                // new SubsystemComponent(Chassis.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(Launcher.INSTANCE),
                new SubsystemComponent(Gate.INSTANCE),
                new SubsystemComponent(Kicker.INSTANCE),
                PedroPathingAutoBlue.INSTANCE,
                // Limelight.INSTANCE,
                TelemetryEx.INSTANCE,
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        kickback = new SequentialGroup(
                Gate.INSTANCE.open,
                new Delay(0.2),
                Launcher.INSTANCE.kickBack,
                new Delay(0.2),
                Gate.INSTANCE.close,
                Launcher.INSTANCE.stopLaunchWheels
        );

        intake = new ParallelGroup(
                Intake.INSTANCE.spinIntakeWheel,
                Gate.INSTANCE.open
        );

        travel = new ParallelGroup(
                Intake.INSTANCE.stopIntakeWheel,
                Gate.INSTANCE.close
        );

        kickSequence = new SequentialGroup(
                Gate.INSTANCE.open,
                Kicker.INSTANCE.kick,
                new Delay(0.4),
                Gate.INSTANCE.close,
                Kicker.INSTANCE.retract
        );

        kickSequence.setInterruptible(true);

        kickSequence.requires(Kicker.INSTANCE, Launcher.INSTANCE);

    }

    @Override
    public void onStartButtonPressed() {
        PedroPathingAutoBlue.INSTANCE.setPathState(PedroPathingAutoBlue.PathState.START);
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
