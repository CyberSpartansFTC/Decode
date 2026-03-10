package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class PedroPathingTeleOp implements Component {
    public static final PedroPathingTeleOp INSTANCE = new PedroPathingTeleOp();
    public Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void postInit() {
        if (follower == null) {
            follower = Constants.createFollower(ActiveOpMode.hardwareMap());
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        }

        // startingPose = startingPose == null ? new Pose(26.295652173913048 - 72,128.97391304347826 - 72,Math.toRadians(-36), PedroCoordinates.INSTANCE) : startingPose;
        // startingPose = new Pose(72 - 26.295652173913048, 128.97391304347826 - 72, Math.toRadians(216));
        startingPose = new Pose(36.000, 9.000, Math.toRadians(90));
        follower.setStartingPose(startingPose);
        follower.update();

        Limelight.INSTANCE.setLimelightPose(startingPose);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72 - 45, 113 - 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(-55), 0.8))
                .build();
    }

    @Override
    public void postStartButtonPressed() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void preUpdate() {
        //Call this once per loop
        setPose();

        follower.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -ActiveOpMode.gamepad1().left_stick_y,
                    -ActiveOpMode.gamepad1().left_stick_x,
                    -ActiveOpMode.gamepad1().right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier,
                    -ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier,
                    -ActiveOpMode.gamepad1().right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
        if (ActiveOpMode.gamepad1().aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        // if (automatedDrive && (ActiveOpMode.gamepad1().bWasPressed() || !follower.isBusy())) {
        if(automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (ActiveOpMode.gamepad1().rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (ActiveOpMode.gamepad1().xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (ActiveOpMode.gamepad1().yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }


        /* TelemetryEx.telemetry.debug("position", follower.getPose());
        TelemetryEx.telemetry.debug("velocity", follower.getVelocity());
        TelemetryEx.telemetry.debug("automatedDrive", automatedDrive);
        TelemetryEx.telemetry.debug("slow mode", slowMode);
        TelemetryEx.telemetry.debug("slow mode multiplier", slowModeMultiplier); */

    }

    @Override
    public void postUpdate() {
        Drawing.drawDebug(follower);
        Drawing.sendPacket();
    }

    public Command autoPosition = new InstantCommand(() -> {
        follower.followPath(pathChain.get());
        automatedDrive = true;
    });

    public Command teleOpDrive = new InstantCommand(() -> {
        follower.startTeleopDrive();
        automatedDrive = false;
    });

    private void setPose() {
        /* Pose limelightPose = Limelight.INSTANCE.getLimelightPose();
         if(limelightPose == null) {
            TelemetryEx.telemetry.debug(follower.getPose());
        }
        else {
            limelightPose = limelightPose.setHeading(follower.getHeading());

            TelemetryEx.telemetry.debug(limelightPose);
            follower.setPose(limelightPose);
        }*/

    }

    public void resetHeading() {
        follower.setPose(follower.getPose().withHeading(0));
    }
}
