package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class PedroPathingTeleOp implements Component {
        private Follower follower;
        public static Pose startingPose; //See ExampleAuto to understand how to use this
        private boolean automatedDrive;
        private Supplier<PathChain> pathChain;
        private TelemetryManager telemetryM;
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
            follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
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
            follower.update();
            telemetryM.update();
            Drawing.drawDebug(follower);
            Drawing.sendPacket();

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
            if (automatedDrive && (ActiveOpMode.gamepad1().bWasPressed() || !follower.isBusy())) {
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

            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);
            telemetryM.debug("slow mode", slowMode);
            telemetryM.debug("slow mode multiplier", slowModeMultiplier);
        }
    }
