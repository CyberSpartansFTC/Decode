package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class PedroPathingAutoRed implements Component {
    public static PedroPathingAutoRed INSTANCE = new PedroPathingAutoRed();
    private Follower follower;
    public enum PathState {
        START,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        GRAB1,
        GRAB2,
        GRAB3,
        LAUNCHPOSITION,
        LAUNCH
    }
    private PathState pathState;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private Supplier<PathChain> pickupBall1, pickupBall2, pickupBall3;
    private final Pose startPose = new Pose(72 - 26.295652173913048, 128.97391304347826 - 72, Math.toRadians(216)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72 - 45, 113 - 72, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose controlPoint = new Pose(72 - 48, 120 - 72);
    private final Pose pickup1Pose = new Pose(72 - 44.03478260869565, 83.34782608695653 - 72, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(72 - 44.03478260869565, 59.34782608695653 - 72, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(72 - 44.03478260869565, 35.34782608695653 - 72, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Timer pathTimer;
    private int numBallsCollected;

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
        pathState = PathState.START;
        pathTimer = new Timer();
        Timer opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        numBallsCollected = 0;
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void postStartButtonPressed() {
        Gate.INSTANCE.close.schedule();
        Intake.INSTANCE.stopIntakeWheel.schedule();
        Kicker.INSTANCE.retract.schedule();
        Launcher.INSTANCE.stopLaunchWheels.schedule();
    }

    @Override
    public void preUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        TelemetryEx.telemetry.addData("path state", pathState);
        TelemetryEx.telemetry.addData("x", follower.getPose().getX());
        TelemetryEx.telemetry.addData("y", follower.getPose().getY());
        TelemetryEx.telemetry.addData("heading", follower.getPose().getHeading());
        TelemetryEx.telemetry.update();
    }

    @Override
    public void postUpdate() {
        Drawing.drawDebug(follower);
        Drawing.sendPacket();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                // follower.followPath(scorePreload);
                setPathState(PathState.PICKUP1);
                break;
            case PICKUP1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    switch (numBallsCollected) {
                        case 0:
                            setPathState(PathState.GRAB1);
                            numBallsCollected++;
                            break;
                        case 1:
                            setPathState(PathState.GRAB2);
                            numBallsCollected++;
                            break;
                        case 2:
                            setPathState(PathState.GRAB3);
                            numBallsCollected++;
                            break;
                    }
                }
                break;
            case PICKUP2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    switch (numBallsCollected) {
                        case 0:
                            setPathState(PathState.GRAB1);
                            numBallsCollected++;
                            break;
                        case 1:
                            setPathState(PathState.GRAB2);
                            numBallsCollected++;
                            break;
                        case 2:
                            setPathState(PathState.GRAB3);
                            numBallsCollected++;
                            break;
                    }
                }
                break;
            case PICKUP3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    switch (numBallsCollected) {
                        case 0:
                            setPathState(PathState.GRAB1);
                            numBallsCollected++;
                            break;
                        case 1:
                            setPathState(PathState.GRAB2);
                            numBallsCollected++;
                            break;
                        case 2:
                            setPathState(PathState.GRAB3);
                            numBallsCollected++;
                            break;
                    }
                }
                break;
            case GRAB1:
                if(!follower.isBusy()) {
                    Intake.INSTANCE.spinIntakeWheel.schedule();
                    Gate.INSTANCE.open.schedule();
                    follower.followPath(pickupBall1.get());
                    setPathState(PathState.LAUNCHPOSITION);
                }
                break;
            case GRAB2:
                if(!follower.isBusy()) {
                    Intake.INSTANCE.spinIntakeWheel.schedule();
                    Gate.INSTANCE.open.schedule();
                    follower.followPath(pickupBall2.get());
                    setPathState(PathState.LAUNCHPOSITION);
                }
                break;
            case GRAB3:
                if(!follower.isBusy()) {
                    Intake.INSTANCE.spinIntakeWheel.schedule();
                    Gate.INSTANCE.open.schedule();
                    follower.followPath(pickupBall3.get());
                    setPathState(PathState.LAUNCHPOSITION);
                }
                break;
            case LAUNCHPOSITION:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Gate.INSTANCE.close.schedule();
                    Intake.INSTANCE.stopIntakeWheel.schedule();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(PathState.LAUNCH);
                }
                break;
            case LAUNCH:
                if(!follower.isBusy()) {
                    Launcher.INSTANCE.spinLaunchWheels.thenWait(2).schedule();
                    DecodeAutoBlue.kickSequence.thenWait(1.5).schedule();
                    setPathState(PathState.PICKUP1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        controlPoint,
                        pickup1Pose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pose,
                        controlPoint,
                        scorePose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        controlPoint,
                        pickup2Pose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pose,
                        controlPoint,
                        scorePose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        controlPoint,
                        pickup3Pose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pose,
                        controlPoint,
                        scorePose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3Pose.getHeading())
                .build();

        pickupBall1 = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose().withX(pickup1Pose.getX() + 7)))
                .addPath(new BezierLine(follower.getPose().withX(pickup1Pose.getX() + 7), follower.getPose()))
                .build();

        pickupBall2 = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose().withX(pickup1Pose.getX() + 12)))
                .addPath(new BezierLine(follower.getPose().withX(pickup1Pose.getX() + 12), follower.getPose()))
                .build();

        pickupBall3 = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose().withX(pickup1Pose.getX() + 17)))
                .addPath(new BezierLine(follower.getPose().withX(pickup1Pose.getX() + 17), follower.getPose()))
                .build();
    }
}
