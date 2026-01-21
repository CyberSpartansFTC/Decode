package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Component {
    private Limelight3A limelight;
    public static final Limelight INSTANCE = new Limelight();
    public Pose limelightPose;

    public void setLimelightPose(Pose pose) {
        limelightPose = pose;
    }

    @Override
    public void preInit() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        TelemetryEx.telemetry.setUpdateInterval(11);

        limelight.pipelineSwitch(0);

    }

    @Override
    public void preStartButtonPressed() {
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        LLResult result = limelight.getLatestResult();
        Pose3D botpose;

        if (result.isValid()) {
            // Access general information
            botpose = result.getBotpose();
            TelemetryEx.telemetry.addData("Botpose", botpose.toString());
        }
        else {
            return null;
        }
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        Pose returnPose = new Pose(botpose.getPosition().x * 39.37, botpose.getPosition().y * 39.7, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        TelemetryEx.telemetry.debug(returnPose);
        return returnPose;
    }

    public Pose getLimelightPose() {
        limelightPose = getRobotPoseFromCamera();
        return limelightPose;
    }
}
