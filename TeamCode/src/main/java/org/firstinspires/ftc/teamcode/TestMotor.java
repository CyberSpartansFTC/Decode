package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestMotor extends OpMode {
    DcMotor spin;
    @Override
    public void init() {
        spin = hardwareMap.get(DcMotor.class, "spin");
    }

    @Override
    public void loop() {

        if(gamepad1.a){
            spin.setPower(1.0);
        }
        else if (gamepad1.b){
            spin.setPower(0.0);
        }
    }
}
