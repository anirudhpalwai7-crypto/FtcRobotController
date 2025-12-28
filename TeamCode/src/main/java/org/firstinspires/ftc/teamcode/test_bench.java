package org.firstinspires.ftc.robotcontroller.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class test_bench extends OpMode {

    DcMotor test_bench;

    @Override
    public void init() {
        test_bench = hardwareMap.get(DcMotor.class, "test_motor");
        test_bench.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

 double y = gamepad2.right_stick_y;
 if (gamepad2.left_stick_y > 0) {
     test_bench.setPower(1);
 }
    }
}
