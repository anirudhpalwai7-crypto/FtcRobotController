package org.firstinspires.ftc.robotcontroller.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Driver_Controlled extends OpMode {
    DrivetrainGO mecanum = new DrivetrainGO();
    private double forward, strafe, turn;
    private DcMotor Intake, Transfer, Turret;
    private Servo servo_lift, back_servo;

    @Override
    public void init() {

        mecanum.init(hardwareMap);
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        servo_lift = hardwareMap.get(Servo.class, "serv_lift");
        back_servo = hardwareMap.get(Servo.class, "back_servo");


    }

    public void loop() {
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double turn = gamepad2.right_stick_x;

        mecanum.drive(forward, strafe, turn);

        if (gamepad2.right_bumper) {
            Intake.setPower(-1);
        } else if (gamepad2.right_trigger > 0) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);

        }
        if (gamepad2.left_bumper) {
            Transfer.setPower(-1);
        } else if (gamepad2.left_trigger > 0) {
            Transfer.setPower(1);
        } else {
            Transfer.setPower(0);


        }
        if (gamepad2.triangle) {
            Turret.setPower(1);
        } else {
            Turret.setPower(0);
        }

        if (gamepad2.dpad_right) {
            servo_lift.setPosition(0.5);
        } else if (gamepad2.dpad_left) {
            servo_lift.setPosition(-0.5);
        } else {
            servo_lift.setPosition(0);

        }
        if (gamepad2.dpad_up) {
            back_servo.setPosition(0.5);
        } else if (gamepad2.dpad_down) {
            back_servo.setPosition((-0.5));

        } else {
            back_servo.setPosition(0);
        }


    }
}


