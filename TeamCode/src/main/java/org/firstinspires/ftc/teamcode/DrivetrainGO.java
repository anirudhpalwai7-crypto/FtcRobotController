package org.firstinspires.ftc.robotcontroller.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DrivetrainGO {


    DcMotor front_left_motor, back_left_motor, back_right_motor, front_right_motor;


    public void init(HardwareMap hardwareMap) {

        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");


        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void drive(double forward, double strafe, double turn) {

        double front_left_motorPower = forward + strafe + turn;
        double back_left_motorPower = forward - strafe + turn;
        double back_right_motorPower = forward + strafe - turn;
        double front_right_motorPower = forward - strafe - turn;

        double maxPower = 1;

        maxPower = Math.max(maxPower, Math.abs(front_left_motorPower));
        maxPower = Math.max(maxPower, Math.abs(back_left_motorPower));
        maxPower = Math.max(maxPower, Math.abs(back_right_motorPower));
        maxPower = Math.max(maxPower, Math.abs(front_right_motorPower));

        front_left_motor.setPower(front_left_motorPower / maxPower);
        back_left_motor.setPower(back_left_motorPower / maxPower);
        back_right_motor.setPower(back_right_motorPower / maxPower);
        front_right_motor.setPower(front_right_motorPower / maxPower);
    }
}
