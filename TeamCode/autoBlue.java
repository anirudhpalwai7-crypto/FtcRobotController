/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class autoBlue extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx front_left_motor, back_left_motor, back_right_motor, front_right_motor;
    DcMotor Intake, Transfer, Turret;
    static final double TICKS_REV = 537.7;
    static final double WHEEL_DIAMETER = 4.0945;
    static final double COUNTS_INCH = (TICKS_REV * 1) / (WHEEL_DIAMETER / Math.PI);
    private IMU imu;

    @Override
    public void runOpMode() {
        front_left_motor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        front_right_motor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Turret = hardwareMap.get(DcMotor.class, "Turret");



        front_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_left_motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);


        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        waitForStart();
        runtime.reset();
        imu.resetYaw();



        while (opModeIsActive()) {
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (imuAngle < 18) {
                front_left_motor.setPower(-0.5);
                back_left_motor.setPower(-0.5);
                front_right_motor.setPower(0.5);
                back_right_motor.setPower(0.5);

                telemetry.addData("Turning Yaw:", imuAngle);
                telemetry.update();
            } else {
                front_left_motor.setPower(0);
                back_left_motor.setPower(0);
                front_right_motor.setPower(0);
                back_right_motor.setPower(0);// exit loop once 18° is reached
            }

        if (runtime.seconds() < 10) {
            Turret.setPower(0.7);
        }
        if (runtime.seconds() >= 4 && runtime.seconds() < 10) {
            Intake.setPower(-0.8);
            Transfer.setPower(-0.8);
        } else {
            Turret.setPower(0);
            Intake.setPower(0);
            Transfer.setPower(0);
        }
        if (time.seconds() >= 10 && time.seconds() <= 12) {
                front_left_motor.setPower(0.6);
                back_left_motor.setPower(0.6);
                front_right_motor.setPower(0.6);
                back_right_motor.setPower(0.6);
            } else {
                front_left_motor.setPower(0);
                back_left_motor.setPower(0);
                front_right_motor.setPower(0);
                back_right_motor.setPower(0);
            }
        }

        idle();



                    telemetry.addData("Yaw:", imu);
                    telemetry.addData("ShooterRPM:", Turret.getMotorType().getTicksPerRev() * 60 / 28);
                    telemetry.update();


                    idle();

                }
            }


