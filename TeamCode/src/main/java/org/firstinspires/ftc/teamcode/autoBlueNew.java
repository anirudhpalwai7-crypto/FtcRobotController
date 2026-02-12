package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class autoBlueNew extends LinearOpMode {
    DcMotorEx front_left_motor, back_left_motor, back_right_motor, front_right_motor;
    DcMotor Intake, Turret, Transfer;
    private IMU imu;
    private Servo back_servo;
    ElapsedTime runtime = new ElapsedTime();
    static final double TICKS_REV = 537.7;
    static final double WHEEL_DIAMETER = 4.0945;
    static final double COUNTS_INCH = (TICKS_REV * 1) / (WHEEL_DIAMETER * 3.1415);

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
        back_servo = hardwareMap.get(Servo.class, "back_servo");

        front_left_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        front_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        waitForStart();

        if(isStopRequested()) return;

        imu.resetYaw();

        //driving backwards
        driveStraight(0.6,44 );
        //driveLeft(0.5,5);
        stopDrive();
        //Transfer.setPower(-0.5);
        //while (opModeIsActive()) {
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
           /* if (imuAngle < 18) {
                front_left_motor.setPower(0.5);
                back_left_motor.setPower(0.5);
                back_right_motor.setPower(-0.5);
                front_right_motor.setPower(-0.5);
                telemetry.addData("Angle:", imuAngle);
            } else {
                front_left_motor.setPower(0);
                back_left_motor.setPower(0);
                back_right_motor.setPower(0);
                front_right_motor.setPower(0);
            }*/



           /* if (time.seconds() > 10 && time.seconds() <= 12) {
                Turret.setPower(0);
                front_left_motor.setPower(-0.5);
                back_left_motor.setPower(-0.5);
                back_right_motor.setPower(-0.5);
                front_right_motor.setPower(-0.5); */
        //back_servo.setPosition(1);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {

            Turret.setPower(-0.67);

            if (runtime.seconds() >= 4) {
                Intake.setPower(-0.8);
                Transfer.setPower(-0.6);
            }
        }


            telemetry.addData("Yaw:", imuAngle);
            telemetry.addData("ShooterRPM:", Turret.getMotorType().getMaxRPM() * Turret.getPower());
            telemetry.update();

        // Feed for 4 seconds
        sleep(4000);

    // Stop everything
        Turret.setPower(0);
       // Intake.setPower(0);
        Transfer.setPower(0);
        //back_servo.setPosition(1);

           // driveLeft(0.5, 24);   // strafe left 24 inches
        driveRight(0.5, 14.5);
            stopDrive();

        //collecting artifacts
        //Intake.setPower(-0.8);

        driveStraight(-0.6,-40);

        if (runtime.seconds() <= 1)
            Transfer.setPower(-0.4);

        //Intake.setPower(0);
        Transfer.setPower(0);
        //driveLeft(0.5,5);
        stopDrive();

        sleep(1000);

        driveStraight(0.6,36);
        stopDrive();

        driveLeft(0.5, 14.5);
        stopDrive();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            Turret.setPower(-0.67);
            if (runtime.seconds() >= 5) {
                Intake.setPower(-0.8);
                //Transfer.setPower(-0.4);
            }
            if (runtime.seconds() >= 2.5 && runtime.seconds() <= 3.5 ) {
               // Intake.setPower(-0.8);
                Transfer.setPower(-0.6);
            }
        }

        sleep(2000);

        Turret.setPower(0);
        Intake.setPower(0);
        Transfer.setPower(0);

        driveRight(0.5, 8);
        stopDrive();

        driveStraight(-0.6,-30);
        stopDrive();
        Intake.setPower(0);

        // telemetry.update();
    }

    public void driveStraight(double power, double inches) {
        int ticks = (int) (inches * 45);
        setDriveTarget(ticks, ticks, ticks, ticks);
        runToPosition(power,3);
    }

    public void driveRight(double power, double inches) {
        int ticks = (int) (inches * COUNTS_INCH);

        setDriveTarget(
                ticks,      // front left
                -ticks,     // front right
                ticks,     // back left
                -ticks       // back right
        );

        runToPosition(power, 3);
    }

    public void driveLeft(double power, double inches) {
        int ticks = (int) (inches * COUNTS_INCH);

        setDriveTarget(
                -ticks,     // front left
                ticks,      // front right
                -ticks,      // back left
                ticks      // back right
        );

        runToPosition(power, 3);
    }

    public void setDriveTarget(int lf, int rf, int lb, int rb) {
        front_left_motor.setTargetPosition(front_left_motor.getCurrentPosition() + lf);
        front_right_motor.setTargetPosition(front_right_motor.getCurrentPosition() + rf);
        back_left_motor.setTargetPosition(back_left_motor.getCurrentPosition() + lb);
        back_right_motor.setTargetPosition(back_right_motor.getCurrentPosition() + rb);

    }

    private void runToPosition(double power, double timeoutSeconds) {
        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left_motor.setPower(power);
        front_right_motor.setPower(power);
        back_left_motor.setPower(power);
        back_right_motor.setPower(power);

        double startTime = getRuntime();

       while(opModeIsActive() && (getRuntime() - startTime < timeoutSeconds)
                && (front_left_motor.isBusy()
                || front_right_motor.isBusy()
                || back_left_motor.isBusy()
                || back_right_motor.isBusy())) {
            telemetry.addData("LF",front_left_motor.getCurrentPosition());
            telemetry.addData("RF",front_right_motor.getCurrentPosition());
            telemetry.addData("BL",back_left_motor.getCurrentPosition());
            telemetry.addData("BF",back_right_motor.getCurrentPosition());


            telemetry.addData("Status","Moving");
            telemetry.update();
        }

        stopDrive();

        front_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    private void stopDrive() {
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }
    }