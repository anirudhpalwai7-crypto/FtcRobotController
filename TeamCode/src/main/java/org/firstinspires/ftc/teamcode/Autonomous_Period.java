package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class Autonomous_Period extends LinearOpMode {
    ElapsedTime seconds = new ElapsedTime();
    private DcMotorEx front_left_motor, back_left_motor, back_right_motor, front_right_motor, Intake;
    private static final double TICKS_PER_REV = 537.7;

    // Wheel diameter in inches (ex: 96mm wheel = 3.78 in)
    private static final double WHEEL_DIAMETER_IN = 3.78;

    // External gear ratio (1.0 if direct drive)
    private static final double GEAR_RATIO = 1.0;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);
    private double ticksPerRev;
    private IMU imu;

    @Override
    public void runOpMode() {
    front_left_motor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
    back_left_motor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
    back_right_motor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
    front_right_motor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
    Intake = hardwareMap.get(DcMotorEx.class, "intake");
    imu = hardwareMap.get(IMU.class, "imu");


    front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    sleep(100);

    front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sleep(100);

    front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
      /*  RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot, usbFacingDirection, logoFacingDirection));
        */

    double ticksPerRev = front_left_motor.getMotorType().getTicksPerRev();

        waitForStart();

        resetEncoders();
        setRunUsingEncoders();

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust if your hub orientation is different:
        RevHubOrientationOnRobot.LogoFacingDirection logoDir =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir)));

        telemetry.addLine("Ready. IMU calibrating/initializing...");
        telemetry.update();

        // Reset yaw to 0 so turns are easy
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        // ------------------ AUTON PATH EXAMPLE ------------------
        driveInches(24, 0.5, 4.0);   // forward 24"
      //  turnToHeading(90, 0.35, 3.0); // turn to +90 degrees (left)
       // driveInches(12, 0.4, 3.0);   // forward 12"
        // --------------------------------------------------------
        Intake.setPower(1.0);
        sleep(2000);
        Intake.setPower(0);
        stopAll();
    }

    // ------------------ Movement Helpers ------------------

    private void driveInches(double inches, double power, double timeoutSec) {
        int moveTicks = (int) Math.round(inches * TICKS_PER_INCH);

        int flTarget = front_left_motor.getCurrentPosition() + moveTicks;
        int frTarget = back_left_motor.getCurrentPosition() + moveTicks;
        int blTarget = back_right_motor.getCurrentPosition() + moveTicks;
        int brTarget = front_right_motor.getCurrentPosition() + moveTicks;

        front_left_motor.setTargetPosition(flTarget);
        back_left_motor.setTargetPosition(frTarget);
        back_right_motor.setTargetPosition(blTarget);
        front_right_motor.setTargetPosition(brTarget);

        setRunToPosition();

        power = Math.abs(power);
        setMotorPowers(power, power, power, power);

        double start = getRuntime();
        while (opModeIsActive()
                && (getRuntime() - start) < timeoutSec
                && (front_left_motor.isBusy() || back_left_motor.isBusy() || back_right_motor.isBusy() || front_right_motor.isBusy())) {

            telemetry.addData("Drive", "inches=%.1f power=%.2f", inches, power);
            telemetry.addData("Pos", "FL %d | FR %d", front_left_motor.getCurrentPosition(), back_left_motor.getCurrentPosition());
            telemetry.update();
        }

        stopAll();
        setRunUsingEncoders();
        sleep(50);
    }

    /**
     * Turns robot to an absolute heading in degrees using IMU yaw.
     * headingDeg: target heading (-180..180). Example: 90 means facing left of start.
     */
   private void turnToHeading(double headingDeg, double maxPower, double timeoutSec) {
        double start = getRuntime();
        maxPower = Math.abs(maxPower);

        while (opModeIsActive() && (getRuntime() - start) < timeoutSec) {
            double current = getYawDeg();
            double error = angleWrap(headingDeg - current);

            // Stop when close enough
            if (Math.abs(error) < 2.0) break;

            // Simple P-control (tune kP)
            double kP = 0.012; // try 0.01 to 0.02
            double turn = Range.clip(error * kP, -maxPower, maxPower);

            // Turn in place: left motors opposite of right motors
            setMotorPowers(-turn, turn, -turn, turn);

            telemetry.addData("Turn", "target=%.1f current=%.1f error=%.1f", headingDeg, current, error);
            telemetry.addData("turnPower", "%.2f", turn);
            telemetry.update();
        }

        stopAll();
        sleep(50);
    }

    // ------------------ Hardware Helpers ------------------

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        front_left_motor.setPower(fl);
        back_left_motor.setPower(fr);
        back_right_motor.setPower(bl);
        front_right_motor.setPower(br);
    }

    private void stopAll() {
        setMotorPowers(0, 0, 0, 0);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        front_left_motor.setZeroPowerBehavior(b);
        back_left_motor.setZeroPowerBehavior(b);
        back_right_motor.setZeroPowerBehavior(b);
        front_right_motor.setZeroPowerBehavior(b);
    }

    private void resetEncoders() {
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
    }

    private void setRunUsingEncoders() {
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPosition() {
        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

   private double getYawDeg() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    /** Wraps angle to [-180, 180) */
    private double angleWrap(double deg) {
        while (deg >= 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }



        }

