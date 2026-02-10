package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Teleop extends LinearOpMode {
    DrivetrainGO mecanum = new DrivetrainGO();

    private DcMotor Intake, Transfer;
    private DcMotorEx Turret;
    private Servo back_servo;
    int targetRPM = 2_900;
    boolean dpadUpLatch = false;
    boolean dpadDownLatch = false;

    boolean trianglePressed = false;
    double Intake_RPM;
    double Transfer_RPM;
    double Turret_RPM;
    boolean slowMode = false;

    double speedExtra;
    static final double TICKS_PER_REV = 28.0;

    double getTurretRPM() {
        return Turret.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    @Override
    public void runOpMode() {

        mecanum.init(hardwareMap);
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake_RPM = Intake.getMotorType().getAchieveableMaxTicksPerSecond() * 60 / 28;
        Transfer_RPM = Transfer.getMotorType().getAchieveableMaxTicksPerSecond() * 60 / 28;
        Turret_RPM = Turret.getMotorType().getAchieveableMaxTicksPerSecond() * 60 / 28;
        back_servo = hardwareMap.get(Servo.class, "back_servo");
        waitForStart();

        // PID constants (TUNE THESE)
        double kP = 0.0004;
        double kI = 0.0000008;
        double kD = 0.0005;

        double turretIntegral = 0;
        double turretLastError = 0;
        long lastTime = 0;


        while (opModeIsActive()) {

            double turn = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.right_stick_x;
            mecanum.drive(forward,strafe,turn);


            if (gamepad1.left_bumper) {
                slowMode = !slowMode;
            }

            speedExtra = slowMode ? 0.2 : 1;

            forward *= speedExtra;
            strafe *= speedExtra;


            telemetry.addData("forward speed :", forward);
            telemetry.addData("strafe speed:", strafe);
            telemetry.addData("turn speed:", strafe);


            if (gamepad2.dpad_up && !dpadUpLatch) {
                targetRPM += 100;
                dpadUpLatch = true;

            }
            if (!gamepad2.dpad_up) {
                dpadUpLatch = false;
            }

            if (gamepad2.dpad_down && !dpadDownLatch) {
                targetRPM -= 100;
                dpadDownLatch = true;
            }
            if (!gamepad2.dpad_down) {
                dpadDownLatch = false;
            }
            targetRPM = Range.clip(targetRPM, 1_600, 4_900);
            telemetry.addData("TARGET RPM:", targetRPM);



            if (gamepad2.right_bumper) {
                Intake.setPower(0.75);
            }
            else if(gamepad2.right_trigger > 0) {
                Intake.setPower(-0.75);
            } else {
                Intake.setPower(0);
            }

            if (gamepad2.left_bumper) {
                Transfer.setPower(1);
            } else if (gamepad2.left_trigger > 0){
                Transfer.setPower(-1);
            } else {
                Transfer.setPower(0);
            }

            if (gamepad2.triangleWasPressed()) {
                trianglePressed = !trianglePressed;
            }
            if (trianglePressed) {
                double currentRPM = getTurretRPM();
                double error = targetRPM - currentRPM;

                long now = System.nanoTime();
                double deltaTime = (now - lastTime) / 1e9;
                lastTime = now;

                turretIntegral += error * deltaTime;
                double derivative = (error - turretLastError) / deltaTime;
                turretLastError = error;

                double output =
                        (kP * error) +
                                (kI * turretIntegral) +
                                (kD * derivative);

                output = Range.clip(output, 0, 1);

                Turret.setPower(output);

                /*double targetTicksPerSecond = targetRPM * TICKS_PER_REV / 60.0;
                Turret.setVelocity(targetTicksPerSecond);*/
            } else {
                //Turret.setVelocity(0);
                Turret.setPower(0);
                turretIntegral = 0;
                turretLastError = 0;
            }
            if (gamepad2.circle) {
                back_servo.setPosition(0);

            } else {
                back_servo.setPosition(0.3);
            }
            //telemetry.addData("Shooter RPM",Turret.getVelocity() * 60.0 / TICKS_PER_REV);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", getTurretRPM());
            telemetry.addData("Error", targetRPM - getTurretRPM());
            //telemetry.addData("Shooter RPM:", Turret_RPM * Turret.getPower() );

            telemetry.addData("Intake RPM:", Intake_RPM * Intake.getPower());
            telemetry.addData("Transfer RPM:", Transfer_RPM * Transfer.getPower());


            telemetry.update();
        }
    }
}