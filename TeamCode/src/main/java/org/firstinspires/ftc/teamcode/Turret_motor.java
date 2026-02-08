package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Turret_motor extends OpMode {
    private DcMotorEx Turret;
    int MAXRPM = 6_000;
    int MINRPM = 0;
    int CURRENT_RPM = 3_000;
    double motorTicks;

    @Override
    public void init(){
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double motorTicks = Turret.getMotorType().getTicksPerRev();
        double MAXRPM = Turret.getMotorType().getMaxRPM();


    }
    @Override
    public void loop() {
        telemetry.addData("MAXIMUM RPM:", MAXRPM);
        telemetry.addData("TICKS:", motorTicks);
        CURRENT_RPM = Range.clip(CURRENT_RPM, 0, 6_000);
    }
}
