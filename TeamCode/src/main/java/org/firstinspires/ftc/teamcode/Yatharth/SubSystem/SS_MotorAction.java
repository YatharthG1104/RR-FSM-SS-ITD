package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SS_MotorAction implements Action {
    DcMotor motor;
    double position_tgt;
    double power;
    ElapsedTime timer;

    public SS_MotorAction(DcMotor m, double p, double pw) {
        this.motor = m;
        this.position_tgt = p;
        this.power = pw;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer == null) {
            timer = new ElapsedTime();
            motor.setPower(power);
            motor.setTargetPosition((int) (position_tgt));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        return false;
    }
    /*
    public Action SS_MotorAction(DcMotor m, double p, double pw) {
        return new SS_MotorAction(m,p,pw);
    }*/
}
