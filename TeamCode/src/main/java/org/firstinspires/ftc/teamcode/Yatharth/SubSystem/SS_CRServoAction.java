package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class SS_CRServoAction implements Action {
    CRServo crServo;
    double power;
    int seconds;
    ElapsedTime timer= null;

    public SS_CRServoAction(CRServo c, double p, int s) {
        this.crServo = c;
        this.seconds = s;
        this.power = p;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer.seconds() < seconds) {
            timer = new ElapsedTime();
            crServo.setPower(power);
        }
        return false;
    }
}
