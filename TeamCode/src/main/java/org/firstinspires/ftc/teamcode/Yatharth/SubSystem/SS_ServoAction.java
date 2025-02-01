package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SS_ServoAction implements Action{
    Servo servo;
    double position_tgt;
    double direction;
    ElapsedTime timer;

    public SS_ServoAction(Servo s, double p, double d) {
        this.servo = s;
        this.position_tgt = p;
        this.direction = d;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (timer == null) {
            timer = new ElapsedTime();
            servo.setPosition(position_tgt * direction);
        }
        return false;
    }
}
