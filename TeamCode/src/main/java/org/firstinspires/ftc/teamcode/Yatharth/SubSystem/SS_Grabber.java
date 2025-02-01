package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// Wrist - Operation Sub-system
// Defines Wrist Servo and its behavior
//  1. Intake Speed
// 2. Outtake Speed

public class SS_Grabber {
    public static CRServo GrabLeft;
    public static CRServo GrabRight;

    //Define all Grab speed
    public static double GrabL_Speed = -0.8;
    public static double GrabR_Speed = 0.8;
    public static int RunTime = 2;
    
    // Define hardware properties
    public SS_Grabber(HardwareMap hardwareMap) {
        GrabLeft = hardwareMap.get(CRServo.class, "Grab Left");
        GrabRight = hardwareMap.get(CRServo.class, "Grab Right");
    }
    /******************/
    //Action 1a. Left Intake
    public static class LeftGrabberIntake implements Action {
        ElapsedTime timer;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer.seconds() < RunTime) {
                timer = new ElapsedTime();
                GrabLeft.setPower(GrabL_Speed);
            }
            return false;
        }
    }

    //Action 1b. Right Intake
    public static class RightGrabberIntake implements Action {
        ElapsedTime timer;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(timer.seconds() < RunTime) {
                timer = new ElapsedTime();
                GrabRight.setPower(GrabR_Speed);
            }
            return false;
        }
    }

    /******************/
    //Action 2a. Left Outtake
    public static class LeftGrabberOuttake implements Action {
        ElapsedTime timer;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer.seconds() < RunTime) {
                timer = new ElapsedTime();
                GrabLeft.setPower(GrabR_Speed);
            }
            return false;
        }
    }

    //Action 2b. Right Outtake
    public static class RightGrabberOuttake implements Action {
        ElapsedTime timer;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(timer.seconds() < RunTime) {
                timer = new ElapsedTime();
                GrabRight.setPower(GrabL_Speed);
            }
            return false;
        }
    }
}
