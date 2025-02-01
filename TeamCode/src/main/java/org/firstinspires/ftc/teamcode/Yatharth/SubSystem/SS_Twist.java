package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Twist - Operation Sub-system
// Defines Twist Servo and its behavior
// Builds all potential Twist actions (add more for buckets, as needed)
//  1. Intake
//  2. Transfer
//  3. Rest


public class SS_Twist {
    public static Servo TwistLeft;
    public static Servo TwistRight;

    //Define all Twist positions
    public static double TwistL_Intake_Pos = -0.8;
    public static double TwistR_Intake_Pos = 0.8;
    
    public static double TwistL_Transfer_Pos = -0.5;
    public static double TwistR_Transfer_Pos = 0.5;

    public static double TwistL_Rest_Pos = 0.0;
    public static double TwistR_Rest_Pos = 0.0;

    // Define hardware properties
    public SS_Twist(HardwareMap hardwareMap) {
        TwistLeft = hardwareMap.get(Servo.class, "Twist Left");
        TwistRight = hardwareMap.get(Servo.class, "Twist Right");

        TwistLeft.scaleRange(0,1);
        TwistRight.scaleRange(-1,0);
        TwistLeft.setDirection(Servo.Direction.REVERSE);
        TwistRight.setDirection(Servo.Direction.FORWARD);
    }
    /******************/
    //Action 1a.The Twist Left Intake Position action
    public static class TwistLeftIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistLeft.setPosition(TwistL_Intake_Pos);
            double posl = TwistLeft.getPosition();
            packet.put("TwistL pos", posl);
            return false;
        }
    }
    //Action 1b.The Twist Right Intake Position action
    public static class TwistRightIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistRight.setPosition(TwistR_Intake_Pos);
            double posr = TwistRight.getPosition();
            packet.put("TwistR pos", posr);
            return false;
        }
    }

    /*******************/
    //Action 2a.The Twist Left Transfer Position action
    public static class TwistLeftTransfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistLeft.setPosition(TwistL_Transfer_Pos);
            double posl = TwistLeft.getPosition();
            packet.put("TwistL pos", posl);
            return false;
        }
    }
    //Action 2b.The Twist Right Transfer Position action
    public static class TwistRightTransfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistRight.setPosition(TwistR_Transfer_Pos);
            double posr = TwistRight.getPosition();
            packet.put("TwistR pos", posr);
            return false;
        }
    }
    /****************/
    //Action 3a.The Twist left Resting Position action
    public static class TwistLeftRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistLeft.setPosition(TwistL_Rest_Pos);
            double posl = TwistLeft.getPosition();
            packet.put("TwistL pos", posl);
            return false;
        }
    }
    //Action 3b.The Twist Right Resting Position action
    public static class TwistRightRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            TwistRight.setPosition(TwistR_Rest_Pos);
            double posr = TwistRight.getPosition();
            packet.put("TwistR pos", posr);
            return false;
        }
    }
}
