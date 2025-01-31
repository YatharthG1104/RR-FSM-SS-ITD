package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Elbow - Operation Sub-system for both servos
// Defines Elbow Servos and its behavior
//  1. ElbowLeft and ElbowRight Intake Position
//  2. ElbowLeft and ElbowRight Hang Position
//  3. ElbowLeft and ElbowRight Basket Delivery Position
//  4. ElbowLeft and ElbowRight Sample Transfer Position
//  5. ElbowLeft and ElbowRight Resting Position

public class SS_Elbow {
    public static Servo ElbowLeft;
    public static Servo ElbowRight;

    //Define all Elbow positions
    public static double ElbowL_Intake_Pos = -0.8;
    public static double ElbowR_Intake_Pos = 0.8;

    public static double ElbowL_Hang_Pos = -0.5;
    public static double ElbowR_Hang_Pos = 0.5;

    public static double ElbowL_Basket_Pos = -0.5;
    public static double ElbowR_Basket_Pos = 0.5;

    public static double ElbowL_Sample_Trnsf_Pos = -0.2;
    public static double ElbowR_Sample_Trnsf_Pos = 0.2;

    public static double ElbowL_Rest_Pos = 0.0;
    public static double ElbowR_Rest_Pos = 0.0;

    // Define hardware properties
    public SS_Elbow(HardwareMap hardwareMap) {
        ElbowLeft = hardwareMap.get(Servo.class, "Elbow Left");
        ElbowRight = hardwareMap.get(Servo.class, "Elbow Right");

        ElbowLeft.scaleRange(0,1);
        ElbowRight.scaleRange(-1,0);
        ElbowLeft.setDirection(Servo.Direction.REVERSE);
        ElbowRight.setDirection(Servo.Direction.FORWARD);
    }
/******************/
    //Action 1a.The Elbow Left Intake Position action
    public static class ElbowLeftIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowLeft.setPosition(ElbowL_Intake_Pos);
            double posl = ElbowLeft.getPosition();
            packet.put("ElbowL pos", posl);
            return false;
        }
    }
    //Action 1b.The Elbow Right Intake Position action
    public static class ElbowRightIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowRight.setPosition(ElbowR_Intake_Pos);
            double posr = ElbowRight.getPosition();
            packet.put("ElbowR pos", posr);
            return false;
        }
    }

/*******************/
    //Action 2a.The Elbow Left Hang Position action
    public static class ElbowLeftHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowLeft.setPosition(ElbowL_Hang_Pos);
            double posl = ElbowLeft.getPosition();
            packet.put("ElbowL pos", posl);
            return false;
        }
    }
    //Action 2b.The Elbow Right Hang Position action
    public static class ElbowRightHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowRight.setPosition(ElbowR_Hang_Pos);
            double posr = ElbowRight.getPosition();
            packet.put("ElbowR pos", posr);
            return false;
        }
    }

/*********************/
    //Action 3a.The Elbow Left Basket Delivery Position action
    public static class ElbowLeftBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowLeft.setPosition(ElbowL_Basket_Pos);
            double posl = ElbowLeft.getPosition();
            packet.put("ElbowL pos", posl);
            return false;
        }
    }
    //Action 3b.The Elbow Right Basket Delivery Position action
    public static class ElbowRightBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowRight.setPosition(ElbowR_Basket_Pos);
            double posr = ElbowRight.getPosition();
            packet.put("ElbowR pos", posr);
            return false;
        }
    }

 /********************/
    //Action 4a.The Elbow left Sample Transfer Position action
    public static class ElbowLeftTrnsf implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowLeft.setPosition(ElbowL_Sample_Trnsf_Pos);
            double posl = ElbowLeft.getPosition();
            packet.put("ElbowL pos", posl);
            return false;
        }
    }
    //Action 3b.The Elbow Right Sample Transfer Position action
    public static class ElbowRightTrnsf implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowRight.setPosition(ElbowR_Sample_Trnsf_Pos);
            double posr = ElbowRight.getPosition();
            packet.put("ElbowR pos", posr);
            return false;
        }
    }

 /****************/
    //Action 5a.The Elbow left Resting Position action
    public static class ElbowLeftRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowLeft.setPosition(ElbowL_Rest_Pos);
            double posl = ElbowLeft.getPosition();
            packet.put("ElbowL pos", posl);
            return false;
        }
    }
    //Action 5b.The Elbow Right Resting Position action
    public static class ElbowRightRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ElbowRight.setPosition(ElbowR_Rest_Pos);
            double posr = ElbowRight.getPosition();
            packet.put("ElbowR pos", posr);
            return false;
        }
    }
}