package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Wrist - Operation Sub-system
// Defines Wrist Servo and its behavior
//  1. Wrist Intake Position
//  2. Wrist Hang Position
//  3. Wrist Basket Delivery Position
//  4. Wrist Sample Transfer Position
//  5. Wrist Resting Position

public class SS_Wrist {
    public static Servo Wrist;

    //Define all Wrist positions 
    public static double Wrist_Intake_Pos = 0.3;
    public static double Wrist_Hang_Pos = 0.95;
    public static double Wrist_Basket_Pos = 0.5;
    public static double Wrist_Sample_Trnsf_Pos = 0.3;
    public static double Wrist_Rest_Pos = 0.3;

    // Define hardware properties
    public SS_Wrist(HardwareMap hardwareMap) {
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Wrist.scaleRange(0,1);
        Wrist.setDirection(Servo.Direction.FORWARD);
    }

    //Action 1.The Wrist Intake Position action
    public static class WristIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Wrist.setPosition(Wrist_Intake_Pos);
            double pos = Wrist.getPosition();
            packet.put("Wrist pos", pos);
            return false;
            }
        }

    //Action 2.The Wrist Hang Position action
    public static class WristHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Wrist.setPosition(Wrist_Hang_Pos);
            double pos = Wrist.getPosition();
            packet.put("Wrist pos", pos);
            return false;
        }
    }

    //Action 3.The Wrist Basket Position action
    public static class WristBasket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Wrist.setPosition(Wrist_Basket_Pos);
            double pos = Wrist.getPosition();
            packet.put("Wrist pos", pos);
            return false;
        }
    }

    //Action 4.The Wrist Sample Transfer Position action
    public static class WristTransfer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Wrist.setPosition(Wrist_Sample_Trnsf_Pos);
            double pos = Wrist.getPosition();
            packet.put("Wrist pos", pos);
            return false;
        }
    }

    //Action 5.The Wrist Resting Position action
    public static class WristRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Wrist.setPosition(Wrist_Rest_Pos);
            double pos = Wrist.getPosition();
            packet.put("Wrist pos", pos);
            return false;
        }
    }
    }
