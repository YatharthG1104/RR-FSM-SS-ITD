package org.firstinspires.ftc.teamcode.YatharthCodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.camera2.params.BlackLevelPattern;
import android.view.View;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Krunaal.ColorSensorDistanceTouch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Rev3_ColorD_Sensor implements Action {
    public RevColorSensorV3 colorSensor;
    public CRServo turningServo;
    public double position;
    public Servo servo;
    public int alliance_color;

    // Thresholds
    public static int LINE_THRESHOLD = 100;  //color intensity
    public static int YELLOW_THRESHOLD = 50;  //color intensity
    public static int BLUE_THRESHOLD = 50;
    public static double MIN_DISTANCE_THRESHOLD = 5.0; // in cm

    public void AlignSensorActionB(RevColorSensorV3 colorSensor, CRServo turningServo, Servo srv, double pos, int all_col) {
        this.colorSensor = colorSensor;
        this.turningServo = turningServo;
        this.position = pos;
        this.servo = srv;
        this.alliance_color = all_col;

    };
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        // Get RGB values
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Get the current distance measurement
        double distance = colorSensor.getDistance(DistanceUnit.CM);

        boolean isYellow = (red > LINE_THRESHOLD && green > YELLOW_THRESHOLD && blue < LINE_THRESHOLD);
        boolean isBlue = (red < BLUE_THRESHOLD && green < BLUE_THRESHOLD && blue > LINE_THRESHOLD);
        boolean isAligned = (red > LINE_THRESHOLD && green < LINE_THRESHOLD && blue < LINE_THRESHOLD);
        boolean isWithinDist = (distance <= MIN_DISTANCE_THRESHOLD);

        if (isAligned) {
            turningServo.setPower(0);
        } else {
            turningServo.setPower(1);
        }

        if(isWithinDist) {
            turningServo.setPower(0);
         } else {
        turningServo.setPower(1);
         }

        if ((isYellow || isBlue) && isAligned && isWithinDist) {
            servo.setPosition(position);
            return true;
        } else
            return false;
    }

}
