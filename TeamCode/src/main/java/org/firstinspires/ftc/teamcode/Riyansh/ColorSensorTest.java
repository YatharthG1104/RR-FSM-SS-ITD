package org.firstinspires.ftc.teamcode.Riyansh;/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Locale;

@TeleOp(name="color")
public class ColorSensorTest extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)


    public CRServo GecoWheelsR;
    public CRServo GecoWheelsL;
    double Geco_IN =1;
    double Geco_OUT=-1;



    ElapsedTime liftTimer = new ElapsedTime();




    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final float SCALE_FACTOR = 255;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.

    public void init() {
        GecoWheelsR = hardwareMap.get(CRServo.class, "Grab Right");
        GecoWheelsL = hardwareMap.get(CRServo.class, "Grab Left");

        // hardware initialization code goes here
        // this needs to correspond with the configuration used



        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("Left Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);



        //in the init we are making sure evreything is reset


    }

    public void loop() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        double hue = hsvValues[0];
        telemetry.addData("hue...",hue);
        if (hue >= 0 && hue < 60  ) {//If we missed or got wrong color we dont click it so we dont get a penelty

            telemetry.addLine("Color detected red");
        }
        if (hue >= 60 && hue < 100 ) {//If we missed or got wrong color we dont click it so we dont get a penelty

            telemetry.addLine("Color detected yellow");
        }
        if(sensorDistance.getDistance(DistanceUnit.CM)>2){
            telemetry.addLine("nothing here missed");
        }
        if(hue >= 200 && hue < 250){
            telemetry.addLine("Color detected blue ");
        }
        telemetry.addData("Left Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

        telemetry.update();

    }
}