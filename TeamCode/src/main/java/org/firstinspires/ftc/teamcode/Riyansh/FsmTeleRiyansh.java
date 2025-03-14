package org.firstinspires.ftc.teamcode.Riyansh;/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

//@TeleOp(name="FSM tele Riyansh")
public class FsmTeleRiyansh extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        RetractAll,//retract all sorts of arm etc
        SubmersibleReady,//aka picking up a block with the slide
        SamplePicked,
        ClawReadyForSample,//transfer of sample
        FinishedBasket,
        SampleDrop,
        SpecimenPicked,
        SpecimenHanged,

        LIFT_START, FINAL
    }

    ;

    LiftState liftState = LiftState.LIFT_START;

    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));//set a inistal pose but will need to change this
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//    TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)//this is our goal pose with our goal heading
//            .lineToX(25)
//            .splineTo(new Vector2d(52, -52), Math.PI / 2)
//            .waitSeconds(3);
//
//    Pose2d initialPose2 = new Pose2d(50, -60, Math.toRadians(90));//set a inistal pose but will need to change this
//
//    TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)//this is our goal pose with our goal heading
//            .lineToX(25)
//            .splineTo(new Vector2d(52, -52), Math.PI / 2)
//            .waitSeconds(3);
//
//
//    Action trajectoryActionChosen = tab1.build();//seting our movment path to a varibale
//    Action trajectoryActionChosen2 = tab2.build();//seting our movment path to a varibale
    public DcMotorEx SlideFrontR;

    public DcMotorEx SlideBackR;
    public DcMotorEx SlideBackL;

    public Servo GecoWristL;
    public Servo GecoWristR;
    public CRServo Front_Rotate;
    public Servo Front_Claw;
    public Servo BackClaw;
    public Servo ClawArmR;
    public Servo ClawArmL;
    public Servo Wrist;

    ElapsedTime liftTimer = new ElapsedTime();

    double Geco_Wrist_Reset = 0.7;//geco wrist is in the middle position
    double Geco_Wrist_Down = -0.87;
    double Geco_Wrist_Claw_Deposit=0.85;


    double FrontClawOpen=-1;
    double FrontClawClose=1;


    double BackClaw_Open=-1;
    double BackClaw_Close=1;


    double Claw_Arm_Specimen_Pick = 0.5;
    double Claw_Arm_Specimen_Drop = 0.5;
    double Claw_Arm_Bucket=0.9;

    double Claw_Arm_Reset = -0.58;

//    double wrist_Specimen_Pick;
//    double getWrist_Specimen_Drop;

    int Back_Slide_Hang; // the high encoder position for the lift
    int Back_slide_Reset; // the high encoder position for the lift
    int Back_slide_Bucket; // the high encoder position for the lift
    int Back_slide_Hang;
    double Front_SLide_in;
    double Front_SLide_out;
    double lfPower;
    double rfPower;
    double rbPower;
    double lbPower;
    double Tp;

//    ColorSensor sensorColor;
//    DistanceSensor sensorDistance;
//    float hsvValues[] = {0F, 0F, 0F};
//
//    // values is a reference to the hsvValues array.
//    final float values[] = hsvValues;
//
//    // sometimes it helps to multiply the raw RGB values with a scale factor
//    // to amplify/attentuate the measured values.
//    final float SCALE_FACTOR = 255;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.

    public void init() {
        liftTimer.reset();

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        SlideFrontR = hardwareMap.get(DcMotorEx.class, "Front Slide");

        SlideBackR = hardwareMap.get(DcMotorEx.class, "Delivery ArmR");
        SlideBackL = hardwareMap.get(DcMotorEx.class, "Delivery ArmL ");//defining the names for all motors and servos in driver hub


        GecoWristL = hardwareMap.get(Servo.class, "Twist Left");
        GecoWristR = hardwareMap.get(Servo.class, "Twist Right");

        Front_Rotate = hardwareMap.get(CRServo.class, "Front Wrist");

        Front_Claw = hardwareMap.get(Servo.class, "Front Claw");

        BackClaw = hardwareMap.get(Servo.class, "Claw");

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        ClawArmR = hardwareMap.get(Servo.class, "Elbow Right");
        ClawArmL = hardwareMap.get(Servo.class, "Elbow Left");
//        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        telemetry.addData("Left Distance (cm)",
//                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//
//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        SlideFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SlideBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reseting the encoders
        SlideBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//ensuring that we are using the encoders

        SlideBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        GecoWristL.setPosition(Geco_Wrist_Reset);
        GecoWristR.setPosition(-Geco_Wrist_Reset);
        BackClaw.setPosition(BackClaw_Close);
        ClawArmL.setPosition(Claw_Arm_Reset);
        ClawArmR.setPosition(-Claw_Arm_Reset);
        //in the init we are making sure evreything is reset


    }

    public void loop() {
//        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                (int) (sensorColor.green() * SCALE_FACTOR),
//                (int) (sensorColor.blue() * SCALE_FACTOR),
//                hsvValues);
//        double hue = hsvValues[0];
//        FrontSLidePower = Range.clip(LeftStick, -1.0, 1.0);


        switch (liftState) {
            case RetractAll:
                if (gamepad2.y) {//set to what color it sees when sample is out of geco wheels
//                    ClawElbow.setPosition(Claw_Arm_Reset);
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(BackClaw,BackClaw_Open),
                            new MotorAction(SlideBackL,-Back_slide_Reset,0.4),
                            new MotorAction(SlideBackR,Back_slide_Reset,0.4),
                            new ServoAction(ClawArmL,-Claw_Arm_Reset),
                            new ServoAction(ClawArmR,Claw_Arm_Reset),
                            new ServoAction(GecoWristL,-Geco_Wrist_Reset),
                            new ServoAction(GecoWristR,Geco_Wrist_Reset)
                    ));
                    liftState = LiftState.SubmersibleReady;

                    //resets all movment items to ensure nothing wrong has happend
                }
                break;
            case SubmersibleReady:
                // otherwise let the driver drive
                if (gamepad2.x) {
                    // x is pressed, start extending
//                    double dd = gamepad2.right_stick_x;
//                    double cp = Range.clip(dd, -0.5, 0.5);//change it to encoder value

                    //allowing oporater to control the distance of the front slide
                    Actions.runBlocking(new ParallelAction(

                            new MotorAction(SlideFrontR,Front_SLide_out,0.5)
                    ));
                    liftState = LiftState.SamplePicked;//changing state so we can move to the next task
                }
                break;
            case SamplePicked:
                move();
                drive.leftFront.setPower(lfPower);
                drive.rightFront.setPower(rfPower);
                drive.leftBack.setPower(lbPower);
                drive.rightBack.setPower(rbPower);
                TwistControl();
                Front_Rotate.setPower(Tp);

                if (gamepad2.b) {//its set b so that it only goes down when the driver and oporater have aligned with the sampele
                    // set the lift dump to dump
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(GecoWristL,Geco_Wrist_Down),
                            new ServoAction(GecoWristR,-Geco_Wrist_Down),
                            new ServoAction(Front_Claw,FrontClawClose)

                    ));

                    liftTimer.reset();
                    liftState = LiftState.ClawReadyForSample;
                    //this grabes the sample that we were attempting to grab
                }
                break;


            case ClawReadyForSample:
//                hue >= 0 && hue < 60 || hue >= 60 && hue < 100 ||hue >= 200 && hue < 250
                if (gamepad2.dpad_right) {//If we missed or got wrong color we dont click it so we dont get a penelty
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(GecoWristL,Geco_Wrist_Claw_Deposit),
                            new ServoAction(GecoWristR,-Geco_Wrist_Claw_Deposit),
                            new MotorAction(SlideFrontR,Front_SLide_in,0.3),
                            new ServoAction(BackClaw,BackClaw_Open),
                            new SequentialAction(
                                    new ServoAction(Front_Claw,FrontClawOpen)
                            )
                    ));
                    liftState = LiftState.SampleDrop;

                    //We have captured the sample and now we are ready to do the transfer from geco wheels to claw

                }
                if(gamepad1.dpad_left) {//if we missed the sample or got the oponets color we dont click anything
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(Front_Claw, FrontClawOpen)

                    ));

                    liftState = LiftState.SamplePicked;




                break;
                //resets all movment things to get ready to attempt to grab a diffrent sample
            }



            case SampleDrop:
                if (gamepad2.a) {
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(BackClaw,BackClaw_Close),


                            new ServoAction(ClawArmL,-Claw_Arm_Specimen_Pick),
                            new ServoAction(ClawArmR,Claw_Arm_Specimen_Pick)
                    ));
                    //does the transfer and the claw is in position to drop in obsevtary zone or do bucket
                    move();
                    drive.leftFront.setPower(lfPower);
                    drive.rightFront.setPower(rfPower);
                    drive.leftBack.setPower(lbPower);
                    drive.rightBack.setPower(rbPower);
                    if (gamepad2.left_bumper) {//click left bumber if we are doing specimen
                        Actions.runBlocking(new ParallelAction(

                                new ServoAction(BackClaw,BackClaw_Open)
                        ));

                        liftState = LiftState.SpecimenHanged;
                        //drops sample in obsevatrey zone
                    }

                    if (gamepad2.right_bumper) {//click right bumber if we are doing basket

                        Actions.runBlocking(new ParallelAction(
                                //let driver contorl

                                new MotorAction(SlideBackL,Back_slide_Bucket,0.4),
                                new MotorAction(SlideBackR,-Back_slide_Bucket,0.4),
                                new ServoAction(ClawArmL,Claw_Arm_Bucket),
                                new ServoAction(ClawArmR,-Claw_Arm_Bucket)
                        ));
                        liftState = LiftState.FinishedBasket;
                        //ready to drop off the sample at bucket slide is rasied to correct hight
                    }


                }
                break;
            case FinishedBasket:
                if (gamepad1.left_bumper) {
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(BackClaw,BackClaw_Open)
                    ));
                    liftState = LiftState.RetractAll;
                    //drops sample in basket and retracts evreything
                }
                break;

            case SpecimenPicked:
                move();
                drive.leftFront.setPower(lfPower);
                drive.rightFront.setPower(rfPower);
                drive.leftBack.setPower(lbPower);
                drive.rightBack.setPower(rbPower);
                if (gamepad2.dpad_up) {//waiting for humen can change to an if statment

                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(BackClaw,BackClaw_Close),
                            new MotorAction(SlideBackL,-Back_Slide_Hang,0.4),
                            new MotorAction(SlideBackR,Back_Slide_Hang,0.4)



                    ));


                    //add moving code to pick up specimen
                    liftState = LiftState.SpecimenHanged;
                    //picks up specimen from obsevatry zone
                }
                break;
            case SpecimenHanged:
                move();
                drive.leftFront.setPower(lfPower);
                drive.rightFront.setPower(rfPower);
                drive.leftBack.setPower(lbPower);
                drive.rightBack.setPower(rbPower);
                if (gamepad1.a) {//waiting to ensure no ones in the way
                    Actions.runBlocking(
                            new ParallelAction(
                                    new MotorAction(SlideBackL, Back_Slide_Hang,0.4),
                                    new MotorAction(SlideBackR, -Back_slide_Reset,0.4),

                                    new SequentialAction(
                                        new ServoAction(BackClaw,BackClaw_Open),
                                            new ParallelAction(
                                                    new MotorAction(SlideBackL, Back_slide_Reset,0.4),
                                                    new MotorAction(SlideBackR, -Back_slide_Reset,0.4)
                                            )
                                    )
                            )

                    );

                }
                liftState = LiftState.SpecimenHanged;
                break;
        }

    }
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer= null;

        public ServoAction(Servo srv, double pos) {
            this.servo= srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return false;
        }
    }

    // Motor Action build for roadrunner to use
    public static class MotorAction implements Action {
        DcMotor motor;
        double position_tgt;
        double power;
        ElapsedTime timer = null;

        public MotorAction (DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return false;
        }
    }
    public void move(){
        double y = -gamepad1. left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        lfPower = (y + x + rx) / denominator;
        lbPower = (y - x + rx) / denominator;
        rfPower = (y - x - rx) / denominator;
        rbPower = (y + x - rx) / denominator;
    }

    //CRServo Action build for roadrunner to use
    public static class CRServoAction implements Action {
        CRServo crServo;
        double power;
        int seconds;
        ElapsedTime timer= null;

        public CRServoAction(CRServo crsrv, int sec, double pow ) {
            this.crServo = crsrv;
            this.seconds = sec;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < seconds) {
                timer = new ElapsedTime();
                crServo.setPower(power);
            }
            return false;
        }
    }
    public static class MotorPowerAction implements Action {
        DcMotor motor;
        double power;
        ElapsedTime timer = null;

        public MotorPowerAction(DcMotor mot, double pow) {
            this.motor = mot;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                motor.setPower(power);
            }
            return false;
        }
    }
    public void TwistControl(){
        double Gs = gamepad2.left_stick_x;//Gs - Gamepad stick
        Tp = Range.clip(Gs, -0.5, 0.5);

    }
}