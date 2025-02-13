package org.firstinspires.ftc.teamcode.YatharthCodes;

/*
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

@TeleOp(name="TeleOps FSM YG")

public class FSM_TeleOps extends OpMode {
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
        LIFT_START,
        FINAL
    }

    ;

    LiftState liftState = LiftState.LIFT_START;

    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));//set a inistal pose but will need to change this
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)//this is our goal pose with our goal heading
            .lineToX(25)
            .splineTo(new Vector2d(52, -52), Math.PI / 2)
            .waitSeconds(3);

    Pose2d initialPose2 = new Pose2d(50, -60, Math.toRadians(90));//set a inistal pose but will need to change this

    TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)//this is our goal pose with our goal heading
            .lineToX(25)
            .splineTo(new Vector2d(52, -52), Math.PI / 2)
            .waitSeconds(3);


    Action trajectoryActionChosen = tab1.build();//seting our movment path to a varibale
    Action trajectoryActionChosen2 = tab2.build();//seting our movment path to a varibale
    public DcMotorEx SlideFrontR;
    public DcMotorEx SlideFrontL;
    public DcMotorEx SlideBackR;
    public DcMotorEx SlideBackL;

    public Servo GecoWristL;
    public Servo GecoWristR;
    public CRServo GecoWheelsR;
    public CRServo GecoWheelsL;
    public Servo Claw;
    public Servo ClawArmR;
    public Servo ClawArmL;
    public Servo Wrist;

    ElapsedTime liftTimer = new ElapsedTime();

    double Geco_Wrist_Reset;//geco wrist is in the middle position
    double Geco_Wrist_Down;
    double Geco_Wrist_Claw_Deposit;


    double Geco_IN;
    double Geco_OUT;


    double Claw_Open;
    double Claw_Close;


    double Claw_Arm_Specimen_Pick = 0.5;
    double Claw_Arm_Transfer = -0.5;
    double Claw_Arm_Reset = -0.3;

    double wrist_Specimen_Pick;
    double getWrist_Specimen_Drop;


    int Back_SLide_Up_Specimen; // the low encoder position for the lift
    int Back_Slide_Hang; // the high encoder position for the lift
    int Back_slide_Reset; // the high encoder position for the lift
    int Back_slide_Bucket; // the high encoder position for the lift
    int Front_SLide_in;
    double FrontSLidePower;
    double LeftStick = -gamepad2.left_stick_x;
    double lfPower;
    double rfPower;
    double rbPower;
    double lbPower;

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
        liftTimer.reset();

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        SlideFrontR = hardwareMap.get(DcMotorEx.class, "Rli");
        SlideFrontL = hardwareMap.get(DcMotorEx.class, "Lli");
        SlideBackR = hardwareMap.get(DcMotorEx.class, "rback");
        SlideBackL = hardwareMap.get(DcMotorEx.class, "lback");//defining the names for all motors and servos in driver hub


        GecoWristL = hardwareMap.get(Servo.class, "lm");
        GecoWristR = hardwareMap.get(Servo.class, "rm");
        GecoWheelsR = hardwareMap.get(CRServo.class, "fr");
        GecoWheelsL = hardwareMap.get(CRServo.class, "fl");
        Claw = hardwareMap.get(Servo.class, "Claw");
        ClawArmR = hardwareMap.get(Servo.class, "BRM");
        ClawArmL = hardwareMap.get(Servo.class, "BLM");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("Left Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        SlideFrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideFrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlideFrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideFrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlideBackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reseting the encoders
        SlideBackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//ensuring that we are using the encoders

        SlideBackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideBackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        GecoWristL.setPosition(-Geco_Wrist_Reset);
        GecoWristR.setPosition(Geco_Wrist_Reset);
        Claw.setPosition(Claw_Close);
        ClawArmL.setPosition(-Claw_Arm_Reset);
        ClawArmR.setPosition(Claw_Arm_Reset);
        //in the init we are making sure evreything is reset


    }

    public void loop() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        double hue = hsvValues[0];
        FrontSLidePower = Range.clip(LeftStick, -1.0, 1.0);


        switch (liftState) {
            case RetractAll:
                if (SlideBackL.getCurrentPosition() - Back_SLide_Up_Specimen < 30) {//set to what color it sees when sample is out of geco wheels
//                    ClawElbow.setPosition(Claw_Arm_Reset);
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(Claw,Claw_Open),
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

                if (Math.abs(SlideBackL.getCurrentPosition() - Back_slide_Reset) < 30 && gamepad2.x) {



                    // x is pressed, start extending
                    double dd = gamepad2.right_stick_x;

                    double cp = Range.clip(dd, -0.5, 0.5);

                    //allowing oporater to control the distance of the front slide
                    Actions.runBlocking(new ParallelAction(
                            new MotorPowerAction(SlideFrontL,-cp),
                            new MotorPowerAction(SlideFrontR,cp)
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

                if (gamepad2.b) {//its set b so that it only goes down when the driver and oporater have aligned with the sampele
                    // set the lift dump to dump
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(GecoWristL,-Geco_Wrist_Down),
                            new ServoAction(GecoWristR,Geco_Wrist_Down),
                            new CRServoAction(GecoWheelsL,1,-Geco_IN),
                            new CRServoAction(GecoWheelsR,1,Geco_IN)

                    ));

                    liftTimer.reset();
                    liftState = LiftState.ClawReadyForSample;
                    //this grabes the sample that we were attempting to grab
                }
                break;


            case ClawReadyForSample:
//                hue >= 0 && hue < 60 || hue > 360 || hue >= 60 && hue < 120
                if (hue >= 0 && hue < 60 || hue >= 60 && hue < 100 ||hue >= 200 && hue < 250) {//If we missed or got wrong color we dont click it so we dont get a penelty
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(GecoWristL,-Geco_Wrist_Claw_Deposit),
                            new ServoAction(GecoWristR,Geco_Wrist_Claw_Deposit),
                            new MotorAction(SlideFrontL,-Front_SLide_in,0.3),
                            new MotorAction(SlideFrontR,Front_SLide_in,0.3),
                            new ServoAction(Claw,Claw_Open)
                    ));
                    liftState = LiftState.SampleDrop;

                    //We have captured the sample and now we are ready to do the transfer from geco wheels to claw

                } if(sensorDistance.getDistance(DistanceUnit.CM)>2) {//if we missed the sample or got the oponets color we dont click anything
                Actions.runBlocking(new ParallelAction(
                        new ServoAction(GecoWristL,-Geco_Wrist_Reset),
                        new ServoAction(GecoWristR,Geco_Wrist_Reset),
                        new CRServoAction(GecoWheelsL,1,-Geco_OUT),
                        new CRServoAction(GecoWheelsR,1,Geco_OUT)
                ));

                liftState = LiftState.SubmersibleReady;

                //resets all movment things to get ready to attempt to grab a diffrent sample
            }
                break;


            case SampleDrop:
                if (Math.abs(SlideFrontR.getCurrentPosition() - Front_SLide_in) < 30 && gamepad2.a) {
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(Claw,Claw_Close),
                            new CRServoAction(GecoWheelsL,1,-Geco_OUT),
                            new CRServoAction(GecoWheelsR,1,Geco_OUT),
                            new ServoAction(ClawArmL,-Claw_Arm_Specimen_Pick),
                            new ServoAction(ClawArmR,Claw_Arm_Specimen_Pick)
                    ));
                    //does the transfer and the claw is in position to drop in obsevtary zone or do bucket

                    if (gamepad2.left_bumper) {//click left bumber if we are doing specimen
                        Actions.runBlocking(new ParallelAction(
                                new ParallelAction(trajectoryActionChosen),//might need to change to sequncal
                                new ServoAction(Claw,Claw_Open)
                        ));

                        liftState = LiftState.SpecimenHanged;
                        //drops sample in obsevatrey zone
                    }

                    if (gamepad2.right_bumper) {//click right bumber if we are doing basket
                        move();
                        drive.leftFront.setPower(lfPower);
                        drive.rightFront.setPower(rfPower);
                        drive.leftBack.setPower(lbPower);
                        drive.rightBack.setPower(rbPower);
                        Actions.runBlocking(new ParallelAction(
                                //let driver contorl

                                new MotorAction(SlideBackL,-Back_slide_Bucket,0.4),
                                new MotorAction(SlideBackR,Back_slide_Bucket,0.4)
                        ));
                        liftState = LiftState.FinishedBasket;
                        //ready to drop off the sample at bucket slide is rasied to correct hight
                    }


                }
                break;
            case FinishedBasket:
                if (Math.abs(SlideBackL.getCurrentPosition() - Back_slide_Bucket) < 30) {
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(Claw,Claw_Open)
                    ));
                    liftState = LiftState.RetractAll;
                    //drops sample in basket and retracts evreything
                }
                break;

            case SpecimenPicked:
                if (gamepad2.dpad_up) {//waiting for humen can change to an if statment
                    //add meep meep move
                    Actions.runBlocking(new ParallelAction(
                            new ServoAction(Claw,Claw_Close),
                            new MotorAction(SlideBackL,-Back_SLide_Up_Specimen,0.4),
                            new MotorAction(SlideBackR,-Back_SLide_Up_Specimen,0.4)
                    ));


                    //add moving code to pick up specimen
                    liftState = LiftState.SpecimenHanged;
                    //picks up specimen from obsevatry zone
                }
                break;
            case SpecimenHanged:
                if (Math.abs(SlideBackR.getCurrentPosition() - Back_SLide_Up_Specimen) < 30 && gamepad2.dpad_down) {//waiting to ensure no ones in the way
                    Actions.runBlocking(new SequentialAction(
                                    trajectoryActionChosen2,
                                    new MotorAction(SlideBackL, -Back_Slide_Hang,0.4),
                                    new MotorAction(SlideBackR, Back_Slide_Hang,0.4)
                            )
                    );//moving to hang specimen
//                    double y = -gamepad1. left_stick_y;
//                    double x = gamepad1.left_stick_x;
//                    double rx = gamepad1.right_stick_x;
//
//                    // Denominator is the largest motor power (absolute value) or 1
//                    // This ensures all the powers maintain the same ratio,
//                    // but only if at least one is out of the range [-1, 1]
//
//                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//
//                    lfPower = (y + x + rx) / denominator;
//                    lbPower = (y - x + rx) / denominator;
//                    rfPower = (y - x - rx) / denominator;
//                    rbPower = (y + x - rx) / denominator;


                }
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
}