package org.firstinspires.ftc.teamcode.YatharthCodes;

import android.app.Activity;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="TeleOps FSM")

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

        LIFT_START, FINAL
    };

    LiftState liftState = LiftState.LIFT_START;

    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));//set a inistal pose but will need to change this
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)//this is our goal pose with our goal heading
            .lineToX(25)
            .splineTo(new Vector2d(52, -52), Math.PI /2)
            .waitSeconds(3);

    Pose2d initialPose2 = new Pose2d(50, -60, Math.toRadians(90));//set a inistal pose but will need to change this

    TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose2)//this is our goal pose with our goal heading
            .lineToX(25)
            .splineTo(new Vector2d(52, -52), Math.PI /2)
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
    int Back_slide_Reset; // the high encoder position for the lift
    int Back_slide_Bucket; // the high encoder position for the lift
    int Front_SLide_in;
    double FrontSLidePower;
    double LeftStick = -gamepad2.left_stick_x;


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




//        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
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
//        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                (int) (sensorColor.green() * SCALE_FACTOR),
//                (int) (sensorColor.blue() * SCALE_FACTOR),
//                hsvValues);
//        double hue = hsvValues[0];
        FrontSLidePower    = Range.clip(LeftStick, -1.0, 1.0) ;


        switch (liftState)
        {
            case RetractAll:
                if (SlideBackL.getCurrentPosition() - Back_SLide_Up_Specimen < 30) {//set to what color it sees when sample is out of geco wheels
//                    ClawElbow.setPosition(Claw_Arm_Reset);
                    Claw.setPosition(Claw_Close);
                    SlideBackL.setTargetPosition(Back_slide_Reset);
                    SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideBackR.setTargetPosition(Back_slide_Reset);
                    SlideBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    ClawArmL.setPosition(-Claw_Arm_Reset);
                    ClawArmR.setPosition(Claw_Arm_Reset);

                    GecoWristL.setPosition(-Geco_Wrist_Reset);
                    GecoWristR.setPosition(Geco_Wrist_Reset);

                    Claw.setPosition(Claw_Open);
                    liftState = LiftState.SubmersibleReady;

                    //resets all movment items to ensure nothing wrong has happend
                }
                break;
            case SubmersibleReady:
                // otherwise do nothing.
                if (Math.abs(SlideBackL.getCurrentPosition() - Back_slide_Reset) < 30 && gamepad2.x) {
                    // x is pressed, start extending
                    double dd = gamepad2.right_stick_x;

                    double cp = Range.clip(dd, -0.5, 0.5);

                    SlideFrontL.setPower(-cp);
                    SlideFrontR.setPower(cp);
                    //allowing oporater to control the distance of the front slide

                    liftState = LiftState.SamplePicked;//changing state so we can move to the next task
                }
                break;
            case SamplePicked:
                if (gamepad2.b) {//its set b so that it only goes down when the driver and oporater have aligned with the sampele
                    // set the lift dump to dump
                    GecoWristL.setPosition(-Geco_Wrist_Down);
                    GecoWristR.setPosition(Geco_Wrist_Down);

                    GecoWheelsL.setPower(-Geco_IN);
                    GecoWheelsR.setPower(Geco_IN);

                    liftTimer.reset();
                    liftState = LiftState.ClawReadyForSample;
                    //this grabes the sample that we were attempting to grab
                }
                break;


            case ClawReadyForSample:
//                hue >= 0 && hue < 60 || hue > 360 || hue >= 60 && hue < 120
                if (gamepad2.dpad_left) {//If we missed or got wrong color we dont click it so we dont get a penelty
                    GecoWristL.setPosition(-Geco_Wrist_Claw_Deposit);
                    GecoWristR.setPosition(Geco_Wrist_Claw_Deposit);

                    SlideFrontL.setTargetPosition(-Front_SLide_in);
                    SlideFrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    SlideFrontR.setTargetPosition(Front_SLide_in);
                    SlideFrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Claw.setPosition(Claw_Open);
                    liftState = LiftState.SampleDrop;

                    //We have captured the sample and now we are ready to do the transfer from geco wheels to claw

                } else {//if we missed the sample or got the oponets color we dont click anything
                    GecoWristL.setPosition(-Geco_Wrist_Reset);
                    GecoWristR.setPosition(Geco_Wrist_Reset);
                    GecoWheelsL.setPower(-Geco_OUT);
                    GecoWheelsR.setPower(Geco_OUT);
                    liftState = LiftState.SubmersibleReady;

                    //resets all movment things to get ready to attempt to grab a diffrent sample
                }
                break;


            case SampleDrop:
                if (Math.abs(SlideFrontR.getCurrentPosition() - Front_SLide_in) < 30 && gamepad2.a) {
                    Claw.setPosition(Claw_Close);
                    GecoWheelsL.setPower(-Geco_OUT);
                    GecoWheelsR.setPower(Geco_OUT);
                    ClawArmL.setPosition(-Claw_Arm_Specimen_Pick);
                    ClawArmR.setPosition(Claw_Arm_Specimen_Pick);
                    //does the transfer and the claw is in position to drop in obsevtary zone or do bucket

                    if (gamepad2.left_bumper) {//click left bumber if we are doing specimen
                        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));//moving change
                        Claw.setPosition(Claw_Open);
                        liftState = LiftState.SpecimenHanged;
                        //drops sample in obsevatrey zone
                    } if (gamepad2.right_bumper) {//click right bumber if we are doing basket
                        SlideBackL.setTargetPosition(-Back_slide_Bucket);
                        SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        SlideBackR.setTargetPosition(Back_slide_Bucket);
                        SlideBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        ClawArmL.setPosition(-Claw_Arm_Specimen_Pick);
                        ClawArmR.setPosition(Claw_Arm_Specimen_Pick);
                        liftState = LiftState.FinishedBasket;
                        //ready to drop off the sample at bucket slide is rasied to correct hight
                    }


                }
                break;
            case FinishedBasket://add if the back slide is at basket hight
                Claw.setPosition(Claw_Open);
                SlideBackL.setTargetPosition(-Back_slide_Reset);
                SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                SlideBackL.setTargetPosition(Back_slide_Reset);
                SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftState = LiftState.RetractAll;
                //drops sample in basket and retracts evreything
                break;

            case SpecimenPicked:
                if ( gamepad2.dpad_up) {//waiting for humen can change to an if statment
                    Claw.setPosition(Claw_Close);
                    SlideBackL.setTargetPosition(Back_SLide_Up_Specimen);
                    SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideBackR.setTargetPosition(Back_SLide_Up_Specimen);
                    SlideBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //add moving code to pick up specimen
                    liftState = LiftState.SpecimenHanged;
                    //picks up specimen from obsevatry zone
                }
                break;
            case SpecimenHanged:
                if(Math.abs(SlideBackR.getCurrentPosition() - Back_SLide_Up_Specimen) < 30 && gamepad2.dpad_down){//waiting to ensure no ones in the way
                    Actions.runBlocking(new SequentialAction(trajectoryActionChosen2));//moving to hang specimen
                    SlideBackL.setTargetPosition(Back_slide_Reset);
                    SlideBackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideBackR.setTargetPosition(Back_slide_Reset);
                    SlideBackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Claw.setPosition(Claw_Open);
                    liftState = LiftState.RetractAll;
                    //drops specimen
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.RetractAll;
        }




    }


}
