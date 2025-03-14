package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="Tele RR FSM")
public class RR_Tele_Test extends OpMode {

    private MecanumDrive mecanumDrive;

    // An Enum is used to represent lift states.

    public enum LiftState {
        RegTele, //regular hardware operations
        SubmersibleReady,//ready to pick up a block with the front slide
        SamplePicked, //pickup completed and front slide gets ready to transfer
        TransferBasket,  //finish basket except first one
        PickSpecimen,  //Get ready to Pick specimen from human player
        SpecimenHangReady, //Pick specimen from human player and get ready to hang
        SpecimenHanged, //Hang specimen
        Stop,
        Begin
    };

    private LiftState CurrentState;
    private boolean EmergencyStop;

    //Initialize Non drive motors and servos
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor deliveryArmLeft = null;
    DcMotor deliveryArmRight = null;
    DcMotor FrontSlide = null;

    Servo Claw = null;
   // Servo Wrist = null;
    Servo TwistLeft = null;
    Servo TwistRight = null;
    CRServo FrontWrist = null;
    Servo FrontClaw = null;
    Servo ElbowLeft = null;
    Servo ElbowRight = null;

    // Sensor Hardware Map
    RevColorSensorV3 sensorColor = null;
    HuskyLens camera = null;

    ElapsedTime liftTimer = new ElapsedTime();

    //Define all Delivery Arm Encoder positions and power
    // Left is positive ticks and right is negative ticks
    // Power is reversed
    public int Delivery_Arm_Resting_Enc = 10;
    public int Delivery_Arm_HangReady_Enc = 1500;
    public int Delivery_Arm_BasketReady_Enc = 2500;
    public int Delivery_Arm_LowBasketR_Enc = 800;
    public int Delivery_Arm_HangDone_Enc = 1000;
    public int Delivery_Arm_IntakeDone_Enc = 300;
    public int Delivery_Arm_Transfer_Enc = 750;
    public int Delivery_Arm_Start_Enc = 700;
    public double Delivery_Arm_Extend_Power = -0.9;
    public double Delivery_Arm_Retract_Power = 0.9;

    //Define all Front Slide Arm Encoder positions and power
    public int Front_Slide_Resting_Enc = -100;
    public int Front_Slide_Intake_Enc = 530;
    public int Front_Slide_Transfer_Enc = -50;
    public double Front_Slide_Extend_Power = 0.8;
    public double Front_Slide_Retract_Power = -0.5;

    //Define all Twist positions
    public static double TwistL_Intake_Pos = 0.75;
    public static double TwistR_Intake_Pos = 0.75;
    public static double TwistL_Transfer_Pos = 0.05;
    public static double TwistR_Transfer_Pos = 0.05;
    public static double TwistL_IntakeMiddle_Pos = 0.45;
    public static double TwistR_IntakeMiddle_Pos = 0.45;
    public static double TwistL_Rest_Pos = 0.0;
    public static double TwistR_Rest_Pos = 0.0;

    //Define all Front Claw Positions
    public static double FrontClaw_Open_Pos = 0.05;
    public static double FrontClaw_Close_Pos = 0.5;

    //Define all Claw positions
    public static double Claw_Open_Pos = 0.25;
    public static double Claw_Close_Pos = 0.8;

    //Define all Elbow positions
    public static double ElbowL_Intake_Pos = 0.015;
    public static double ElbowR_Intake_Pos = 0.015;
    public static double ElbowL_Transfer_Pos = 0.6;
    public static double ElbowR_Transfer_Pos = 0.6;
    public static double ElbowL_Hang_Pos = 0.2;
    public static double ElbowR_Hang_Pos = 0.2;
    public static double ElbowL_HangDone_Pos = 0.05;
    public static double ElbowR_HangDone_Pos = 0.05;
    public static double ElbowL_Basket_Pos = 0.18;
    public static double ElbowR_Basket_Pos = 0.18;

    //Define all Wrist positions and mode
//    public static double Wrist_Intake_Pos = 0.25;
//    public static double Wrist_Hang_Pos = 0.9;
//    public static double Wrist_Rest_Pos = 0.25;

    //Define wait variable
    public static double action_wait = 0.1;
    public static double pause = 1;

//    public double lfPower;
//    public double rfPower;
//    public double rbPower;
//    public double lbPower;

    @Override
    public void init() {

        Pose2d InitialPose = new Pose2d(0, 0, 0);     // Beginning pose

        //Importing the hardware maps for all drive motors and setting the robot position
        mecanumDrive = new MecanumDrive(hardwareMap, InitialPose);

        liftTimer.reset();

        //Define Hardware Map for all components
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.FORWARD);

        deliveryArmLeft = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        deliveryArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmLeft.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm left motor set reversed
        deliveryArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        deliveryArmRight = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        deliveryArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmRight.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm right motor set reversed
        deliveryArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        ElbowLeft = hardwareMap.get(Servo.class, "Elbow Left");
        ElbowLeft.setDirection(Servo.Direction.REVERSE);

        ElbowRight = hardwareMap.get(Servo.class, "Elbow Right");
        ElbowRight.setDirection(Servo.Direction.FORWARD);

        FrontSlide = hardwareMap.get(DcMotor.class, "Front Slide");
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Front Slide zero power behavior
        FrontSlide.setDirection(DcMotor.Direction.REVERSE);          //Front Slide motor set forward
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide motor reset
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide run using encoders

        FrontWrist = hardwareMap.get(CRServo.class, "Front Wrist");

        FrontClaw = hardwareMap.get(Servo.class, "Front Claw");
        FrontClaw.setDirection(Servo.Direction.FORWARD);

        TwistLeft = hardwareMap.get(Servo.class, "Twist Left");
        TwistLeft.setDirection(Servo.Direction.REVERSE);

        TwistRight = hardwareMap.get(Servo.class, "Twist Right");
        TwistRight.setDirection(Servo.Direction.FORWARD);

//        Wrist = hardwareMap.get(Servo.class, "Wrist");
//        Wrist.setDirection(Servo.Direction.FORWARD);

        camera = hardwareMap.get(HuskyLens.class, "AI Husky Lens");

        sensorColor = hardwareMap.get(RevColorSensorV3.class, "Color");

        CurrentState = LiftState.RegTele;
        EmergencyStop = false;
    }

    public void loop() {

        move();
        Pose2d currentpose = mecanumDrive.localizer.getPose();


        //Print Telemetry for all components and current state
        telemetry.addData("State", CurrentState);
        telemetry.addData("Current Pose", currentpose.position);
        telemetry.addData("Current Pose", currentpose.heading);
        telemetry.addData("Delivery ArmL Position: ", deliveryArmLeft.getCurrentPosition());
        telemetry.addData("Delivery ArmR Position: ", deliveryArmRight.getCurrentPosition());
        telemetry.addData("Elbowleft Position: ", ElbowLeft.getPosition());
        telemetry.addData("ElbowRight Position: ", ElbowRight.getPosition());
        telemetry.addData("Twist Left Position: ", TwistLeft.getPosition());
        telemetry.addData("Twist Right Position: ", TwistRight.getPosition());
        telemetry.addData("Frontslide Position: ", FrontSlide.getCurrentPosition());
//        telemetry.addData("Wrist Position: ", Wrist.getPosition());
        telemetry.addData("Claw Position: ", Claw.getPosition());
        telemetry.addData("Front Claw Position:", FrontClaw.getPosition());
        telemetry.update();

        switch (CurrentState) {
            case RegTele:
                move();
                //Regular Operations
                TeleOperations();
                //Other States
                if (gamepad1.right_bumper) {
                    CurrentState = LiftState.SubmersibleReady;
                } else if (gamepad1.left_bumper) {
                    CurrentState = LiftState.SamplePicked;
                } else if (gamepad2.dpad_left) {
                    CurrentState = LiftState.TransferBasket;
                } else if (gamepad2.b) {
                    CurrentState = LiftState.PickSpecimen;
                } else if (gamepad2.x) {
                    CurrentState = LiftState.SpecimenHangReady;
                } else if (gamepad2.dpad_right) {
                    CurrentState = LiftState.SpecimenHanged;
                } else if (gamepad1.back) {
                    CurrentState = LiftState.Stop;
                } else if (gamepad2.start) {
                    CurrentState = LiftState.Begin;
                }
                telemetry.update();
                break;

            case SubmersibleReady:
                move();
                Actions.runBlocking(new ParallelAction(
                        new MotorAction2(FrontSlide, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                        new DoubleServoAction(TwistLeft, TwistRight, TwistL_Intake_Pos, TwistR_Intake_Pos),
                        // new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Transfer_Pos, ElbowR_Transfer_Pos),
                        new ServoAction(FrontClaw, FrontClaw_Open_Pos)
                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case SamplePicked:
                move();
                Actions.runBlocking(new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        new DoubleServoAction(TwistLeft, TwistRight, TwistL_Transfer_Pos, TwistR_Transfer_Pos),
                                        new MotorAction2(FrontSlide, Front_Slide_Transfer_Enc, Front_Slide_Retract_Power)
                                )
                               // new ServoAction(FrontClaw, FrontClaw_Open_Pos)
                        ),
                        //  new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Transfer_Pos, ElbowR_Transfer_Pos),
                         new ServoAction(Claw,Claw_Open_Pos)

                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case TransferBasket:
                 move();
                Actions.runBlocking(new SequentialAction(
                        new ServoAction(Claw, Claw_Close_Pos),
                        new DoubleMotorAction(deliveryArmLeft, deliveryArmRight, Delivery_Arm_BasketReady_Enc, -Delivery_Arm_BasketReady_Enc, Delivery_Arm_Extend_Power, Delivery_Arm_Extend_Power),
                        new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Basket_Pos, ElbowR_Basket_Pos),
                        new SleepAction(pause),
                        new ServoAction(Claw, Claw_Open_Pos),
                        new ParallelAction(
                                new MotorAction2(deliveryArmLeft, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power),
                                new MotorAction2(deliveryArmRight, -Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power),
                                new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Transfer_Pos, ElbowR_Transfer_Pos)
//                                new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power)

                        )
                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case PickSpecimen:
                move();
                Actions.runBlocking(new ParallelAction(
                        new ParallelAction(
                                new MotorAction2(deliveryArmLeft, Delivery_Arm_Resting_Enc, Delivery_Arm_Retract_Power),
                                new MotorAction2(deliveryArmRight, -Delivery_Arm_Resting_Enc, Delivery_Arm_Retract_Power)
                        ),
                        new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Intake_Pos, ElbowR_Intake_Pos),
//                        new ServoAction(Wrist, Wrist_Intake_Pos),
                        new ServoAction(Claw, Claw_Open_Pos)
//                        new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power)
                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case SpecimenHangReady:
                 move();
                Actions.runBlocking(new SequentialAction(
                                new ServoAction(Claw, Claw_Close_Pos),
                                new ParallelAction(
                                        new ParallelAction(
                                                new MotorAction2(deliveryArmLeft, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                                                new MotorAction2(deliveryArmRight, -Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power)
                                        ),
                                        new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Hang_Pos, ElbowR_Hang_Pos)
//                                        new ServoAction(Wrist, Wrist_Hang_Pos),
//                                        new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power)
                                )
                        )
                );
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case SpecimenHanged:
                 move();
                Actions.runBlocking(new SequentialAction(
                                new ParallelAction(
                                        new ParallelAction(
                                                new MotorAction2(deliveryArmLeft, Delivery_Arm_HangDone_Enc, Delivery_Arm_Retract_Power),
                                                new MotorAction2(deliveryArmRight, -Delivery_Arm_HangDone_Enc, Delivery_Arm_Retract_Power)
                                        ),
                                        new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_HangDone_Pos, ElbowR_HangDone_Pos)
//                                        new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power)
                                )
//                                new ServoAction(Claw, Claw_Open_Pos)
                        )
                );
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case Begin:
                Actions.runBlocking(
                        new SequentialAction(
                                new DoubleMotorAction(deliveryArmLeft, deliveryArmRight, Delivery_Arm_Transfer_Enc, -Delivery_Arm_Transfer_Enc, Delivery_Arm_Extend_Power, Delivery_Arm_Extend_Power),
                                new DoubleServoAction(ElbowLeft, ElbowRight, ElbowL_Transfer_Pos, ElbowR_Transfer_Pos),
                                new ServoAction(Claw, Claw_Open_Pos)
                        )
                );
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case Stop:
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                EmergencyStop = true;

                if (gamepad2.back) {
                    EmergencyStop = false;
                    CurrentState = LiftState.RegTele;
                }
                break;

        }
    }


    public void move() {
        //Driving
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        PoseVelocity2d drivePower = new PoseVelocity2d(new Vector2d(x, y), rx);

        // Use MecanumDrive class to drive the robot
        mecanumDrive.setDrivePowers(drivePower);
        mecanumDrive.updatePoseEstimate(); // Update the drive to apply motor powers
    }

    public void TeleOperations() {
        //Back Claw
        if (gamepad1.x) {
            Claw.setPosition(Claw_Close_Pos);
        }
        if (gamepad1.b) {
            Claw.setPosition(Claw_Open_Pos);
        }
        //FrontSlide
        double FrontSlideStick = gamepad2.right_stick_x;
        double FrontSlidePower = Range.clip(FrontSlideStick, Front_Slide_Retract_Power, Front_Slide_Extend_Power);
        FrontSlide.setPower(FrontSlidePower);
        //Back Elbow
        if (gamepad1.a) {
            ElbowLeft.setPosition(ElbowL_Intake_Pos);
            ElbowRight.setPosition(ElbowR_Intake_Pos);
        }
        if (gamepad2.dpad_up) {
            ElbowLeft.setPosition(ElbowL_Basket_Pos);
            ElbowRight.setPosition(ElbowR_Basket_Pos);
        }
        if (gamepad2.dpad_down) {
            ElbowLeft.setPosition(ElbowL_Transfer_Pos);
            ElbowRight.setPosition(ElbowR_Transfer_Pos);
        }
        //Front Twist
        if (gamepad2.left_bumper) {
            TwistRight.setPosition(TwistR_Intake_Pos);
            TwistLeft.setPosition(TwistL_Intake_Pos);
        }
        if (gamepad2.right_bumper) {
            TwistRight.setPosition(TwistR_Transfer_Pos);
            TwistLeft.setPosition(TwistL_Transfer_Pos);
        }
        // Front Claw
        if (gamepad2.y) {
            FrontClaw.setPosition(FrontClaw_Open_Pos);
        }
        if (gamepad2.a) {
            FrontClaw.setPosition(FrontClaw_Close_Pos);
        }
        // Front Wrist
        if (gamepad2.right_stick_button) {
            FrontWrist.setPower(1);
        } else if (gamepad2.left_stick_button) {
            FrontWrist.setPower(-1);
        } else
            FrontWrist.setPower(0);
        //Delivery Arm
        double DeliveryArmStick = gamepad2.left_stick_y;
        double DeliveryArmPower = Range.clip(DeliveryArmStick, Delivery_Arm_Extend_Power, Delivery_Arm_Retract_Power);
        deliveryArmLeft.setPower(DeliveryArmPower);
        deliveryArmRight.setPower(-DeliveryArmPower);
    }

    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer = null;

        public ServoAction(Servo srv, double pos) {
            this.servo = srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < action_wait;
        }
    }

    // Double Motor Action build for roadrunner to use
    public static class DoubleMotorAction implements Action {
        DcMotor motor1;
        DcMotor motor2;
        double position_tgt1;
        double position_tgt2;
        double power1;
        double power2;
        ElapsedTime timer = null;

        public DoubleMotorAction(DcMotor mot1, DcMotor mot2, double pos1, double pos2, double pow1, double pow2) {
            this.motor1 = mot1;
            this.position_tgt1 = pos1;
            this.power1 = pow1;
            this.motor2 = mot2;
            this.position_tgt2 = -pos2;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (motor1.getCurrentPosition() < position_tgt1) {
                timer = new ElapsedTime();
                motor1.setPower(power1);
                motor2.setPower(power2);
                motor1.setTargetPosition((int) (position_tgt1));
                // motor2.setTargetPosition((int) (position_tgt2));
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return timer.seconds() < action_wait;
        }
    }

    // Double Servo action build for roadrunner to use
    public static class DoubleServoAction implements Action {
        Servo servo1;
        double position1;
        Servo servo2;
        double position2;
        ElapsedTime timer = null;

        public DoubleServoAction(Servo srv1, Servo srv2, double pos1, double pos2) {
            this.servo1 = srv1;
            this.servo2 = srv2;
            this.position1 = pos1;
            this.position2 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo1.setPosition(position1);
                servo2.setPosition(position2);
            }

            return timer.seconds() < action_wait;
        }
    }

    // Motor Action-2 build for roadrunner to use
    public static class MotorAction2 implements Action {
        DcMotor motor;
        double position_tgt;
        double power;

        public MotorAction2(DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (motor.getCurrentPosition() != position_tgt) {
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

    public static class AlignSensorActionB implements Action {
        public RevColorSensorV3 colorSensor;
        public CRServo turningServo;
        public double position;
        public Servo servo;
        public int alliance_color;

        // Thresholds
        public static int LINE_THRESHOLD = 100;  //color intensity
        public static int YELLOW_THRESHOLD = 50;  //color intensity
        public static int BLUE_THRESHOLD = 90;
        public static double MIN_DISTANCE_THRESHOLD = 5.0; // in cm

        public AlignSensorActionB(RevColorSensorV3 colorSensor, CRServo turningServo, Servo srv, double pos, int all_col) {
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

            if (isWithinDist) {
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
};