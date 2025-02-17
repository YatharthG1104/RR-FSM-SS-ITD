package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "RRAction Auto_OZ_ZONE")
public class RRAction_Auto_OZ extends LinearOpMode {

    // Time into the Autonomous round
    public ElapsedTime mRuntime = new ElapsedTime();

    //Initialize Non drive motors and servos
    DcMotor deliveryArmLeft = null;
    DcMotor deliveryArmRight = null;
    DcMotor FrontSlide = null;

    Servo Claw = null;
    Servo Wrist = null;
    Servo TwistLeft = null;
    Servo TwistRight = null;
    CRServo GrabLeft = null;
    CRServo GrabRight = null;
    Servo ElbowLeft = null;
    Servo ElbowRight = null;

    // Color Sensor Hardware Map
    ColorSensor sensorColor = null;

    //Define all Wrist positions and mode
    public static double Wrist_Intake_Pos = 0.3;
    public static double Wrist_Hang_Pos = 0.8;
    public static double Wrist_Rest_Pos = 0.3;


    //Define all Claw positions
    public static double Claw_Open_Pos = 0.7;
    public static double Claw_Close_Pos = 0.3;
    public static double Claw_Initial_Pos = 0.0;

    //Define all Delivery Arm Encoder positions and power
    public static int Delivery_Arm_Resting_Enc = -100;
    public static int Delivery_Arm_HangReady_Enc = -2600;
    public static int Delivery_Arm_HangDone_Enc = -2300;
    public static int Delivery_Arm_IntakeDone_Enc = -800;
    public static int Delivery_Arm_HangIntake_Enc = -540;
    public static double Delivery_Arm_Extend_Power = -0.9;
    public static double Delivery_Arm_Retract_Power = 0.9;

    //Define all Elbow positions
    public static double ElbowL_Intake_Pos = 0.8;
    public static double ElbowR_Intake_Pos = 0.8;
    public static double ElbowL_Hang_Pos = 0.5;
    public static double ElbowR_Hang_Pos = 0.5;
    public static double ElbowL_Rest_Pos = 0.0;
    public static double ElbowR_Rest_Pos = 0.0;

    //Define all Front Slide Arm Encoder positions and power
    public static int Front_Slide_Resting_Enc = 10;
    public static int Front_Slide_Intake_Enc = 400;
    public static int Front_Slide_Transfer_Enc = 100;
    public static double Front_Slide_Extend_Power = 0.7;
    public static double Front_Slide_Retract_Power = -0.7;


    @Override
    public void runOpMode() throws InterruptedException {

        //Define all Poses here
        Pose2d InitialPose = new Pose2d(0,0,0);     //Beginning pose
        Pose2d Pose1 = new Pose2d(28,0,0);          //Ready for 1st hang
        Pose2d Pose2 = new Pose2d(15,0,0);          // After 1st Hang
        Pose2d Pose3  = new Pose2d(3, -30,0 );      // Ready for Intake
        Pose2d Pose4  = new Pose2d(28, 0,0);        // Ready for next hang

        //Importing the hardware maps for all drive motors and setting the robot position
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        //Define Hardware Map for all components
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(0,1);
        Claw.setDirection(Servo.Direction.FORWARD);

        deliveryArmLeft = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        deliveryArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmLeft.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm left motor set reversed
        deliveryArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        deliveryArmRight = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        deliveryArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmRight.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm right motor set reversed
        deliveryArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        ElbowLeft = hardwareMap.get(Servo.class, "Elbow Left");
        ElbowLeft.scaleRange(-1,1);
        ElbowLeft.setDirection(Servo.Direction.REVERSE);


        ElbowRight = hardwareMap.get(Servo.class, "Elbow Right");
        ElbowRight.scaleRange(-1,1);
        ElbowRight.setDirection(Servo.Direction.FORWARD);


        FrontSlide = hardwareMap.get(DcMotor.class, "Front Slide");
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Front Slide zero power behavior
        FrontSlide.setDirection(DcMotor.Direction.FORWARD);          //Front Slide motor set forward
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide motor reset
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide run using encoders

        GrabLeft = hardwareMap.get(CRServo.class, "Grab Left");

        GrabRight = hardwareMap.get(CRServo.class, "Grab Right");

        TwistLeft = hardwareMap.get(Servo.class, "Twist Left");
        TwistLeft.scaleRange(-1,1);
        TwistLeft.setDirection(Servo.Direction.REVERSE);

        TwistRight = hardwareMap.get(Servo.class, "Twist Right");
        TwistRight.scaleRange(-1,1);
        TwistRight.setDirection(Servo.Direction.FORWARD);

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Wrist.scaleRange(-1,1);
        Wrist.setDirection(Servo.Direction.FORWARD);

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");


        //set  start game clock
        mRuntime.reset();                               // Zero game clock

        //Build all Trajectories
        TrajectoryActionBuilder Path1 = drive.actionBuilder(InitialPose)
                .lineToX(28)
                .waitSeconds(0.5);

        TrajectoryActionBuilder Path2 = drive.actionBuilder(Pose1)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .lineToX(15);

        TrajectoryActionBuilder Path3 = drive.actionBuilder(Pose2)
                .strafeTo(new Vector2d(15, -25))
                .strafeTo(new Vector2d(50, -40))
                .setReversed(true)
                .setTangent(0)
                .lineToX(5)
                .strafeTo(new Vector2d(50,-50))
                .setReversed(true)
                .setTangent(0)
                .lineToX(5)
                .strafeToLinearHeading(new Vector2d(3,-30), 0);

        TrajectoryActionBuilder Path4 = drive.actionBuilder(Pose3)
                .strafeToLinearHeading(new Vector2d(28, 0),0);

        TrajectoryActionBuilder Path5 = drive.actionBuilder(Pose4)
                .strafeToLinearHeading(new Vector2d(3, -30),0);

        TrajectoryActionBuilder Path6 = drive.actionBuilder(Pose4)
                .strafeTo(new Vector2d(3, -55));

        PoseVelocity2d velEstimate = drive.updatePoseEstimate();    //Get current velocity

        Action trajectorychosen;       // Define Action to choose the trajectory in the Action Builder

        waitForStart();

        //extra- PLEASE DELETE LATER
        trajectorychosen = Path1.build();
         Actions.runBlocking(
                new SequentialAction
                        (
                      //  trajectorychosen,
                       // new MotorAction(deliveryArmLeft, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                        //new MotorAction(deliveryArmRight, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                      //  new MotorAction(FrontSlideLeft, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                        //new MotorAction(FrontSlideRight, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                       // new ServoAction(ElbowLeft, ElbowL_Hang_Pos),
                       // new ServoAction(ElbowRight, ElbowR_Hang_Pos),
                        new ServoAction(Wrist, Wrist_Hang_Pos),
                        new DoubleMotorAction(deliveryArmLeft,deliveryArmRight, Delivery_Arm_HangIntake_Enc, -Delivery_Arm_HangIntake_Enc, Delivery_Arm_Extend_Power, Delivery_Arm_Extend_Power))
        );
        Pose2d poseEstimate = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

/*  // Actual code commented
        trajectorychosen = Path1.build();
        Actions.runBlocking(
                new ParallelAction(
                        trajectorychosen,
                        new MotorAction(deliveryArmLeft, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                        new MotorAction(deliveryArmRight, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                        new MotorAction(FrontSlideLeft, Front_Slide_Resting_Enc, Front_Slide_Retract_Power),
                        new MotorAction(FrontSlideRight, Front_Slide_Resting_Enc, Front_Slide_Retract_Power),
                        new ServoAction(ElbowLeft, ElbowL_Hang_Pos),
                        new ServoAction(ElbowRight, ElbowR_Hang_Pos),
                        new ServoAction(Wrist, Wrist_Hang_Pos))
                );
        Pose2d poseEstimate = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

        trajectorychosen = Path2.build();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new MotorAction(deliveryArmRight, Delivery_Arm_HangDone_Enc, Delivery_Arm_Extend_Power),
                                new MotorAction(deliveryArmLeft, Delivery_Arm_HangDone_Enc, Delivery_Arm_Extend_Power)
                        ),
                        new ServoAction(Claw, Claw_Open_Pos),
                        new ParallelAction(
                                trajectorychosen,
                                new ServoAction(ElbowRight, ElbowR_Intake_Pos),
                                new ServoAction(ElbowLeft, ElbowL_Intake_Pos),
                                new ServoAction(Wrist,Wrist_Intake_Pos),
                                new MotorAction(deliveryArmRight,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power),
                                new MotorAction(deliveryArmLeft,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power)
                        )
                )
        );
        Pose2d poseEstimate1 = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate1.heading);
        telemetry.addData("X,Y", poseEstimate1.position);
        telemetry.update();

        trajectorychosen = Path3.build();
        Actions.runBlocking(
                trajectorychosen
        );
        Pose2d poseEstimate2 = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate2.heading);
        telemetry.addData("X,Y", poseEstimate2.position);
        telemetry.update();

        trajectorychosen = Path4.build();
        Actions.runBlocking(
                new SequentialAction(
                        new ServoAction(Claw, Claw_Close_Pos),
                        new ParallelAction(
                                new MotorAction(deliveryArmLeft, Delivery_Arm_IntakeDone_Enc, Delivery_Arm_Extend_Power),
                                new MotorAction(deliveryArmRight, Delivery_Arm_IntakeDone_Enc, Delivery_Arm_Extend_Power)
                        ),
                        new ParallelAction(
                                trajectorychosen,
                                new MotorAction(deliveryArmLeft, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                                new MotorAction(deliveryArmRight, Delivery_Arm_HangReady_Enc, Delivery_Arm_Extend_Power),
                                new ServoAction(ElbowRight, ElbowR_Hang_Pos),
                                new ServoAction(ElbowLeft, ElbowL_Hang_Pos),
                                new ServoAction(Wrist, Wrist_Hang_Pos)
                        )
                )
        );
        Pose2d poseEstimate3 = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate3.heading);
        telemetry.addData("X,Y", poseEstimate3.position);
        telemetry.update();

        trajectorychosen = Path5.build();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new MotorAction(deliveryArmRight, Delivery_Arm_HangDone_Enc, Delivery_Arm_Retract_Power),
                                new MotorAction(deliveryArmLeft, Delivery_Arm_HangDone_Enc, Delivery_Arm_Retract_Power)
                        ),
                        new ServoAction(Claw, Claw_Open_Pos),
                        new ParallelAction(
                                trajectorychosen,
                                new MotorAction(deliveryArmRight, Delivery_Arm_HangIntake_Enc, Delivery_Arm_Retract_Power),
                                new MotorAction(deliveryArmLeft, Delivery_Arm_HangIntake_Enc, Delivery_Arm_Retract_Power),
                                new ServoAction(ElbowLeft, ElbowL_Intake_Pos),
                                new ServoAction(ElbowRight, ElbowR_Intake_Pos),
                                new ServoAction(Wrist, Wrist_Intake_Pos)
                        )
                )

        );
        Pose2d poseEstimate4 = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate4.heading);
        telemetry.addData("X,Y", poseEstimate4.position);
        telemetry.update();

        trajectorychosen = Path6.build();
        Actions.runBlocking(
                new ParallelAction(
                        trajectorychosen,
                        new MotorAction(deliveryArmRight, Delivery_Arm_Resting_Enc, Delivery_Arm_Retract_Power),
                        new MotorAction(deliveryArmLeft, Delivery_Arm_Resting_Enc, Delivery_Arm_Retract_Power),
                        new ServoAction(ElbowLeft, ElbowL_Rest_Pos),
                        new ServoAction(ElbowRight, ElbowR_Rest_Pos),
                        new ServoAction(Wrist, Wrist_Rest_Pos),
                        new ServoAction(Claw, Claw_Initial_Pos)
                )
        );
        Pose2d poseEstimate5 = drive.localizer.getPose();            //Get current pose
        telemetry.addData("heading", poseEstimate5.heading);
        telemetry.addData("X,Y", poseEstimate5.position);
        telemetry.update();
        */
    }


    // Servo action build for roadrunner to use
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

            return timer.seconds() < 0.5;
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
            return timer.seconds() < 0.5;
        }
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
            return timer.seconds() < 0.5;
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

        public DoubleMotorAction (DcMotor mot1, DcMotor mot2, double pos1, double pos2, double pow1, double pow2) {
            this.motor1 = mot1;
            this.position_tgt1 = pos1;
            this.power1 = pow1;
            this.motor2 = mot2;
            this.position_tgt2 = pos2;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                motor1.setPower(power1);
                motor2.setPower(power2);
                motor1.setTargetPosition((int) (position_tgt1));
                motor2.setTargetPosition((int) (position_tgt2));
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return timer.seconds() < 0.5;
        }
    }

    // Double Servo action build for roadrunner to use
    public static class DoubleServoAction implements Action {
        Servo servo1;
        double position1;
        Servo servo2;
        double position2;
        ElapsedTime timer= null;

        public DoubleServoAction(Servo srv1, Servo srv2,double pos1,double pos2) {
            this.servo1 = srv1;
            this.servo2 = srv2;
            this.position1 = pos1;
            this.position2 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo1.setPosition(position1);
                servo2.setPosition(position2);
            }

            return timer.seconds() < 0.5;
        }
    }

    //Double CRServo Action build for roadrunner to use
    public static class DoubleCRServoAction implements Action {
        CRServo crServo1;
        CRServo crServo2;
        double power1;
        double power2;
        int seconds1;
        int seconds2;
        ElapsedTime timer= null;

        public DoubleCRServoAction(CRServo crsrv1, CRServo crsrv2,  int sec1, int sec2, double pow1, double pow2) {
            this.crServo1 = crsrv1;
            this.crServo2 = crsrv2;
            this.seconds1 = sec1;
            this.seconds2 = sec2;
            this.power1 = pow1;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < seconds1) {
                timer = new ElapsedTime();
                crServo1.setPower(power1);
                crServo2.setPower(power2);
            }
            return timer.seconds() < 0.5;
        }
    }

    // Double Motor and Servo Action build for roadrunner to use
    public static class DoubleServoAndMotorAction implements Action{

        DcMotor motor;
        Servo servo;
        double Motor_pos_target;
        double motor_power;
        double servPos;
        ElapsedTime timer = null;

        public DoubleServoAndMotorAction (DcMotor mot, Servo serv, double motPos, double motPow, double servoPos) {
            this.motor = mot;
            this.servo = serv;
            this.Motor_pos_target = motPos;
            this.motor_power = motPow;
            this.servPos = servoPos;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(servPos);
                motor.setPower(motor_power);
                motor.setTargetPosition((int)(Motor_pos_target));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return timer.seconds() < 0.5;
        }
    }


}


