package org.firstinspires.ftc.teamcode.Krunaal;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele Automated", group="Linear OpMode")
public class Auto_Tele extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    Servo backClaw; // back claw
    CRServo Front_Rotate; // front left gecko
    Servo Front_Claw; // front right gecko
    Servo wrist; // wrist pivot point
    Servo leftslide    ; // left misumi slide
    Servo rightslide; // right misumi slide

    Servo BackLeftSlide;//left misumi

    Servo BackRightSlide;//rightmisumi
    double cp;
    double p;
    private double servoPosition = 0.0;
    private final double MIN_POSITION = 0.0;
    private final double MAX_POSITION = 1.0;
    private double positionAdjustment = 0.1;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightBack");



        DcMotor DL = hardwareMap.get(DcMotor.class, "Delivery ArmL");//left back misumi motor
        DcMotor DR = hardwareMap.get(DcMotor.class,"Delivery ArmR");//right back misumi motor
        DcMotor FS = hardwareMap.get(DcMotor.class,"Front Slide");// linkage
        // Init servos
        backClaw = hardwareMap.get(Servo.class, "Claw");

        wrist = hardwareMap.get(Servo.class, "Wrist");

        Front_Claw = hardwareMap.get(Servo.class, "Front Claw");
        Front_Rotate = hardwareMap.get(CRServo.class, "Front Wrist");

        leftslide = hardwareMap.get(Servo.class, "Twist Left");//front slide
        rightslide = hardwareMap.get(Servo.class, "Twist Right");

        BackLeftSlide = hardwareMap.get(Servo.class, "Elbow Left");
        BackRightSlide = hardwareMap.get(Servo.class, "Elbow Right");


        DL.setDirection(DcMotor.Direction.REVERSE);
        DL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DR.setDirection(DcMotorSimple.Direction.REVERSE);
        DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //  RightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);
        FS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Defining variable for the motor's power
            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double y = -gamepad1.left_stick_y;
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

            // Setting all the drive motors to their power
            leftFront.setPower(lfPower);
            leftRear.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightRear.setPower(rbPower);


            if(gamepad2.x){
                backClaw.setPosition(-1);
            }
            if(gamepad2.b){
                backClaw.setPosition(1);
            }

            double dd = -gamepad2.right_stick_x;

            cp    = Range.clip(dd, -0.5, 0.5) ;

            FS.setPower(cp);

           /* while(gamepad2.y){
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                lfPower = (y + x + rx) / denominator;
                lbPower = (y - x + rx) / denominator;
                rfPower = (y - x - rx) / denominator;
                rbPower = (y + x - rx) / denominator;

                // Setting all the drive motors to their power
                leftFront.setPower(lfPower);
                leftRear.setPower(lbPower);
                rightFront.setPower(rfPower);
                rightRear.setPower(rbPower);
              //  Front_Rotate.setPosition(0.5);
                // Front_Claw.setPower(-0.5);
            }
            while(gamepad2.a){
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                lfPower = (y + x + rx) / denominator;
                lbPower = (y - x + rx) / denominator;
                rfPower = (y - x - rx) / denominator;
                rbPower = (y + x - rx) / denominator;

                // Setting all the drive motors to their power
                leftFront.setPower(lfPower);
                leftRear.setPower(lbPower);
                rightFront.setPower(rfPower);
                rightRear.setPower(rbPower);
               // Front_Rotate.setPower(-0.5);
               // Front_Claw.setPower(0.5);

            }
           // Front_Rotate.setPower(0);
           // Front_Claw.setPower(0);*/

            if(gamepad2.dpad_down){
                BackLeftSlide.setPosition(-0.53);
                BackRightSlide.setPosition(0.53);
            }
            if(gamepad2.dpad_up){
                BackLeftSlide.setPosition(0.42);//L 436 - 444 R
                BackRightSlide.setPosition(-0.42);
            }
            if(gamepad1.a){
                BackLeftSlide.setPosition(-0.1);
                BackRightSlide.setPosition(0.1);
            }
            if(gamepad2.dpad_left){
                wrist.setPosition(1);
            }
            if(gamepad2.dpad_right){
                wrist.setPosition(-1);
            }
            if(gamepad2.left_bumper){
                rightslide.setPosition(1);
                leftslide.setPosition(-1);
            }
            if(gamepad2.right_bumper){
                rightslide.setPosition(-0.8);//0.5
                leftslide.setPosition(0.8);
            }

            /****Front claw uses Front_Claw***/
            if(gamepad2.y){
                Front_Claw.setPosition(1);
            }
            if(gamepad2.a){
                Front_Claw.setPosition(-1);
            }

            /**Front wrist uses Front_Rotate****/

            if(gamepad2.right_stick_button) {
                Front_Rotate.setPower(1);
            } else if(gamepad2.left_stick_button) {
                Front_Rotate.setPower(-1);
            }
            else
                Front_Rotate.setPower(0);

            if(gamepad1.right_bumper){
                FS.setPower(0.5);
                wrist.setPosition(0.5);
                Front_Claw.setPosition(0.9);
            }

            if(gamepad1.left_bumper){
                FS.setPower(-0.5);
                wrist.setPosition(0.5);
                Front_Claw.setPosition(-0.9);
            }


//            if (servoPosition > MAX_POSITION) {
//                servoPosition = MAX_POSITION;
//            } else if (servoPosition < MIN_POSITION) {
//                servoPosition = MIN_POSITION;
//            }


            double d = gamepad2.left_stick_y;

            p = Range.clip(d, -0.6, 0.6) ;
            DL.setPower(p);//0.5
            DR.setPower(-p);


        }
    }
}
