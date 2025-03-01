package org.firstinspires.ftc.teamcode.Krunaal;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Tele-op test krunaal", group="Linear OpMode")

public class tt extends LinearOpMode {

    // Declare OpMode members.
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
        DcMotor FrontSlide = hardwareMap.get(DcMotor.class,"Front Slide");// linkage
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

        BackLeftSlide.setDirection(Servo.Direction.REVERSE);
        BackRightSlide.setDirection(Servo.Direction.FORWARD);



        //  RightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

            FrontSlide.setPower(cp);

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
                FrontSlide.setPower(-1);
                FrontSlide.setTargetPosition(1000);
                rightslide.setPosition(1);
                leftslide.setPosition(-1);
               // sleep(1000);
                //Front_Claw.setPosition(-1);
            }

            if(gamepad1.left_bumper){
                FrontSlide.setPower(1);
                FrontSlide.setTargetPosition(10);
                rightslide.setPosition(-0.8);//0.5
                leftslide.setPosition(0.8);
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
