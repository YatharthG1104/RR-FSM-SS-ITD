package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name= "Encoder tick test")
public class EncoderTest extends LinearOpMode {
    public static int testPos = 0; // edit in dash
    public static double kP_O = 0.01;
    @Override
    public void runOpMode() {

        DcMotor motor1 = hardwareMap.get(DcMotor.class,"x");
        DcMotor motor2 = hardwareMap.get(DcMotor.class,"y");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())  {
            motor1.setPower(outtakePid(testPos,motor1.getCurrentPosition()));
            motor2.setPower(outtakePid(testPos,motor2.getCurrentPosition()));

            telemetry.addData("Motor 1 pos", motor1.getCurrentPosition());
            telemetry.addData("Motor 2 pos", motor2.getCurrentPosition());
            telemetry.addData("testPos", testPos);
            telemetry.update();
        }

    }

    public double outtakePid(int target, int current){
        return (target - current)*kP_O;
    }


}
