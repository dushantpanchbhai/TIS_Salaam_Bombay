package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "motor test 2",group = "motor")
public class MotorTest2 extends LinearOpMode {
    public int MAX_ANGLE_1;
    public int MIN_ANGLE_1;
    public int MAX_ANGLE_2;
    public int MIN_ANGLE_2;

    public DcMotorEx bottomMotor,middleMotor,upperMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        bottomMotor = hardwareMap.get(DcMotorEx.class,"motor1");
        middleMotor = hardwareMap.get(DcMotorEx.class,"motor2");
        upperMotor = hardwareMap.get(DcMotorEx.class,"motor3");

        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.x)
            {
                bottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bottomMotor.setTargetPosition(MAX_ANGLE_1);
            }
            else if(gamepad1.a)
            {
                bottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bottomMotor.setTargetPosition(MIN_ANGLE_1);
            }
            else if(gamepad1.y)
            {
                middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor.setTargetPosition(MAX_ANGLE_2);
            }
            else if(gamepad1.b)
            {
                middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor.setTargetPosition(MIN_ANGLE_2);
            }
            else if(gamepad1.dpad_up)
            {
                upperMotor.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                upperMotor.setPower(0);
            }

            telemetry.addData("motor1",bottomMotor.getCurrentPosition());
            telemetry.addData("motor2",middleMotor.getCurrentPosition());
            telemetry.addData("motor3",upperMotor.getCurrentPosition());
        }
    }
}
