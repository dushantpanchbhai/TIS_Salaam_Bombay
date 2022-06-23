package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "motor test 1",group = "motor")
public class MotorTest1 extends LinearOpMode {
    public DcMotorEx bottomMotor,middleMotor,upperMotor,turret;
    public Servo servo;

    @Override
    public void runOpMode(){
        bottomMotor = hardwareMap.get(DcMotorEx.class,"motor1");
        middleMotor = hardwareMap.get(DcMotorEx.class,"motor2");
        upperMotor = hardwareMap.get(DcMotorEx.class,"motor3");
        servo = hardwareMap.get(Servo.class,"servo");

        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class,"motor4");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x)
            {
                bottomMotor.setTargetPosition(bottomMotor.getCurrentPosition() + 10);
                bottomMotor.setPower(3);
                bottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                bottomMotor.setVelocity(300);
            }
            else if(gamepad1.a)
            {
                bottomMotor.setTargetPosition(bottomMotor.getTargetPosition() - 10);
                bottomMotor.setPower(0.5);
                bottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                bottomMotor.setVelocity(-300);
            }
            else if(gamepad1.y)
            {
                middleMotor.setTargetPosition(middleMotor.getCurrentPosition() + 10);
                middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor.setPower(3);
            }
            else if(gamepad1.b)
            {
                middleMotor.setTargetPosition(middleMotor.getCurrentPosition() - 10);
                middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor.setPower(3);
            }
            else if(gamepad1.left_bumper)
            {
                turret.setTargetPosition(turret.getCurrentPosition() - 10);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.3);
            }
            else if(gamepad1.right_bumper)
            {
                turret.setTargetPosition(turret.getCurrentPosition() + 10);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.3);
            }
            else if(gamepad1.dpad_up)
            {
                upperMotor.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                upperMotor.setPower(-1);
            }
            else if(gamepad1.dpad_left)
            {
                upperMotor.setPower(0);
            }
            else if(gamepad1.left_stick_y != 0)
            {
                servo.setPosition(servo.getPosition() + gamepad1.left_stick_y);
                sleep(50);
                idle();
            }
            else
            {
                telemetry.addData("no button","pressed");
            }

            telemetry.addData("motor1",bottomMotor.getCurrentPosition());
            telemetry.addData("motor2",middleMotor.getCurrentPosition());
            telemetry.addData("motor3",upperMotor.isMotorEnabled());
            telemetry.addData("servo position",servo.getPosition());
            telemetry.update();
        }
    }
}
