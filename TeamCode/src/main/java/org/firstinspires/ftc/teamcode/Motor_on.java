package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "motor run",group = "drives")
public class Motor_on extends LinearOpMode {
    DcMotorEx motor1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class,"motor1");

        waitForStart();

        while(opModeIsActive())
        {
            motor1.setPower(0.5);
            motor1.setVelocity(400);

            telemetry.addData("motor status",motor1.getCurrentPosition());
            telemetry.addData("motor enabled",motor1.isMotorEnabled());
            telemetry.update();
        }
    }
}
