package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DROP;
import static org.firstinspires.ftc.teamcode.Constants.GRIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "trajectory_seq_3",group = "Linear Opmode")
public class TrajectorySequence3 extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx arm,extension;

    public static PIDFCoefficients extpidcoffs = new PIDFCoefficients(10,0,0,0);

    private Servo leftClaw,rightClaw;

    // bot path parameters
    double y = 7;

    //distance between point and carousel;
    double frontDist = 28;
    //distance between top and bottom point
    double backDist = 10;//40.25-23;

    DcMotorEx carousel;

    Rev2mDistanceSensor distance1;
    Rev2mDistanceSensor distance2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status","Initialized");
        telemetry.update();

        telemetry = dashboard.getTelemetry();
        telemetry.addData("status","Initialized");
        telemetry.update();

        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);

        arm = hardwareMap.get(DcMotorEx.class,"arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        extension = hardwareMap.get(DcMotorEx.class,"extension");
        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, extpidcoffs);
        extension.setPositionPIDFCoefficients(Constants.EXT_COEFF);

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        // Carousel
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "distance2");

        // Get arm and extension ready
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(Constants.ARM_MOVE);

        extension.setTargetPosition(0);
        extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension.setPower(Constants.EXT_MOVE);


        // Start pose

        Pose2d start = new Pose2d(23,57.5+y,Math.toRadians(-90));
        Pose2d start2 = new Pose2d(23,57.5,Math.toRadians(0));

        tankDrive.setPoseEstimate(start);
        // Trajectories

        TrajectorySequence move2 = tankDrive.trajectorySequenceBuilder(start2)
//                .turn(Math.toRadians(90)).waitSeconds(0.1)
                .forward(backDist).waitSeconds(0.1)

                .addTemporalMarker(()->{
                    moveArm(-160);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    extension.setTargetPosition(25);
                }).waitSeconds(0.5) //0.5
                .addTemporalMarker(()->{
                    leftClaw.setPosition(Constants.GRIP);
                    rightClaw.setPosition(Constants.GRIP);
                }).waitSeconds(0.5) //0.5


                .addTemporalMarker(()->{
                    moveArm(390);
                }).waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    extension.setTargetPosition(50);
                }).waitSeconds(0.2)

                .back(backDist).waitSeconds(0.1)
                .turn(Math.toRadians(-(45+90))).waitSeconds(0.1)
                .forward(frontDist).waitSeconds(0.1)

                .addTemporalMarker(()->{
                    leftClaw.setPosition(DROP);
                    rightClaw.setPosition(DROP);
                })

                .back(frontDist).waitSeconds(0.01)
                .turn(Math.toRadians(90 + 45)).waitSeconds(0.01)

                .build();


        TrajectorySequence move = tankDrive.trajectorySequenceBuilder(start)
                //arm intial position sequence

                .addTemporalMarker(()->{
                    leftClaw.setPosition(0.9);
                    rightClaw.setPosition(0.9);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(40);
                }).waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    leftClaw.setPosition(0.9);
//                    rightClaw.setPosition(0.9);
//                }).waitSeconds(0.2)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(200); //210
                }).waitSeconds(0.2)
                .addTemporalMarker(()->{
                    extension.setTargetPosition(50);
                }).waitSeconds(0.2)

                //detecting distance for arm position
                .addTemporalMarker(()->{
                    if (getAvgDis(distance1) < 10){
                        moveArm(15);
                    }
                    else if (getAvgDis(distance2) < 10){
                        moveArm(170);
                    }
                    else {
                        moveArm(390);
                    }
                }).waitSeconds(0.2)

                //moving forward toward the carousel and turning right
                .forward(y)
                .turn(Math.toRadians(-45 + 5))
                .forward(frontDist)

                //dropping the claw cube
                .addTemporalMarker(()->{
                    moveArm(300);
                }).waitSeconds(0.1)
                .addTemporalMarker(()->{
                    leftClaw.setPosition(Constants.DROP);
                    rightClaw.setPosition(Constants.DROP);
                }).waitSeconds(0.1)

                .back(frontDist)
                .turn(Math.toRadians(90 + 45 - 5))
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested())  return;

        tankDrive.followTrajectorySequence(move);
        PoseStorage.currentPose = tankDrive.getPoseEstimate();

        while(opModeIsActive())
        {
            tankDrive.followTrajectorySequence(move2);
            telemetry.addData("arm position",arm.getCurrentPosition());
            telemetry.addData("extension position",extension.getCurrentPosition());
            telemetry.addData("left claw",leftClaw.getPosition());
            telemetry.addData("right claw",rightClaw.getPosition());
            telemetry.update();
        }

    }
    public double getAvgDis(Rev2mDistanceSensor distanceSensor){
        double sumOfReads = 0;
        double[] disReads = new double[5];
        for (int i = 0; i < 5; i++){
            disReads[i] = distanceSensor.getDistance(DistanceUnit.INCH);
            sumOfReads += disReads[i];
        }
        return sumOfReads / disReads.length;
    }
    public void moveArm(int armTarget){
        arm.setTargetPosition(armTarget);
        arm.setPower(Constants.ARM_MOVE);
    }
}
