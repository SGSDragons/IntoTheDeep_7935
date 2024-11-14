package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Optional;

@Autonomous(name = "7935Auto")
@Config
public class IntoTheDeepAuto extends LinearOpMode {

    public static int DRIVE1 = 18;
    public static int DRIVE2 = -9;
    public static int DRIVE3 = 30;
    public static int TURN1 = -90;
    public static int ARM1 = 175;
    public static int STRAFETEST = 5;

    int RIGHT = -90;
    int LEFT = 90;
    int FORWARD = 0;
    int BACKWARD = 180;


    public Autodrive driver;
    DcMotor arm;
    DcMotor lift;
    Servo dump;
    Servo clamp;
    IMU imu;


    @Override
    public void runOpMode() {
        runauto();
    }

    public void runauto() {

        waitForStart();

        driver = new Autodrive(hardwareMap, this::opModeIsActive);
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        Servo clamp = hardwareMap.get(Servo.class, "claw");
        Servo dump = hardwareMap.get(Servo.class, "dump");
        IMU imu = hardwareMap.get(IMU.class,"imu");

        int armstart = arm.getCurrentPosition();

        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        clamp.setPosition(0.05);
        driver.drive(DRIVE1,FORWARD,false);
        driver.arm(ARM1);
        driver.drive(DRIVE2,BACKWARD,true);
        clamp.setPosition(0.3);
        sleep(500);
        driver.turn(TURN1);

        }
    }


