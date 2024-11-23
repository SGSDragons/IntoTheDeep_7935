package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ArmAuto")
@Config
public class ArmAuto extends LinearOpMode {

    public static int DRIVE1 = 11;
    public static int DRIVE2 = -9;
    public static int DRIVE3 = 36;
    public static int TURN1 = -90;
    public static int ARM1 = 170;
    public static int ARM2 = -200;

    int RIGHT = -90;
    int LEFT = 90;


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
        sleep(1000);
        driver.drive(DRIVE1,false);
        driver.arm(ARM1);
        driver.drive(DRIVE2,true);
        clamp.setPosition(0.5);
        sleep(500);
        driver.arm(ARM2);
        sleep(500);
        driver.turn(TURN1);
        driver.drive(DRIVE3,false);

        }
    }


