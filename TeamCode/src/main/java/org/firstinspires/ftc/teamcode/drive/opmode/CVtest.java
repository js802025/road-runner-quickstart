package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.mapping.CVMapping;


@TeleOp(group = "drive")
public class CVtest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        CVMapping sensorMapping = new CVMapping(this, drive, hardwareMap, telemetry);
        Lift lift = new Lift(telemetry, hardwareMap);

        waitForStart();
        sensorMapping.turnToColor();

        while (!isStopRequested()) {
            sensorMapping.update();
            if (!sensorMapping.isBusy()){
                lift.setHeight(200);

            }
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Sensor Mapping", sensorMapping);
            telemetry.update();
        }
    }
}
