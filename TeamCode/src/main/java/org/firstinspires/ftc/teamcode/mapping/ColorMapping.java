package org.firstinspires.ftc.teamcode.mapping;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ColorMapping {

    NormalizedColorSensor colorSensor;
    SampleMecanumDrive drive;
    Boolean correcting = false; //whether robot is correcting
    Telemetry telemetry;

    public static double forwardMultiplier = 0.3; //when robot moves forward it goes faster
    public static double backwardMultiplier = 0.05; //moves backwards to correct to line

    public enum COLORS {
        RED (20, 40),
        BLUE (200, 220);

        public int lowerHue;
        public int upperHue;

        private COLORS(int lower, int upper) {
            this.lowerHue = lower;
            this.upperHue = upper;
        }
    }
    COLORS activeColor;

    public ColorMapping(NormalizedColorSensor color, SampleMecanumDrive drive, Telemetry telemetry) {
        this.colorSensor = color;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    public void stopAtLine(COLORS color) {
        drive.setWeightedDrivePower(new Pose2d(0.3, 0, 0)); //go forward

    }

    private void setColor(COLORS color) {
        activeColor = color;
    }

    public void update() {
        if (activeColor != null) {
            final float[] hsvValues = new float[3];
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] >= activeColor.lowerHue && hsvValues[0] <= activeColor.upperHue) { //checks colors
                if (!correcting) { //if it is first time seeing color back up slower
                    drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
                    correcting = true;
                } else {
                    drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    activeColor = null;
                    correcting = false;
                }
            }
        }
    }

    public boolean isBusy() {
        return activeColor != null;
    }


}
