package org.firstinspires.ftc.teamcode.mapping;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.EmptySequenceException;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


/*
In this year's challenge, there were junctions at every place where four tiles met,
creating significant obstruction. To avoid running into those junctions, we
divided the field into 5 streets up/down and 5 streets left/right that run
through the middle of the tiles. This program navigates from a starting position to
many key locations on the field, using these streets if a direct route is not easy.
 */


public class FieldTrajectorySequence {
    @Config
    public static class FieldTrajContstants {
        public static double coneoffset = 8;
        public static int clawlength = 0;
        public static double centeroffset = 0;
        public static double stackoffset = 0;//5;
        public static double stackyoffset = 0;
        public static double turnoffset = 0;//18;
        public static double parkingoffset = 3;
        public static double parkingoffsetx = 0;
        public static double highconeoffset = 2;
        public static double clawoffset = 2;

    }
    public double border;


    public enum sides {
        UP (-90),
        LEFT (0),
        RIGHT (180),
        DOWN (90);
        public double heading;
        sides (double heading) {
            this.heading = heading;
        }

        public double heading() { return this.heading; }
    }


    public double width;
    public double length;
    public Field.autoZones autoZone;
    public Pose2d lastPose;
    public TrajectorySequenceBuilder trajectory;
    private Telemetry telemetry;

    public FieldTrajectorySequence(TrajectorySequenceBuilder t, Pose2d startPose, double rW, double rL, Field.autoZones aZ, Telemetry realTelemetry) {
        //  super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);;
        width = rW;
        length = rL;
        autoZone = aZ;
        trajectory = t;
        lastPose = startPose;
        telemetry = realTelemetry;
        // Space on either side of the robot as it moves on a street
        border = (24-getDimension())/2;
    }



    // Uses the map to make a trajectory sequence
    //TODO: add optional speed controls.
    public FieldTrajectorySequence toLocation(Pose2d toPose, boolean xfirst) {
        Pose2d startPose;
        // Get starting position
        try {
            startPose = trajectory.build().end();
        } catch (EmptySequenceException e) {
            startPose = lastPose;
        }

        // Checks if a direct route will run into a cone
        if (doesIntersects(startPose, getStreetNum(toPose.getX()), 0, false) && doesIntersects(startPose, 0, getStreetNum(toPose.getY()), true))
        {
            // Gets the street we're starting on
            Pose2d startStreet = new Pose2d(getXStreet(startPose, startPose), getYStreet(startPose, startPose), startPose.getHeading());
            double startc;
            if (!xfirst) {
                // Centers robot's x coordinate if it's not in a street
                if ((doesIntersects(startPose, false))) {
                    startc = startPose.getY();
                    trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), startPose.getY(), startPose.getHeading()));
                } else {
                    startc = startPose.getY();
                }
                // Moves us in the y direction to the correct left/right street if not already there
                if (getYStreet(toPose, startStreet) != startc || (!inStreet(toPose, getStreetNum(startStreet.getY()), true)) && (!inStreet(toPose, getStreetNum(startStreet.getX()), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), getYStreet(toPose, startStreet), startPose.getHeading()));
                }
                // Moves us in the x direction to correct tile if needed
                if (startStreet.getX() != getXStreet(toPose, startStreet) & (!inStreet(toPose, getStreetNum(getYStreet(toPose, startStreet)), true))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
                }
            } else {
                // The above but adjusts y then x then y
                if (doesIntersects(startPose, true)) {
                    startc = startPose.getX();
                    trajectory.lineToLinearHeading(new Pose2d(startPose.getX(), startStreet.getY(), startPose.getHeading()));
                } else {
                    startc = startPose.getX();
                }
                if ((startc != getXStreet(toPose, startStreet)) & (!inStreet(toPose, getStreetNum(startStreet.getY()), true)) & (!inStreet(toPose, getStreetNum(startStreet.getX()), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), startStreet.getY(), startStreet.getHeading()));
                }
                if ((getYStreet(toPose, startStreet) != startStreet.getY()) & (!inStreet(toPose, getStreetNum(getXStreet(toPose, startStreet)), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
                }

            }
        }
        lastPose = toPose;
        // Finishes by taking us to the right place in the tile
        trajectory.lineToLinearHeading(toPose);
        return this;

    }

//

    public FieldTrajectorySequence setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        trajectory.setVelConstraint(velConstraint);
        return this;
    }
    public FieldTrajectorySequence setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint){
        trajectory.setAccelConstraint(accelConstraint);
        return this;
    }

    public FieldTrajectorySequence turn(double degrees) {
        trajectory.turn(Math.toRadians(degrees));
        return this;
    }

    public FieldTrajectorySequence toPole(int poleX, int poleY, sides side, boolean backwardsDrop, boolean xfirst) {
        Pose2d targetPole = getTargetPole(poleX, poleY, side.heading(), backwardsDrop);
        return toLocation(targetPole, xfirst);
    }

    public FieldTrajectorySequence toPole(int poleX, int poleY, double heading, boolean backwardsDrop, boolean xfirst, Pose2d offset) {
        Pose2d targetPole = getTargetPole(poleX, poleY, heading, backwardsDrop);
        targetPole = targetPole.plus(offset);
        return toLocation(targetPole, xfirst);
    }

    public FieldTrajectorySequence toPole(int poleX, int poleY, double heading, boolean backwardsDrop, boolean xfirst) {
        Pose2d targetPole = getTargetPole(poleX, poleY, heading, backwardsDrop);
        return toLocation(targetPole, xfirst);
    }
    public FieldTrajectorySequence toPole(int poleX, int poleY, boolean backwardsDrop, boolean xfirst) {
        Pose2d targetPole = getTargetPole(poleX, poleY, backwardsDrop);
        return toLocation(targetPole, xfirst);
    }
    public FieldTrajectorySequence toPole(boolean backwardsDrop, boolean xfirst) {
        double [] poleCoords = getNearestPole();
        int poleX = (int)poleCoords[0];
        int poleY = (int)poleCoords[1];
        Pose2d targetPole = getTargetPole(poleX, poleY, backwardsDrop);
        return toLocation(targetPole, xfirst);
    }
    // Generates a trajectory sequence to a zone based on our detection.
    public FieldTrajectorySequence toSignalZone(int zone) {
        double x;
        double y;
        switch (this.autoZone) {
            case REDRIGHT:
                x = (zone*24)-(12+FieldTrajContstants.parkingoffsetx);
                y = -((getDimension()/2)+border+FieldTrajContstants.parkingoffset);
                toLocation( new Pose2d(x, y, Math.toRadians(180)), false);
                break;
            case REDLEFT:
                x = -(((4-zone)*24)-(12+ FieldTrajContstants.parkingoffsetx));
                y = -((getDimension()/2)+border+FieldTrajContstants.parkingoffset);
                toLocation(new Pose2d(x, y, Math.toRadians(0)), false);
                break;
            case BLUERIGHT:
                x = -((zone*24)-(12+ FieldTrajContstants.parkingoffsetx));
                y = (getDimension()/2)+border+FieldTrajContstants.parkingoffset;
                toLocation(new Pose2d(x, y, Math.toRadians(0)), false);
                break;
            case BLUELEFT:
                x = (((4-zone)*24)-(12+ FieldTrajContstants.parkingoffsetx));
                y = (getDimension()/2)+border+FieldTrajContstants.parkingoffset;
                toLocation(new Pose2d(x, y, Math.toRadians(180)), false);
                break;

        }
        return this;
    }


    // Trajectory sequence for the stack of cones corresponding to a starting position




    public FieldTrajectorySequence toStack(boolean xfirst, int cycle) {
        double x =  72-(getDistance()+FieldTrajContstants.stackoffset);
        double y = 12+FieldTrajContstants.stackyoffset;

        telemetry.log().add("toStack "+x);
        telemetry.update();

        switch (autoZone) {
            case REDRIGHT:
                toLocation(new Pose2d(x, -y, Math.toRadians(0+FieldTrajContstants.turnoffset)), xfirst);
                break;
            case REDLEFT:
                toLocation(new Pose2d(-x, -y, Math.toRadians(180-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUERIGHT:
                toLocation(new Pose2d(-x, y, Math.toRadians(180-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUELEFT:
                toLocation(new Pose2d(x, y, Math.toRadians(0+FieldTrajContstants.turnoffset)), xfirst);
                break;
        }
        return  this;
    }
    public FieldTrajectorySequence toStack(boolean xfirst, int cycle, Pose2d offset) {
        double x =  72-(getDistance()+offset.getX());
        double y = 12+FieldTrajContstants.stackyoffset+offset.getY();
        if (cycle == 1) {
            x += 2;
        }

        telemetry.log().add("toStack "+x);
        telemetry.update();

        switch (autoZone) {
            case REDRIGHT:
                toLocation(new Pose2d(x, -y, Math.toRadians(0+FieldTrajContstants.turnoffset)), xfirst);
                break;
            case REDLEFT:
                toLocation(new Pose2d(-x, -y, Math.toRadians(180-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUERIGHT:
                toLocation(new Pose2d(-x, y, Math.toRadians(180-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUELEFT:
                toLocation(new Pose2d(x, y, Math.toRadians(0+FieldTrajContstants.turnoffset)), xfirst);
                break;
        }
        return  this;
    }

    public FieldTrajectorySequence toStack(boolean xfirst, int cycle, double heading) {

        double distance =         (getDistance());
        double x =  72;
        double y = 12;
        double[] offset = getOffsetFromDegrees(distance, heading);

//        x -= offset[0];
//        y -= offset[1];

        switch (autoZone) {
            case REDRIGHT:
                toLocation(new Pose2d(x-offset[0], -y-offset[1], Math.toRadians(heading+FieldTrajContstants.turnoffset)), xfirst);
                break;
            case REDLEFT:
                toLocation(new Pose2d(-x-offset[0], -y-offset[1], Math.toRadians(heading-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUERIGHT:
                toLocation(new Pose2d(-x-offset[0], y-offset[1], Math.toRadians(heading-FieldTrajContstants.turnoffset)), xfirst);
                break;
            case BLUELEFT:
                toLocation(new Pose2d(x-offset[0], y-offset[1], Math.toRadians(heading+FieldTrajContstants.turnoffset)), xfirst);
                break;
        }
        return  this;
    }
    public FieldTrajectorySequence addMarker(MarkerCallback callback, double delay) {
        double currentDuration;
        try {
            currentDuration = trajectory.build().duration();
        } catch (EmptySequenceException e) {
            currentDuration = 0;
        }
        trajectory.addTemporalMarker(delay, callback);
        return this;
    }

    public FieldTrajectorySequence wait(double seconds) {
        trajectory.waitSeconds(seconds);
        return this;
    }

    public TrajectorySequence build() {
        return trajectory.build();
    }

    // Determines if the robot is currently not in any street in a particular dimension
    public boolean doesIntersects(Pose2d start, boolean x) {
        double minX;
        double maxX;
        double minY;
        double maxY;
        // Locates the four corners of the robot
        if (Math.toDegrees(start.getHeading()) == 0 || Math.toRadians(start.getHeading()) == 180) {
            minX = start.getX()-length/2;
            maxX = start.getX()+length/2;
            minY = start.getY()-width/2;
            maxY = start.getY()+width/2;
        } else {
            minX = start.getX()-width/2;
            maxX = start.getX()+width/2;
            minY = start.getY()-length/2;
            maxY = start.getY()+length/2;
        }
        // Determines what tile we might be on
        double xStreet = getStreetNum(getXStreet(start, start));
        double yStreet = getStreetNum(getYStreet(start, start));
        double tileStartX = (xStreet*24)+1;
        double tileEndX = (xStreet*24)+(23);
        double tileStartY = (yStreet*24)+1;
        double tileEndY = (yStreet*24)+(23);
        // Checks if we're not in the tile in the dimension we care about
        if ((minX >= tileStartX) & (maxX <= tileEndX) & (!x)) {
            return false;
        } else if ( (x) & (minY >= tileStartY) & (maxY <= tileEndY)){
            return false;
        } else {
            return true;
        }
//        if ((Math.abs(start % 24) > (getDimension()/2)-6 ) & Math.abs(start % 24) <  (24 - ((getDimension() /2)-6))) {
//            return false;
//        } else {
//            return true;
//        }
    }

    // Determines if the robot is not in a particular street in a particular dimension
    public boolean doesIntersects(Pose2d start, double xStreet, double yStreet, boolean x) {
        // Basically the same process as the previous function
        double minX;
        double maxX;
        double minY;
        double maxY;
        if (Math.toDegrees(start.getHeading()) == 0 || Math.toRadians(start.getHeading()) == 180) {
            minX = start.getX()-(length/2+FieldTrajContstants.clawlength);
            maxX = start.getX()+(length/2+FieldTrajContstants.clawlength);
            minY = start.getY()-width/2;
            maxY = start.getY()+width/2;
        } else {
            minX = start.getX()-width/2;
            maxX = start.getX()+width/2;
            minY = start.getY()-(length/2-FieldTrajContstants.clawlength);
            maxY = start.getY()+(length/2-FieldTrajContstants.clawlength);
        }
        double tileStartX = (xStreet*24)+1;
        double tileEndX = (xStreet*24)+(23);
        double tileStartY = (yStreet*24)+1;
        double tileEndY = (yStreet*24)+(23);
        if ((minX >= tileStartX) & (maxX <= tileEndX) & (!x)) {
            return false;
        } else if ( (x) & (minY >= tileStartY) & (maxY <= tileEndY)){
            return false;
        } else {
            return true;
        }
//        if ((Math.abs(start % 24) > (getDimension()/2)-6 ) & Math.abs(start % 24) <  (24 - ((getDimension() /2)-6))) {
//            return false;
//        } else {
//            return true;
//        }
    }
    //more accurate than others but also breaks auto
    public boolean doesIntersects(Pose2d start, Pose2d end, boolean x) {
        double xShift = (end.getX()-start.getX());
        double yShift = (end.getY()-start.getY());
        xShift /= 100;
        yShift /= 100;
        double currX = start.getX();
        double currY = start.getY();
        boolean stillOnPath  = true;
        boolean intersects = false;
        double max = Math.sqrt(Math.pow(width, 2) + Math.pow(length, 2))/2;
        while (stillOnPath) {
            intersects = doesIntersects(new Pose2d(currX, currY, 90), x);
            if (intersects) {
                break;
            }
            currX += xShift;
            currY += yShift;
            if ((currX >= end.getX())) {
                stillOnPath = false;
            }



        }
        return intersects;
    }



    // Determines if the robot is in a specific street
    public boolean inStreet(Pose2d loc, double street, boolean x ) {
        double coord;
        if (x) {
            coord = loc.getY();
        } else {
            coord = loc.getX();
        }
        if ((coord > (street*24)+(getDimension()/2)) & (coord < (((street)*24))+(24-getDimension()/2) )) {
            return true;
        } else {
            return false;
        }
    }

    // Gets the bigger dimension of the robot
    public double getDimension() {
        if (this.length > this.width) {
            return this.length;
        } else {
            return this.width;
        }
    }

    // Determines which street a coordinate corresponds to. Streets numbered -3 through 2.
    public double getStreetNum(double lane) {
        return Math.floor(lane/24);
    }

    // Determines what street running up/down we should use to get to pos (we will move in the x direction to get there later)
    public double getXStreet(Pose2d pos, Pose2d start) {
        // If we're going between two streets, chose the one closer to the robot
        if (pos.getX()%24 == 0) {
            double streetNum = pos.getX()/24;
            double lane1 = (streetNum * 24) + border + getDimension() / 2;
            double lane2 = ((streetNum-1) * 24) + border + getDimension() / 2;
            if (Math.abs(lane1-start.getX()) <= Math.abs(lane2-start.getX())) {
                return lane1;
            } else {
                return lane2;
            }
        } else {
            // Otherwise choose the one closest to where we're going
            double streetNum = Math.floor(pos.getX() / 24);
            double lane = (streetNum * 24) + border + getDimension() / 2;
            return lane;
        }
    }

    // Same thing for left/right streets
    public double getYStreet(Pose2d pos, Pose2d start) {
        if (pos.getY()%24 == 0) {
            double streetNum = pos.getY()/24;
            double lane1 = (streetNum * 24) + border + getDimension() / 2;
            double lane2 = ((streetNum-1) * 24) + border + getDimension() / 2;
            if (Math.abs(lane1-start.getY()) <= Math.abs(lane2-start.getY())) {
                return lane1;
            } else {
                return lane2;
            }
        } else {
            double streetNum = Math.floor(pos.getY() / 24);
            double lane = (streetNum * 24) + border + getDimension() / 2;
            return lane;
        }
    }

    public double[] getNearestPole() {
        double rX = clamp(Math.round(lastPose.getX()/24), -2, 2);
        double rY = clamp(Math.round(lastPose.getY()/24), -2, 2);
        return new double[] {rX, rY};



    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }



    public double getDistance() {
        return FieldTrajContstants.clawoffset;
    }

    public double[] getOffsetFromDegrees(double distance, double degrees) {
        double x = Math.cos(Math.toRadians(degrees)) * distance;
        double y = Math.sin(Math.toRadians(degrees)) * distance;
        return new double[] {x, y};
    }

    //    public double getNearestDropPoint(int poleX, int poleY) {
//        return Math.toDegrees(Math.atan((lastPose.getY()-poleY)/(lastPose.getX()-poleX)));
//    }
    public double[] getNearestDropPoint(int cx, int cy, double r) {
        // Calculate the length of the line segment from the center of the circle to the given point
        double px = lastPose.getX();
        double py = lastPose.getY();
        double len = Math.sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
        // Calculate the coordinates of the point on the circle that is closest to the given point
        double x = cx + r * (px - cx) / len;
        double y = cy + r * (py - cy) / len;
        return new double[]{x, y};
    }


    // Uses a coordinate system to find the robot's position at a junction (corner) from some particular angle. Coordinates (-2,-2) through (2,2)
//    public Pose2d getTargetPole(int poleX, int poleY, sides side, boolean backwardsDrop) {
//        double poleoffset = getDistanceFromPole(poleX, poleY, backwardsDrop);
//        poleX *= 24;
//        poleY *= 24;
//
//        int heading = 0;
//        switch (side) {
//            case LEFT:
//                poleX -= poleoffset;
//                poleY += centeroffset;
//                heading = 0;
//                if (backwardsDrop) heading = 180;
//                break;
//            case RIGHT:
//                poleX += poleoffset;
//                poleY += centeroffset;
//                heading = 180;
//                if (backwardsDrop) heading = 0;
//                break;
//            case UP:
//                poleY += poleoffset;
//                poleX += centeroffset;
//                heading = -90;
//                if (backwardsDrop) heading = 90;
//                break;
//            case DOWN:
//                poleY -= poleoffset;
//                poleX += centeroffset;
//                heading = 90;
//                if (backwardsDrop) heading = -90;
//                break;
//
//        }
//        return new Pose2d(poleX, poleY, Math.toRadians(heading));
//    }

    public Pose2d getTargetPole(int poleX, int poleY, double heading, boolean backwardsDrop) {
        double poleoffset = getDistance();
        poleX *= 24;
        poleY *= 24;
        double[] offset = getOffsetFromDegrees(poleoffset, heading);

        poleX -= offset[0];
        poleY -= offset[1];
        if (backwardsDrop) heading = 180+heading;
        return new Pose2d(poleX, poleY, Math.toRadians(heading));
    }
    public Pose2d getTargetPole(int poleX, int poleY, boolean backwardsDrop) {

        double poleoffset = getDistance();
        poleX *= 24;
        poleY *= 24;
        double[] offset = getNearestDropPoint(poleX, poleY, poleoffset);
        // double heading = 0;
        double heading = Math.toDegrees(Math.atan2(offset[1] - lastPose.getY(), offset[0] - lastPose.getX()));
        double distanceToPole = Math.sqrt(Math.pow(poleY-lastPose.getY(), 2)+ Math.pow(poleX-lastPose.getX(), 2));
        if (distanceToPole < poleoffset) heading += 180;
        //double[] offset = getOffsetFromDegrees(poleoffset, heading);
        if (backwardsDrop) heading = 180+heading;
        poleX = (int)offset[0];
        poleY = (int)offset[1];
        return new Pose2d(poleX, poleY, Math.toRadians(heading));
    }

}