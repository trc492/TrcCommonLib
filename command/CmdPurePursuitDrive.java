/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.command;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcHolonomicPurePursuitDrive;
import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcWaypoint;

/**
 * This class implements a generic Pure Pursuit Drive command. It allows the caller to specify the drive path by
 * calling start with an array of Waypoint poses.
 */
public class CmdPurePursuitDrive implements TrcRobot.RobotCommand
{
    private final TrcHolonomicPurePursuitDrive purePursuitDrive;
    private final TrcEvent event;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param posPidCoeff specifies the PID coefficients for position control.
     * @param turnPidCoeff specifies the PID coefficients for turn control.
     * @param velPidCoeff specifies the PID coefficients for velocity control.
     */
    public CmdPurePursuitDrive(
            TrcDriveBase driveBase, TrcPidController.PidCoefficients posPidCoeff,
            TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
    {
        purePursuitDrive = new TrcHolonomicPurePursuitDrive(
                "PurePursuitDrive", driveBase, 6, 3.0, 2,
                posPidCoeff, turnPidCoeff, velPidCoeff);
        event = new TrcEvent("event");
    }   //CmdPurePursuitDrive

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param timeout specifies the maximum time allowed for this operation.
     * @param incrementalPath specifies true if each point in the path is relative to its previous point, false if
     *                        each point in the path is relative to the reference pose which is the pose when the
     *                        path following starts.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(double timeout, boolean incrementalPath, TrcPose2D... poses)
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[poses.length];

        for (int i = 0; i < waypoints.length; i++)
        {
            waypoints[i] = new TrcWaypoint(incrementalPath? poses[i].relativeTo(poses[0]): poses[i], null);
        }

        purePursuitDrive.start(new TrcPath(true, waypoints), event, timeout);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param absolutePoses specifies true if the poses in the array are relative to the first pose, false if
     *                      relative to the previous pose in the array.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(boolean absolutePoses, TrcPose2D... poses)
    {
        start(0.0, absolutePoses, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcPose2D... poses)
    {
        start(0.0, false, poses);
    }   //start

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return purePursuitDrive.isActive();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        purePursuitDrive.cancel();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        return event.isSignaled();
    }   //cmdPeriodic

}   //CmdPurePursuitDrive
