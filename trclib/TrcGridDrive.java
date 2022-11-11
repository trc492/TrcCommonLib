/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package TrcCommonLib.trclib;

import java.util.ArrayList;

/**
 * This class implements the auto-assist grid drive. It allows the driver to use the DPad to quickly navigate the
 * field maze in grid cell units in a square pattern accurately without the risk of running into obstacles at grid
 * cell intersections. This algorithm assumes we have accurate odometry. If we don't, all bets are off.
 */
public class TrcGridDrive
{
    private static final String moduleName = "TrcGridDrive";

    private final TrcDriveBase driveBase;
    private final TrcPurePursuitDrive purePursuitDrive;
    private final double gridCellSize;
    private final double turnStartAdj;
    private final double turnEndAdj;
    private final TrcTaskMgr.TaskObject gridDriveTaskObj;
    private final ArrayList<TrcPose2D> gridDriveQueue = new ArrayList<>();
    private TrcDbgTrace msgTracer = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param purePursuitDrive specifies the pure pursuit drive object.
     * @param gridCellSize specifies the grid cell size in inches.
     * @param turnStartAdj specifies the distance adjustment for the previous segment endpoint before the turn.
     * @param turnEndAdj specifies the distance adjustment for the nextsegment startpoint after the turn.
     */
    public TrcGridDrive(
        TrcDriveBase driveBase, TrcPurePursuitDrive purePursuitDrive, double gridCellSize, double turnStartAdj,
        double turnEndAdj)
    {
        this.driveBase = driveBase;
        this.purePursuitDrive = purePursuitDrive;
        this.gridCellSize = gridCellSize;
        this.turnStartAdj = turnStartAdj;
        this.turnEndAdj = turnEndAdj;
        gridDriveTaskObj = TrcTaskMgr.createTask("gridDriveTask", this::gridDriveTask);
    }   //TrcGridDrive

    /**
     * This method enables/disables tracing for the auto-assist task.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    /**
     * This method cancels Grid Drive if one is in progress.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "Canceling Grid Drive.");
        }
        // Note: this is a cooperative multi-task, so don't need to be thread-safe.
        setTaskEnabled(false);
        gridDriveQueue.clear();
        purePursuitDrive.cancel();
        driveBase.releaseExclusiveAccess(moduleName);
    }   //cancel

    /**
     * This method checks if Grid Drive is currently in progress.
     *
     * @return true if Grid Drive is active, false otherwise.
     */
    public boolean isGridDriveActive()
    {
        return gridDriveTaskObj.isRegistered();
    }   //isGridDriveActive

    /**
     * This method enables/disables the Grid Drive task.
     *
     * @param enabled specifies true to enable Grid Drive task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";
        boolean taskActive = isGridDriveActive();

        if (!taskActive && enabled)
        {
            gridDriveTaskObj.registerTask(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK);
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "Enabling task.");
            }
        }
        else if (taskActive && !enabled)
        {
            gridDriveTaskObj.unregisterTask();
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "Disabling task.");
            }
        }
    }   //setTaskEnabled

    /**
     * This method resets the drive base odometry to the nearest grid cell center. Odometry may get inaccurate for
     * various reasons such as odometry wheel slippage, sensor drifting etc. This situation can be corrected by
     * manually driving the robot to the center of a grid cell and call this method. It assumes the drift is within
     * the current grid cell. Therefore, it resets the odometry back to the nearest grid cell center.
     */
    public void resetGridCellCenter()
    {
        TrcPose2D robotPose = driveBase.getFieldPosition();
        TrcPose2D gridCenterPose = new TrcPose2D(
            gridCellCenterPosition(robotPose.x) * gridCellSize,
            gridCellCenterPosition(robotPose.y) * gridCellSize,
            gridCellHeading(robotPose.angle));
        driveBase.setFieldPosition(gridCenterPose);
    }   //resetGridCellCenter

    /**
     * This method adds an X movement segment to the array list.
     *
     * @param gridCells specifies the X movement in number of grid cells, positive for East, negative for West.
     */
    public void setRelativeXGridTarget(int gridCells)
    {
        if (gridCells != 0.0)
        {
            gridDriveQueue.add(new TrcPose2D(gridCells, 0.0, 0.0));
            setTaskEnabled(true);
        }
    }   //setRelativeXGridTarget

    /**
     * This method adds an Y movement segment to the array list.
     *
     * @param gridCells specifies the Y movement in number of grid cells, positive for North, negative for South.
     */
    public void setRelativeYGridTarget(int gridCells)
    {
        if (gridCells != 0.0)
        {
            gridDriveQueue.add(new TrcPose2D(0.0, gridCells, 0.0));
            setTaskEnabled(true);
        }
    }   //setRelativeYGridTarget

    /**
     * This method is called periodically to process the drive queue.
     *
     * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private void gridDriveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (!purePursuitDrive.isActive())
        {
            if (gridDriveQueue.isEmpty())
            {
                // Nothing to do, cleanup and disable task.
                // The task will be enabled when something is added to the drive queue.
                cancel();
            }
            else
            {
                // Something in the drive queue, go process the queue.
                startGridDrive();
            }
        }
    }   //gridDriveTask

    /**
     * This method processes the gridDriveQueue to build a path for PurePursuitDrive to follow.
     */
    private void startGridDrive()
    {
        final String funcName = "startGridDrive";

        if (driveBase.acquireExclusiveAccess(moduleName))
        {
            TrcPose2D robotPose = driveBase.getFieldPosition();
            TrcPose2D startGridPose = new TrcPose2D(
                gridCellCenterPosition(robotPose.x) * gridCellSize,
                gridCellCenterPosition(robotPose.y) * gridCellSize,
                gridCellHeading(robotPose.angle));
            TrcPose2D prevSegment = new TrcPose2D(0.0, 0.0, startGridPose.angle);
            // The first point of the path is center of the grid cell the robot is on.
            TrcPathBuilder pathBuilder = new TrcPathBuilder(robotPose, false).append(startGridPose);

            if (msgTracer != null)
            {
                msgTracer.traceInfo(
                    funcName, ">>>>> robotPose=%s, startGridPose=%s, QSize=%d",
                    robotPose, startGridPose, gridDriveQueue.size());
            }

            while (gridDriveQueue.size() > 0)
            {
                TrcPose2D nextSegment = gridDriveQueue.get(0);

                if (msgTracer != null)
                {
                    msgTracer.traceInfo(
                        funcName, "Adjacent Segments: prevSegment=%s, nextSegment=%s", prevSegment, nextSegment);
                }

                if (!willTurn(prevSegment, nextSegment))
                {
                    // Not turning, can coalesce the nextSegment to the prevSegment.
                    prevSegment.x += nextSegment.x;
                    prevSegment.y += nextSegment.y;
                    gridDriveQueue.remove(nextSegment);

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Coalesced compatible segments: prevSegment=%s (QSize=%d)",
                            prevSegment, gridDriveQueue.size());
                    }
                }
                else
                {
                    TrcPose2D prevEndPoint = prevSegment.clone();
                    TrcPose2D nextStartPoint = new TrcPose2D();
                    TrcPose2D nextSegmentPoint = nextSegment.clone();
                    // Turning, create an endpoint for the first segment and a startpoint for the next segment.
                    if (prevSegment.angle == 0.0)
                    {
                        // Heading is North.
                        prevEndPoint.y += turnStartAdj;
                        if (nextSegment.x > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + turnEndAdj;
                            nextStartPoint.y = prevSegment.y + 1.0;
                            nextStartPoint.angle = 90.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - turnEndAdj;
                            nextStartPoint.y = prevSegment.y + 1.0;
                            nextStartPoint.angle = 270.0;
                        }
                        nextSegmentPoint.x += prevSegment.x;
                        nextSegmentPoint.y = nextStartPoint.y;
                    }
                    else if (prevSegment.angle == 180.0)
                    {
                        // Heading is South.
                        prevEndPoint.y -= turnStartAdj;
                        if (nextSegment.x > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + turnEndAdj;
                            nextStartPoint.y = prevSegment.y - 1.0;
                            nextStartPoint.angle = 90.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - turnEndAdj;
                            nextStartPoint.y = prevSegment.y - 1.0;
                            nextStartPoint.angle = 270.0;
                        }
                        nextSegmentPoint.x += prevSegment.x;
                        nextSegmentPoint.y = nextStartPoint.y;
                    }
                    else if (prevSegment.angle == 90.0)
                    {
                        // Headihg is East.
                        prevEndPoint.x += turnStartAdj;
                        if (nextSegment.y > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x + 1.0;
                            nextStartPoint.y = prevSegment.y + turnEndAdj;
                            nextStartPoint.angle = 0.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x + 1.0;
                            nextStartPoint.y = prevSegment.y - turnEndAdj;
                            nextStartPoint.angle = 180.0;
                        }
                        nextSegmentPoint.x = nextStartPoint.x;
                        nextSegmentPoint.y += prevSegment.y;
                    }
                    else if (prevSegment.angle == 270.0)
                    {
                        // Heading is West.
                        prevEndPoint.x -= turnStartAdj;
                        if (nextSegment.y > 0.0)
                        {
                            nextStartPoint.x = prevSegment.x - 1.0;
                            nextStartPoint.y = prevSegment.y + turnEndAdj;
                            nextStartPoint.angle = 0.0;
                        }
                        else
                        {
                            nextStartPoint.x = prevSegment.x - 1.0;
                            nextStartPoint.y = prevSegment.y - turnEndAdj;
                            nextStartPoint.angle = 180.0;
                        }
                        nextSegmentPoint.x = nextStartPoint.x;
                        nextSegmentPoint.y += prevSegment.y;
                    }
                    nextSegmentPoint.angle = nextStartPoint.angle;

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Turn Adjustments: prevEndPoint=%s, nextStartPoint=%s, nextSegPoint=%s",
                            prevEndPoint, nextStartPoint, nextSegmentPoint);
                    }
                    // Create endpoint of the previous segment.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + prevEndPoint.x * gridCellSize,
                                      startGridPose.y + prevEndPoint.y * gridCellSize,
                                      prevEndPoint.angle));
                    // Create startpoint of the next segment.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + nextStartPoint.x * gridCellSize,
                                      startGridPose.y + nextStartPoint.y * gridCellSize,
                                      nextStartPoint.angle));
                    // Create the next segment point.
                    pathBuilder.append(
                        new TrcPose2D(startGridPose.x + nextSegmentPoint.x * gridCellSize,
                                      startGridPose.y + nextSegmentPoint.y * gridCellSize,
                                      nextSegmentPoint.angle));

                    gridDriveQueue.remove(nextSegment);
                    prevSegment = nextSegment;
                    prevSegment.x = prevSegment.y = 0.0;
                }
            }

            if (prevSegment.x != 0.0 || prevSegment.y != 0.0)
            {
                // The last loop was a coalesce loop. It means we have one more segment to add to the path.
                pathBuilder.append(
                    new TrcPose2D(startGridPose.x + prevSegment.x * gridCellSize,
                                  startGridPose.y + prevSegment.y * gridCellSize,
                                  prevSegment.angle));
            }

            TrcPath path = pathBuilder.toRelativeStartPath();
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "GridDrivePath=%s", path);
            }

             purePursuitDrive.start(moduleName, path, null, null, 0.0);
        }
    }   //startGridDrive

    /**
     * This method rounds the given position value to the center of a grid cell, meaning it will round the value so
     * that the position will be at the center of the grid cell in number of grid units.
     *
     * @param position specifies the field position in inches.
     * @return field position at the center of the grid cell in grid units.
     */
    private double gridCellCenterPosition(double position)
    {
        return Math.signum(position)*(((int) (Math.abs(position) / gridCellSize)) + 0.5);
    }   //gridCellCenterPosition

    /**
     * This method adjusts the heading to the nearest 90-degree multiple.
     *
     * @param heading specifies the heading to be adjusted to the nearest 90-degree multiple.
     * @return adjusted heading in the range of [0.0, 360.0)
     */
    private double gridCellHeading(double heading)
    {
        heading %= 360.0;
        if (heading < 0.0) heading += 360.0;
        return (((int) (heading / 90.0 + 0.5)) * 90.0) % 360.0;
    }   //gridCellHeading

    /**
     * This method determines if the next segment has a compatible heading with the previous segment. If not, the robot
     * will turn.
     *
     * @param prevSegment specifies the previous segment position.
     * @param nextSegment specifies the next segment position.
     * @return true if the robot will turn from the previous segment to the next segment, false otherwise.
     */
    private boolean willTurn(TrcPose2D prevSegment, TrcPose2D nextSegment)
    {
        return (nextSegment.x != 0.0 && (prevSegment.angle == 0.0 || prevSegment.angle == 180.0)) ||
               (nextSegment.y != 0.0 && (prevSegment.angle == 90.0 || prevSegment.angle == 270.0));
    }   //willTurn

}   //class TrcGridDrive
