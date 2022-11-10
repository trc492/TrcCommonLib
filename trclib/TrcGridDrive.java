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
    private final ArrayList<TrcPose2D> gridDriveQueue = new ArrayList<>();
    private TrcDbgTrace msgTracer = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param purePursuitDrive specifies the pure pursuit drive object.
     * @param gridCellSize specifies the grid cell size in inches.
     */
    public TrcGridDrive(TrcDriveBase driveBase, TrcPurePursuitDrive purePursuitDrive, double gridCellSize)
    {
        this.driveBase = driveBase;
        this.purePursuitDrive = purePursuitDrive;
        this.gridCellSize = gridCellSize;
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
     * This method checks if Grid Drive is currently in progress.
     *
     * @return true if Grid Drive is active, false otherwise.
     */
    public boolean isGridDriveActive()
    {
        return purePursuitDrive.isActive();
    }   //isGridDriveActive

    /**
     * This method stops Grid Drive and does the clean up.
     */
    private void stop()
    {
        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }
        driveBase.releaseExclusiveAccess(moduleName);
        msgTracer.traceInfo("GridDriveStop", "stop: releasing ownership.");
        TrcDbgTrace.printThreadStack();
        purePursuitDrive.setIncrementalTurnEnabled(true);
    }   //stop

    /**
     * This method cancels Grid Drive if one is in progress.
     */
    public void cancel()
    {
        stop();
        gridDriveQueue.clear();
    }   //cancel

//    /**
//     * This method enables/disables Grid Drive.
//     *
//     * @param enabled specifies true to enable grid drive, false to disable.
//     */
//    private void setGridDriveEnabled(boolean enabled)
//    {
//        if (enabled && !purePursuitDrive.isActive())
//        {
//            if (driveBase.acquireExclusiveAccess(moduleName))
//            {
//                purePursuitDrive.setIncrementalTurnEnabled(false);
//                startGridDrive();
//            }
//        }
//        else if (!enabled && purePursuitDrive.isActive())
//        {
//            stop();
//        }
//    }   //setGridDriveEnabled

    /**
     * This method adds an X movement segment to the array list.
     *
     * @param gridCells specifies the X movement in number of grid cells, positive for East, negative for West.
     */
    public void setRelativeXGridTarget(int gridCells)
    {
        gridDriveQueue.add(new TrcPose2D(gridCells, 0.0, 0.0));
        startGridDrive();
    }   //setRelativeXGridTarget

    /**
     * This method adds an Y movement segment to the array list.
     *
     * @param gridCells specifies the Y movement in number of grid cells, positive for North, negative for South.
     */
    public void setRelativeYGridTarget(int gridCells)
    {
        gridDriveQueue.add(new TrcPose2D(0.0, gridCells, 0.0));
        startGridDrive();
    }   //setRelativeYGridTarget

    /**
     * This method processes the gridDriveQueue to build a path for PurePursuitDrive to follow.
     */
    private void startGridDrive()
    {
        final String funcName = "startGridDrive";
        final double turnAdj = 0.35;

        msgTracer.traceInfo(funcName, "ppd is %s", purePursuitDrive.isActive()? "Active": "Not Active");
        if (!purePursuitDrive.isActive() && driveBase.acquireExclusiveAccess(moduleName))
        {
            msgTracer.traceInfo(funcName, "Acquired ownership.");
            TrcPose2D robotPose = driveBase.getFieldPosition();
            double robotCellX = gridCellCenterPosition(robotPose.x);
            double robotCellY = gridCellCenterPosition(robotPose.y);
            double robotCellHeading = gridCellHeading(robotPose.angle);
            // The first point of the path is center of the grid cell the robot is on.
            TrcPathBuilder pathBuilder = new TrcPathBuilder(robotPose, true).append(
                new TrcPose2D(
                    robotCellX*gridCellSize - robotPose.x,
                    robotCellY*gridCellSize - robotPose.y,
                    robotCellHeading));
            TrcPose2D lastSegment = new TrcPose2D(0.0, 0.0, robotCellHeading);

            if (msgTracer != null)
            {
                msgTracer.traceInfo(
                    funcName, ">>>>> robotPose=%s, robotCell(%.2f,%.2f,%.2f), QSize=%d",
                    robotPose, robotCellX, robotCellY, robotCellHeading, gridDriveQueue.size());
            }

            purePursuitDrive.setIncrementalTurnEnabled(false);
            while (gridDriveQueue.size() > 0)
            {
                TrcPose2D currSegment = gridDriveQueue.get(0);

                if (msgTracer != null)
                {
                    msgTracer.traceInfo(
                        funcName, "Adjacent Segments: lastSegment=%s, currSegment=%s", lastSegment, currSegment);
                }

                if (willTurn(lastSegment, currSegment))
                {
                    TrcPose2D currSegmentAdj = currSegment.clone();
                    // Turning, create an endpoint for the first segment and a startpoint for the next segment.
                    if (lastSegment.angle == 0.0)
                    {
                        // Heading is North.
                        lastSegment.y += turnAdj;
                        currSegmentAdj.y += 1 - turnAdj;
                        currSegmentAdj.angle = currSegment.x > 0.0 ? 90.0 : 270.0;
                    }
                    else if (lastSegment.angle == 180.0)
                    {
                        // Heading is South.
                        lastSegment.y -= turnAdj;
                        currSegmentAdj.y -= 1 - turnAdj;
                        currSegmentAdj.angle = currSegment.x > 0.0 ? 90.0 : 270.0;
                    }
                    else if (lastSegment.angle == 90.0)
                    {
                        // Headihg is East.
                        lastSegment.x += turnAdj;
                        currSegmentAdj.x += 1 - turnAdj;
                        currSegmentAdj.angle = currSegment.y > 0.0 ? 0.0 : 180.0;
                    }
                    else if (lastSegment.angle == 270.0)
                    {
                        // Heading is West.
                        lastSegment.x -= turnAdj;
                        currSegmentAdj.x -= 1 - turnAdj;
                        currSegmentAdj.angle = currSegment.y > 0.0 ? 0.0 : 180.0;
                    }

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Adjusting for turn: lastSegment=%s, currSegmentAdj=%s", lastSegment,
                            currSegmentAdj);
                    }
                    // Create endpoint of the first segment.
                    pathBuilder.append(
                        new TrcPose2D(lastSegment.x*gridCellSize, lastSegment.y*gridCellSize, 0.0));
                    // Create startpoint of the next segment.
                    pathBuilder.append(
                        new TrcPose2D(currSegmentAdj.x*gridCellSize, currSegmentAdj.y*gridCellSize,
                                      currSegmentAdj.angle - lastSegment.angle));
                    gridDriveQueue.remove(currSegment);
                    lastSegment = currSegment;
                    lastSegment.x = lastSegment.y = 0.0;
                }
                else
                {
                    // Not turning, can coalesce the currSegment to the lastSegment.
                    lastSegment.x += currSegment.x;
                    lastSegment.y += currSegment.y;
                    gridDriveQueue.remove(currSegment);

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "Coalesced compatible segments: lastSegment=%s (QSize=%d)",
                            lastSegment, gridDriveQueue.size());
                    }
                }
            }

            if (lastSegment.x != 0.0 || lastSegment.y != 0.0)
            {
                // Create endpoint of the last segment.
                pathBuilder.append(
                    new TrcPose2D(lastSegment.x*gridCellSize, lastSegment.y*gridCellSize, 0.0));
            }

            TrcPath path = pathBuilder.toRelativeStartPath();
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "GridDrivePath=%s", path);
            }

             purePursuitDrive.start(moduleName, path, null, this::completionCallback, 0.0);
        }
    }   //startGridDrive

    /**
     * This method is called when pure pursuit completed the path. It does the cleanup and releasing the lock on the
     * drive base.
     *
     * @param context not used.
     */
    private void completionCallback(Object context)
    {
        stop();
        if (!gridDriveQueue.isEmpty())
        {
            startGridDrive();
        }
    }   //completionCallback

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
     * @return adjusted heading.
     */
    private double gridCellHeading(double heading)
    {
        heading %= 360.0;
        if (heading < 0.0) heading += 360.0;
        return (((int) (heading / 90.0 + 0.5)) * 90.0) % 360.0;
    }   //gridCellHeading

    /**
     * This method determines if the current segment has a compatible heading with the last segment. If not, the robot
     * will turn.
     *
     * @param lastSegment specifies the last segment position.
     * @param currSegment specifies the current segment position.
     * @return true if the robot will turn from the last segment to the current segment, false otherwise.
     */
    private boolean willTurn(TrcPose2D lastSegment, TrcPose2D currSegment)
    {
        return (currSegment.y == 0.0 && (lastSegment.angle == 0.0 || lastSegment.angle == 180.0)) ||
               (currSegment.x == 0.0 && (lastSegment.angle == 90.0 || lastSegment.angle == 270.0));
    }   //willTurn

}   //class TrcGridDrive
