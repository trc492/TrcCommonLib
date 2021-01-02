/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class builds a TrcPath for path following driving. The path can be built from two types of waypoints:
 * INCREMENTAL_PATH - points in the path are relative to their previous points (referencePose is set to null).
 * REFERENCE_FRAME_PATH - points in the path are relative to the referencePose (referencePose is non-null).
 */
public class TrcPathBuilder
{
    private final ArrayList<TrcWaypoint> waypointList = new ArrayList<>();
    private final TrcPose2D referencePose;
    private final boolean inDegrees;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param referencePose specifies appended waypoints are relative to the reference point, null if relative to
     *                      previous points.
     * @param inDegrees specifies true if appended waypoints have headings with degree unit, false with radian units.
     */
    public TrcPathBuilder(TrcPose2D referencePose, boolean inDegrees)
    {
        this.referencePose = referencePose;
        this.inDegrees = inDegrees;
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     *
     * @param referencePose specifies appended waypoints are relative to the reference point, null if relative to
     *                      previous points.
     */
    public TrcPathBuilder(TrcPose2D referencePose)
    {
        this(referencePose, true);
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcPathBuilder()
    {
        this(null, true);
    }   //TrcPathBuilder

    /**
     * Appends the specified waypoint to the path.
     *
     * @param waypoint specifies the waypoint to be added to the path. If referencePose is null, waypoint is relative
     *                 to the previous point, otherwise it is relative to the referencePose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcWaypoint waypoint)
    {
        if (referencePose == null)
        {
            //
            // waypoint is relative to the previous point for an INCREMENTAL_PATH.
            //
            if (waypointList.size() == 0)
            {
                waypointList.add(waypoint);
            }
            else
            {
                TrcPose2D prevPose = waypointList.get(waypointList.size() - 1).pose;
                waypoint.pose.setAs(prevPose.addRelativePose(waypoint.pose));
                waypointList.add(waypoint);
            }
        }
        else
        {
            //
            // waypoint is relative to referencePose for a REFERENCE_FRAME_PATH.
            //
            waypoint.pose.setAs(waypoint.pose.relativeTo(referencePose));
            waypointList.add(waypoint);
        }

        return this;
    }   //append

    /**
     * Appends the specified pose and velocity to the path.
     *
     * @param pose specifies the pose to be added to the path. If referencePose is null, pose is relative to the
     *             previous point, otherwise it is relative to the referencePose.
     * @param velocity specifies the velocity at the pose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D pose, TrcPose2D velocity)
    {
        return append(new TrcWaypoint(pose, velocity));
    }   //append

    /**
     * Appends the specified pose to the path.
     *
     * @param pose specifies the pose to be added to the path. If referencePose is null, pose is relative to the
     *             previous point, otherwise it is relative to the referencePose.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D pose)
    {
        return append(new TrcWaypoint(pose, null));
    }   //append

    /**
     * This method returns the TrcPath built.
     *
     * @return resulting TrcPath.
     */
    public TrcPath toPath()
    {
        return new TrcPath(inDegrees, (TrcWaypoint[])waypointList.toArray());
    }   //toPath

}   //class TrcPathBuilding
