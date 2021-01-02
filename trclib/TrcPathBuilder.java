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
 * This class builds a TrcPath for path following driving. The path can be built from three types of waypoints:
 * RELATIVE_PATH - points in the path are relative to their previous points.
 * RELATIVE_TO_START_PATH - points in the path are relative to the first point of the path.
 * ABSOLUTE_POSITION_PATH - points in the path are relative to an absolute reference point.
 */
public class TrcPathBuilder
{
    private final ArrayList<TrcWaypoint> waypointList = new ArrayList<>();
    private final boolean inDegrees;
    private final TrcPose2D referencePose;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param inDegrees specifies true if waypoints have headings with degree unit, false with radian units.
     * @param referencePose specifies waypoints are relative to the reference point, null if relative to previous
     *                      points.
     */
    public TrcPathBuilder(boolean inDegrees, TrcPose2D referencePose)
    {
        this.inDegrees = inDegrees;
        this.referencePose = referencePose;
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     *
     * @param inDegrees specifies true if waypoints have headings with degree unit, false with radian units.
     */
    public TrcPathBuilder(boolean inDegrees)
    {
        this(inDegrees, null);
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     *
     * @param referencePose specifies waypoints are relative to the reference point, null if relative to previous
     *                      points.
     */
    public TrcPathBuilder(TrcPose2D referencePose)
    {
        this(true, referencePose);
    }   //TrcPathBuilder

    /**
     * Constructor: Create an instance of the object.
     */
    public TrcPathBuilder()
    {
        this(true, null);
    }   //TrcPathBuilder

    /**
     * Appends the specified pose to the path.
     *
     * @param pose specifies the pose to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D pose)
    {
        if (referencePose == null)
        {
            //
            // The pose is relative to the previous point for an INCREMENTAL_PATH.
            //
            if (waypointList.size() == 0)
            {
                waypointList.add(new TrcWaypoint(new TrcPose2D(0, 0), null));
            }
            TrcPose2D prevPose = waypointList.get(waypointList.size() - 1).pose;
            waypointList.add(new TrcWaypoint(prevPose.addRelativePose(pose), null));
        }
        else
        {
            //
            // The pose is relative to the referencePose. If referencePose is (0, 0), it is a
            // RELATIVE_TO_START_PATH. If referencePose is non-zero, it is an ABSOLUTE_POSITION_PATH.
            //
            waypointList.add(new TrcWaypoint(pose.relativeTo(referencePose), null));
        }

        return this;
    }   //append

    /**
     * Appends the specified poses to the path.
     *
     * @param poses specifies an array of poses to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcPose2D... poses)
    {
        for (TrcPose2D pose: poses)
        {
            append(pose);
        }

        return this;
    }   //append

    /**
     * Appends the specified waypoint to the path.
     *
     * @param waypoint specifies the waypoint to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcWaypoint waypoint)
    {
        return append(waypoint.pose);
    }   //append

    /**
     * Appends the specified waypoints to the path.
     *
     * @param waypoints specifies the array of waypoints to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(TrcWaypoint... waypoints)
    {
        for (TrcWaypoint wp: waypoints)
        {
            append(wp.pose);
        }

        return this;
    }   //append

    /**
     * Appends the specified pose to the path.
     *
     * @param inDegrees specifies true if the pose heading is in degrees, false if in radians.
     * @param pose specifies the pose to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(boolean inDegrees, TrcPose2D pose)
    {
        if (inDegrees != this.inDegrees)
        {
            pose.angle = inDegrees? Math.toRadians(pose.angle): Math.toDegrees(pose.angle);
        }
        append(pose);

        return this;
    }   //append

    /**
     * Appends the specified poses to the path.
     *
     * @param inDegrees specifies true if the pose heading is in degrees, false if in radians.
     * @param poses specifies an array of poses to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(boolean inDegrees, TrcPose2D... poses)
    {
        for (TrcPose2D pose: poses)
        {
            append(inDegrees, pose);
        }

        return this;
    }   //aopend

    /**
     * Appends the specified waypoint to the path.
     *
     * @param inDegrees specifies true if the pose heading is in degrees, false if in radians.
     * @param waypoint specifies the waypoint to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(boolean inDegrees, TrcWaypoint waypoint)
    {
        append(inDegrees, waypoint.pose);

        return this;
    }   //append

    /**
     * Appends the specified waypoints to the path.
     *
     * @param inDegrees specifies true if the pose heading is in degrees, false if in radians.
     * @param waypoints specifies the array of waypoints to be added to the path.
     * @return this instance.
     */
    public TrcPathBuilder append(boolean inDegrees, TrcWaypoint... waypoints)
    {
        for (TrcWaypoint wp: waypoints)
        {
            append(inDegrees, wp.pose);
        }

        return this;
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
