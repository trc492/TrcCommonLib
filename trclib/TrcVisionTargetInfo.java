/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.Locale;

/**
 * This class stores the info for a vision detected target.
 */
public class TrcVisionTargetInfo<O extends TrcVisionTargetInfo.ObjectInfo>
{
    /**
     * This class abstracts the detected object info for different vision processors. This is intended to be extended
     * by various vision processors and requires them to provide a method to return the detected object rect.
     */
    public static abstract class ObjectInfo
    {
        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        public abstract Rect getRect();

    }   //class ObjectInfo

    public O detectedObj;
    public int imageWidth;
    public int imageHeight;
    public Point distanceFromImageCenter;
    public Point distanceFromCamera;
    public double targetWidth;
    public double horizontalAngle;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param detectedObj specifies the detected object.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param homographyMapper specifies the homography mapper, can be null if not provided in which case
     *        distanceFromCamera, targetWidth and horizontalAngle will not be determined.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     */
    public TrcVisionTargetInfo(
        O detectedObj, int imageWidth, int imageHeight, TrcHomographyMapper homographyMapper,
        double objHeightOffset, double cameraHeight)
    {
        Rect rect = detectedObj.getRect();

        this.detectedObj = detectedObj;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        distanceFromImageCenter = new Point(
            rect.x + rect.width/2.0 - imageWidth/2.0, rect.y + rect.height/2.0 - imageHeight/2.0);

        if (homographyMapper != null)
        {
            Point bottomLeft = homographyMapper.mapPoint(new Point(rect.x, rect.y + rect.height));
            Point bottomRight = homographyMapper.mapPoint(new Point(rect.x + rect.width, rect.y + rect.height));
            distanceFromCamera = new Point((bottomLeft.x + bottomRight.x)/2.0, (bottomLeft.y + bottomRight.y)/2.0);
            targetWidth = bottomRight.x - bottomLeft.x;
            double horiAngleRadian = Math.atan2(distanceFromCamera.x, distanceFromCamera.y);
            horizontalAngle = Math.toDegrees(horiAngleRadian);
            if (objHeightOffset > 0.0)
            {
                double adjustment =
                    objHeightOffset * TrcUtil.magnitude(distanceFromCamera.x, distanceFromCamera.y) / cameraHeight;
                double xAdjustment = adjustment * Math.sin(horiAngleRadian);
                double yAdjustment = adjustment * Math.cos(horiAngleRadian);
                distanceFromCamera.x -= xAdjustment;
                distanceFromCamera.y -= yAdjustment;
            }
        }
    }   //TrcVisionTargetInfo

    /**
     * This method returns the string form of the target info.
     *
     * @return string form of the target info.
     */
    @Override
    public String toString()
    {
        String s;

        if (distanceFromCamera != null)
        {
            s = String.format(
                Locale.US,
                "Obj=%s image(w=%d,h=%d) distImageCenter=%s distCamera=%s targetWidth=%.1f horiAngle=%.1f",
                detectedObj, imageWidth, imageHeight, distanceFromImageCenter, distanceFromCamera, targetWidth,
                horizontalAngle);
        }
        else
        {
            s = String.format(
                Locale.US,
                "Obj=%s image(w=%d,h=%d) distImageCenter=%s",
                detectedObj, imageWidth, imageHeight, distanceFromImageCenter);
        }

        return s;
    }   //toString

}   //class TrcVisionTargetInfo
