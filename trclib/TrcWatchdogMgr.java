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
import java.util.HashMap;
import java.util.Locale;

import TrcCommonLib.trclib.TrcTaskMgr.TaskType;

/**
 * This class is a singleton. It manages a list of watchdogs. Watchdogs are typically used for detecting thread
 * deadlocks. A thread can register a watchdog with the Watchdog manager and periodically sends a heartbeat to it.
 * If a heart beat is not sent within the heart beat threshold time, a stack trace for the thread will be dump to the
 * trace log. This allows us to catch thread deadlocks quite easily. Since watchdog is designed per thread, only one
 * watchdog needs to be registered for each thread.
 */
public class TrcWatchdogMgr
{
    private static final String moduleName = "TrcWatchdogMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final double DEF_TASK_INTERVAL = 0.5;        // in seconds.
    private static final double DEF_HEARTBEAT_THRESHOLD = 0.5;  // in seconds.
    private static final TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
    private static TrcWatchdogMgr instance;
    private static ArrayList<Watchdog> watchdogList;
    private static HashMap<String, Watchdog> watchdogMap;
    private static TrcTaskMgr.TaskObject watchdogTaskObj;

    /**
     * This class encapsulates the state of the watchdog. A watchdog has an identifiable name, an associated thread,
     * a maximum time interval between which the heart beat must be received and the next heart beat expiraton time.
     */
    public static class Watchdog
    {
        private final String name;
        private final double heartBeatThreshold;
        private final Thread thread;
        private volatile double heartBeatExpiredTime;
        private volatile boolean expired;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the watchdog.
         * @param heartBeatThreshold specifies the maximum heart beat interval in seconds.
         */
        private Watchdog(String name, double heartBeatThreshold)
        {
            this.name = name;
            this.heartBeatThreshold = heartBeatThreshold;
            this.thread = Thread.currentThread();
            this.heartBeatExpiredTime = TrcUtil.getCurrentTime() + heartBeatThreshold;
            this.expired = false;
        }   //Watchdog

        /**
         * This method is called by the thread that registered the watchdog to send a heart beat. This will update
         * the next heart beat expiration time.
         *
         * @return true if the calling thread is the owner of the watchdog, false otherwise.
         */
        public boolean sendHeartBeat()
        {
            boolean success = false;

            if (this.thread == Thread.currentThread())
            {
                synchronized (this)
                {
                    heartBeatExpiredTime = TrcUtil.getCurrentTime() + heartBeatThreshold;
                    expired = false;
                }
            }

            return success;
        }   //sendHeartBeat

        /**
         * This method unregisters this watchdog from Watchdog Manager.
         * Important: this method must be called in the thread the watchdog is monitoring. In other words, the
         * caller's thread must be the owner of the watchdog.
         *
         * @return true if watchdog is successfully unregister, false otherwise.
         */
        public boolean unregister()
        {
            boolean success = false;

            if (this.thread == Thread.currentThread())
            {
                success = unregisterWatchdog(this);
            }

            return success;
        }   //unregsiter

        /**
         * This method returns the string containing the info of the watchdog.
         *
         * @return string form of the watchdog info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "[%.3f] %s: threshold=%.3f, expiredTime=%.3f, expired=%s",
                TrcUtil.getCurrentTime(), name, heartBeatThreshold, heartBeatExpiredTime, expired);
        }   //toString

    }   //class Watchdog

    /**
     * This method should only be called once. It creates a singleton of the Watchdog Manager. If it is called again,
     * it will just return the Watchdog Manager instance created previously.
     *
     * @param taskInterval specifies the watchdog task interval.
     * @return Watchdog Manager instance.
     */
    public static TrcWatchdogMgr getInstance(double taskInterval)
    {
        if (instance == null)
        {
            instance = new TrcWatchdogMgr(taskInterval);
        }

        return instance;
    }   //getInstance

    /**
     * This method should only be called once. It creates a singleton of the Watchdog Manager. If it is called again,
     * it will just return the Watchdog Manager instance created previously.
     *
     * @return Watchdog Manager instance.
     */
    public static TrcWatchdogMgr getInstance()
    {
        return getInstance(DEF_TASK_INTERVAL);
    }   //getInstance

    /**
     * This method is called to shutdown the Watchdog Manager. It basically stops the watchdog task and clears the
     * watchdog list and map.
     */
    public static void Shutdown()
    {
        if (instance != null)
        {
            synchronized (watchdogList)
            {
                watchdogTaskObj.unregisterTask();
                watchdogMap.clear();
                watchdogList.clear();
                instance = null;
            }
        }
    }   //shutdown

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param taskInterval specifies the watchdog task interval.
     */
    private TrcWatchdogMgr(double taskInterval)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        watchdogList = new ArrayList<>();
        watchdogMap = new HashMap<>();
        watchdogTaskObj = TrcTaskMgr.createTask(moduleName, this::watchdogTask);
        watchdogTaskObj.registerTask(TaskType.STANDALONE_TASK, (long) (taskInterval*1000));
    }   //TrcWatchdogMgr

    /**
     * This method registers a new watchdog for the current thread if one is not already registered.
     * Important: this method must be called in the thread the watchdog is monitoring.
     *
     * @param name specifies the name of the watchdog.
     * @param heartBeatThreshold specifies the maximum heart beat interval in seconds.
     * @return newly created watchdog.
     */
    public static Watchdog registerWatchdog(String name, double heartBeatThreshold)
    {
        Watchdog watchdog = null;

        instance = getInstance();
        synchronized (watchdogList)
        {
            if (!watchdogMap.containsKey(name))
            {
                watchdog = new Watchdog(name, heartBeatThreshold);
                watchdogList.add(watchdog);
                watchdogMap.put(name, watchdog);
            }
        }

        return watchdog;
    }   //registerWatchdog

    /**
     * This method registers a new watchdog for the current thread if one is not already registered.
     * Important: this method must be called in the thread the watchdog is monitoring.
     *
     * @param name specifies the name of the watchdog.
     * @return newly created watchdog.
     */
    public static Watchdog registerWatchdog(String name)
    {
        return registerWatchdog(name, DEF_HEARTBEAT_THRESHOLD);
    }   //registerWatchdog

    /**
     * This method removes the watchdog from the watchdog list and map. It can be called from any thread as long as
     * it provides the watchdog to be unregistered.
     *
     * @param watchdog specifies the watchdog to be removed.
     * @return true if watchdog is removed successfully, false if watchdog does not exist.
     */
    public static boolean unregisterWatchdog(Watchdog watchdog)
    {
        boolean success = false;

        synchronized (watchdogList)
        {
            watchdogMap.remove(watchdog.name);
            success = watchdogList.remove(watchdog);
        }

        return success;
    }   //unregisterWatchdog

    /**
     * This method runs periodically to check for watchdog timeouts.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     */
    private void watchdogTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "watchdogTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        synchronized (watchdogList)
        {
            double currTime = TrcUtil.getCurrentTime();

            for (Watchdog watchdog: watchdogList)
            {
                if (!watchdog.expired && currTime > watchdog.heartBeatExpiredTime)
                {
                    watchdog.expired = true;
                    tracer.traceErr(
                        funcName, "[%.3f] watchdog (%s) timed out: %s",
                        currTime, watchdog, watchdog.thread.getStackTrace());
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //watchdogTask

}   //class TrcWatchdogMgr
