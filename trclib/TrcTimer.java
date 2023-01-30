/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * This class implements a timer that will signal an event or make a notification callback when the time has expired.
 * This is useful for doing delays in autonomous, for example.
 */
public class TrcTimer
{
    private static final String moduleName = "TrcTimer";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    /**
     * This class encapsulates the state of a timer that must be updated atomically. Therefore, when accessing this
     * object, you must acquire its synchronized lock.
     */
    private static class State
    {
        AtomicLong expiredTimeInMsec = new AtomicLong(0);
        AtomicBoolean expired = new AtomicBoolean(false);
        AtomicBoolean canceled = new AtomicBoolean(false);
        TrcEvent notifyEvent = null;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "(expiredTimeMS=%d,expired=%s,canceled=%s,event=%s)",
                expiredTimeInMsec.get(), expired.get(), canceled.get(), notifyEvent);
        }   //toString

    }   //class State

    private final State state = new State();
    private final String instanceName;

    /**
     * Constructor: Creates an instance of the timer with the given name.
     *
     * @param instanceName specifies the name to identify this instance of the timer.
     */
    public TrcTimer(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcTimer

    /**
     * This method returns the instance name and the timer state.
     *
     * @return instance name and its state.
     */
    @Override
    public String toString()
    {
        return String.format(Locale.US, "%s=%s", instanceName, state);
    }   //toString

    /**
     * This methods sets the expire time relative to the current time. When the time expires, it will signal the
     * event if provided. It will also do the notification callback if provided.
     *
     * @param time specifies the expire time in seconds relative to the current time.
     * @param event specifies the event to signal when time has expired.
     * @param callback specifies the callback to call when time has expired.
     * @param callbackContext specifies the object to pass back for the callback, can be null if not provided.
     * @throws IllegalArgumentException if both event and callback are null.
     */
    public void set(double time, TrcEvent event, TrcEvent.Callback callback, Object callbackContext)
    {
        final String funcName = "set";

        if (event == null && callback == null)
        {
            throw new IllegalArgumentException("Either event or callback must not be null.");
        }

        if (event != null)
        {
            event.clear();
        }

        synchronized (state)
        {
            //
            // In case the timer is still active, cancel it first.
            // If we are re-arming an active timer, don't notify the event about the cancel.
            //
            cancel(false);
            state.expiredTimeInMsec.set(TrcTimer.getCurrentTimeMillis() + (long)(time*1000));
            state.expired.set(false);
            state.canceled.set(false);
            // Notification callback requires an event. If the caller did not provide one, create one ourselves.
            state.notifyEvent = event != null? event: new TrcEvent(instanceName + ".callbackEvent");
            if (callback != null)
            {
                state.notifyEvent.setCallback(callback, callbackContext);
            }
        }
        addTimer(this);

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "timer=%s, time=%.3f, event=%s, callback=%s, context=%s",
                this, time, event, callback != null, callbackContext != null);
        }
    }   //set

    /**
     * This methods sets the expire time relative to the current time. When the time expires, it will signal the
     * given event.
     *
     * @param time specifies the expire time in seconds relative to the current time.
     * @param event specifies the event to signal when time has expired.
     */
    public void set(double time, TrcEvent event)
    {
        set(time, event, null, null);
    }   //set

    /**
     * This methods sets the expire time relative to the current time. When the time expires, it will do the
     * notificaton callback.
     *
     * @param time specifies the expire time in seconds relative to the current time.
     * @param callback specifies the callback when time has expired.
     * @param callbackContext specifies the object to pass back for the callback, can be null if not provided.
     */
    public void set(double time, TrcEvent.Callback callback, Object callbackContext)
    {
        set(time, null, callback, callbackContext);
    }   //set

    /**
     * This methods sets the expire time relative to the current time. When the time expires, it will do the
     * notificaton callback.
     *
     * @param time specifies the expire time in seconds relative to the current time.
     * @param callback specifies the callback when time has expired.
     */
    public void set(double time, TrcEvent.Callback callback)
    {
        set(time, null, callback, null);
    }   //set

    /**
     * This method cancels the timer if it's set but has not expired. If the timer is canceled, the event is signaled.
     *
     * @param doNotify specifies true to notify the timer owner of the cancellation, false if called internally to
     *                 to re-arm the previous non-expired timer.
     */
    private void cancel(boolean doNotify)
    {
        final String funcName = "cancel";
        TrcEvent event;

        synchronized (state)
        {
            if (isActive())
            {
                state.expiredTimeInMsec.set(0);
                state.expired.set(false);
                state.canceled.set(true);
                event = state.notifyEvent;
                state.notifyEvent = null;
            }
            else
            {
                // Canceling a non-active timer is a no-op.
                event = null;
            }
        }
        removeTimer(this);

        if (doNotify && event != null)
        {
            // If there is a notification callback, it will be done automatically by the event.
            event.cancel();
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "timer=%s, doNotify=%s", this, doNotify);
        }
    }   //cancel

    /**
     * This method cancels the timer if it's set but has not expired. If the timer is canceled, the event is signaled.
     */
    public void cancel()
    {
        cancel(true);
    }   //cancel

    /**
     * This method returns the expired timestamp in milliseconds. If the timer is not set, it returns 0.
     *
     * @return expired timestamp in msec.
     */
    private long getExpiredTimeInMsec()
    {
        return state.expiredTimeInMsec.get();
    }   //getExpiredTimeInMsec

    /**
     * This method checks if the timer has expired.
     *
     * @return true if the timer has expired, false otherwise.
     */
    public boolean hasExpired()
    {
        return state.expired.get();
    }   //hasExpired

    /**
     * This method checks if the timer was canceled.
     *
     * @return true if the timer was canceled, false otherwise.
     */
    public boolean isCanceled()
    {
        return state.canceled.get();
    }   //isCanceled

    /**
     * This method checks if the timer is active (i.e. running, not expired and not canceled).
     *
     * @return true if the timer is active, false otherwise.
     */
    public boolean isActive()
    {
        boolean active;

        synchronized (state)
        {
            active = state.expiredTimeInMsec.get() > 0 && !state.expired.get() && !state.canceled.get();
        }

        return active;
    }   //isActive

    /**
     * This method is called when the timer has expired.
     */
    private void setExpired()
    {
        final String funcName = "setExpired";
        TrcEvent event;

        synchronized (state)
        {
            if (!state.canceled.get())
            {
                state.expiredTimeInMsec.set(0);
                state.expired.set(true);
                state.canceled.set(false);
                event = state.notifyEvent;
                state.notifyEvent = null;
            }
            else
            {
                // Timer was canceled, event would have been notified already.
                event = null;
            }
        }
        // If there is a notification callback, it will be done automatically by the event.
        if (event != null)
        {
            event.signal();
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "Timer %s has expired.", this);
        }
    }   //setExpired

    //
    // Timer Management: It is a singleton. Therefore, everything here are static.
    //

    private static final TrcHighPrecisionTime modeStartTime = new TrcHighPrecisionTime("ModeStartTime");
    private static final ArrayList<TrcTimer> timerList = new ArrayList<>();
    private static Thread timerThread = null;
    private static volatile boolean shuttingDown = false;
    private static volatile TrcTimer nextTimerToExpire = null;
    private static volatile TrcTimer preemptingTimer = null;

    /**
     * This method is called at the start of a competition mode to set the mode start timestamp so that
     * getModeElapsedTime can calculate the mode elapsed time.
     */
    public static void recordModeStartTime()
    {
        modeStartTime.recordTimestamp();
    }   //recordModeStartTime

    /**
     * This method returns the competition mode elapsed time by subtracting mode start time from the current time.
     * If this method is called before the competition mode is started, the system elapsed time is returned instead.
     *
     * @return mode elapsed time in seconds.
     */
    public static double getModeElapsedTime()
    {
        return modeStartTime.getElapsedTime();
    }   //getModeElapsedTime

    /**
     * This method returns the elapsed time in seconds of the specified epoch time from the competition mode start
     * epoch time.
     *
     * @param epochTime specifies the epoch time to compute the elapsed time from the mode start time.
     * @return elapsed time in seconds from last mode start time.
     */
    public static double getModeElapsedTime(double epochTime)
    {
        return modeStartTime.getElapsedTime(epochTime);
    }   //getModeElapsedTime

    /**
     * Returns the current time in seconds.  Note that while the unit of time of the return value is in seconds,
     * the precision is in nanoseconds but not necessarily nanosecond resolution (that is, how frequently the value
     * changes). There is no guarantee except that the resolution is at least as good as that of
     * System.currentTimeMillis().
     *
     * @return the time difference in seconds, between the current time and midnight, January 1, 1970 UTC with
     *         nanosecond precision.
     */
    public static double getCurrentTime()
    {
        return modeStartTime.getCurrentTime();
    }   //getCurrentTime

    /**
     * This method returns the nano second timestamp since a fixed arbitrary time referenced by the Java VM.
     * Note: this is not epoch time so it is not interchangeable with the time returned from getCurrentTime().
     *
     * @return current time in nano second.
     */
    public static long getNanoTime()
    {
        return System.nanoTime();
    }   //getNanoTime

    /**
     * This method returns the current epoch time in msec.
     *
     * @return current time in msec.
     */
    public static long getCurrentTimeMillis()
    {
        return System.currentTimeMillis();
    }   //getCurrentTimeMillis

    /**
     * This method returns the current time stamp with the specified format.
     *
     * @param format specifies the time stamp format.
     * @return current time stamp string with the specified format.
     */
    public static String getCurrentTimeString(String format)
    {
        SimpleDateFormat dateFormat = new SimpleDateFormat(format, Locale.US);
        return dateFormat.format(new Date());
    }   //getCurrentTimeString

    /**
     * This method returns the current time stamp with the default format.
     *
     * @return current time stamp string with the default format.
     */
    public static String getCurrentTimeString()
    {
        return getCurrentTimeString("yyyyMMdd@HHmmss");
    }   //getCurrentTimeString

    /**
     * This method puts the current thread to sleep for the given time in msec. It handles InterruptException where
     * it recalculates the remaining time and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param milliTime specifies sleep time in msec.
     */
    public static void sleep(long milliTime)
    {
        long wakeupTime = System.currentTimeMillis() + milliTime;

        while (milliTime > 0)
        {
            try
            {
                Thread.sleep(milliTime);
                break;
            }
            catch (InterruptedException e)
            {
                milliTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }   //sleep

    /**
     * This method adds the timer to the timer list in the order of expiration.
     *
     * @param timer specifies the timer to be added to the list.
     */
    private static void addTimer(TrcTimer timer)
    {
        final String funcName = "addTimer";
        long expiredTimeInMsec = timer.getExpiredTimeInMsec();

        synchronized (timerList)
        {
            if (nextTimerToExpire != null && expiredTimeInMsec < nextTimerToExpire.getExpiredTimeInMsec())
            {
                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "[%d] Adding timer %s preempting %s.",
                        TrcTimer.getCurrentTimeMillis(), timer, nextTimerToExpire);
                }
                //
                // The added new timer expires sooner than the one we are sleeping on. Let's interrupt its sleep and
                // process this one first.
                //
                preemptingTimer = timer;
                timerThread.interrupt();
            }
            else
            {
                // Default to end of list in case this timer is the last in the list or the only one in the list.
                int position = timerList.size();

                for (int i = 0; i < timerList.size(); i++)
                {
                    if (expiredTimeInMsec < timerList.get(i).getExpiredTimeInMsec())
                    {
                        position = i;
                        break;
                    }
                }

                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "[%d] Adding timer %s to queue position %d.",
                        TrcTimer.getCurrentTimeMillis(), timer, position);
                }
                timerList.add(position, timer);
                //
                // In case this is the first and only timer in the list, kick start the timer thread.
                //
                timerList.notify();
            }
        }

        if (timerThread == null)
        {
            // Timer thread does not exist, let's create one and start it.
            timerThread = new Thread(TrcTimer::timerTask, moduleName);
            timerThread.start();
        }
    }   //addTimer

    /**
     * This method removes a timer from the list.
     *
     * @param timer specifies the timer to be removed.
     */
    private static void removeTimer(TrcTimer timer)
    {
        final String funcName = "removeTimer";
        // Only do this if we are not shutting down. If we are shutting down, thread cleanup will take care of
        // the timer list.
        if (!shuttingDown)
        {
            synchronized (timerList)
            {
                if (timer == nextTimerToExpire)
                {
                    timerThread.interrupt();
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(funcName, "Removing current active timer %s", timer);
                    }
                }
                else
                {
                     boolean success = timerList.remove(timer);
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(funcName, "Removing timer %s in the list=%s", timer, success);
                    }
                }
            }
        }
        else if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "Shutdown in progress, not removing timer %s.", timer);
        }
    }   //removeTimer

    /**
     * This method is called by the TrcTaskMgr to shut down the timer thread when it is exiting.
     */
    public static void shutdown()
    {
        if (timerThread != null)
        {
            shuttingDown = true;
            timerThread.interrupt();
        }
    }   //shutdown

    /**
     * This method runs by the timer thread to wait for the next timer in the list and signal the timer object when
     * it expires.
     */
    private static void timerTask()
    {
        final String funcName = "timerTask";

        if (debugEnabled)
        {
            globalTracer.traceInfo("%s thread is starting...", moduleName);
        }

        TrcWatchdogMgr.Watchdog timerThreadWatchdog = TrcWatchdogMgr.registerWatchdog("TimerThread");
        while (!shuttingDown)
        {
            // Sending heartbeat will also unpause the watchdog if it was paused.
            timerThreadWatchdog.sendHeartBeat();

            try
            {
                long sleepTimeInMsec;

                synchronized (timerList)
                {
                    if (nextTimerToExpire == null && timerList.isEmpty())
                    {
                        if (debugEnabled)
                        {
                            globalTracer.traceInfo(funcName, "%s: waiting for timer ...", moduleName);
                        }
                        // Must take the timerList lock before calling wait because it will release the ownership
                        // and wait for notification.
                        // We need to pause the watchdog before we wait because we can't send heartbeat while waiting.
                        timerThreadWatchdog.pauseWatch();
                        timerList.wait();
                        timerThreadWatchdog.resumeWatch();

                        if (debugEnabled)
                        {
                            globalTracer.traceInfo(funcName, "%s: new timer arrived.", moduleName);
                        }
                    }

                    if (nextTimerToExpire == null)
                    {
                        if (!timerList.isEmpty())
                        {
                            if (debugEnabled)
                            {
                                globalTracer.traceInfo(
                                    funcName, "Get a timer from the queue (qSize=%d).", timerList.size());
                            }
                            // There is no timer in progress, get one from the timer queue.
                            nextTimerToExpire = timerList.remove(0);
                            if (nextTimerToExpire.isCanceled())
                            {
                                // This timer has been canceled, skip it.
                                nextTimerToExpire = null;
                            }
                        }
                        else
                        {
                            // There should be a timer in the queue. If not, somebody has canceled it and taken it
                            // out of the queue before we get to it. It's really not a problem but let's log a
                            // warning and move on.
                            globalTracer.traceWarn(funcName, "Expecting a timer from the queue but there is none.");
                        }
                    }
                }

                // sleepTimeInMsec is not zero only if we have a pending timer to process.
                sleepTimeInMsec = nextTimerToExpire != null?
                    nextTimerToExpire.getExpiredTimeInMsec() - TrcTimer.getCurrentTimeMillis(): 0;
                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "[%d]: timer=%s, sleepTimeInMsec=%d",
                        TrcTimer.getCurrentTimeMillis(), nextTimerToExpire, sleepTimeInMsec);
                }

                if (sleepTimeInMsec > 0)
                {
                    // We need to pause the watchdog before we go to sleep because we can't send heartbeat while
                    // sleeping.
                    timerThreadWatchdog.pauseWatch();
                    Thread.sleep(sleepTimeInMsec);
                    timerThreadWatchdog.resumeWatch();
                }

                if (nextTimerToExpire != null)
                {
                    // Timer has expired, signal it.
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%d]: timer %s expired.", TrcTimer.getCurrentTimeMillis(), nextTimerToExpire);
                    }
                    nextTimerToExpire.setExpired();
                    nextTimerToExpire = null;
                }
            }
            catch (InterruptedException e)
            {
                // There are three reasons we get interrupted:
                //  1. Somebody just added a preempting timer.
                //  2. Somebody just canceled and removed the current active timer.
                //  3. Somebody is shutting down the timer thread.
                synchronized (timerList)
                {
                    if (preemptingTimer != null)
                    {
                        // Somebody just added a preempting timer.
                        if (nextTimerToExpire != null)
                        {
                            // Somebody just added a timer that will expire sooner than the one we are sleeping on.
                            // Push this timer back to the front of the list and continue the next loop so the
                            // preempting timer will be processed first. If nextTimerToExpire is null, it means
                            // the current timer has been canceled, so don't put it back in the timer queue.
                            timerList.add(0, nextTimerToExpire);
                        }

                        if (debugEnabled)
                        {
                            globalTracer.traceInfo(
                                funcName, "[%d] Timer %s is preempting %s.",
                                TrcTimer.getCurrentTimeMillis(), preemptingTimer, nextTimerToExpire);
                        }

                        nextTimerToExpire = preemptingTimer;
                        preemptingTimer = null;
                    }
                    else if (nextTimerToExpire != null && nextTimerToExpire.isCanceled())
                    {
                        // Somebody just canceled and removed the current active timer.
                        if (debugEnabled)
                        {
                            globalTracer.traceInfo(
                                funcName, "[%d] timer %s was canceled.",
                                TrcTimer.getCurrentTimeMillis(), nextTimerToExpire);
                        }
                        nextTimerToExpire = null;
                    }
                    else if (shuttingDown)
                    {
                        // Somebody is shutting down the timer thread.
                        if (debugEnabled)
                        {
                            globalTracer.traceInfo(funcName, "Terminating %s thread.", moduleName);
                        }
                        break;
                    }
                }
            }
        }
        //
        // The thread is terminating, cancel all pending timers before exiting.
        //
        synchronized (timerList)
        {
            if (debugEnabled)
            {
                globalTracer.traceInfo(funcName, "Terminating: canceling %d timers.", timerList.size());
            }

            for (TrcTimer timer: timerList)
            {
                if (debugEnabled)
                {
                    globalTracer.traceInfo(funcName, "Canceling %s", timer);
                }
                timer.cancel();
            }

            timerList.clear();
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "%s thread is terminated.", moduleName);
        }
        //
        // The thread is now terminated. Destroy this instance so we will recreate the thread the next time around.
        //
        timerThreadWatchdog.unregister();
        timerThread = null;
        shuttingDown = false;
    }   //timerTask

}   //class TrcTimer
