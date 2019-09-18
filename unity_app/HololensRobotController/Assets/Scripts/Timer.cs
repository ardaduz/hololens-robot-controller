using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace HololensRobotController.Utilities
{
    public static class Timer
    {
        private static TimeSpan offsetUTC;
        private static Stopwatch stopwatch;
        private static bool initialized = false;

        public static void Init()
        {
            if(!initialized)
            {
                DateTime hololensStartTimeUTC = DateTime.UtcNow;
                stopwatch = Stopwatch.StartNew();

                DateTime beginningOfTime = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
                offsetUTC = hololensStartTimeUTC - beginningOfTime;
                initialized = true;
            }
        }

        public static TimeSpan SampleCurrentStopwatch()
        {
            return stopwatch.Elapsed;
        }

        public static TimeSpan GetOffsetUTC()
        {
            return offsetUTC;
        }

        public static double GetElapsedTimeInMilliseconds(TimeSpan timeSpan)
        {
            return timeSpan.TotalMilliseconds;
        }

        public static double GetElapsedTimeInSeconds(TimeSpan timeSpan)
        {
            return timeSpan.TotalSeconds;
        }

        public static int[] GetSecondsNanosecondsStructure(TimeSpan timeSpan)
        {
            double totalMilliseconds = timeSpan.TotalMilliseconds;
            double totalMillisecondsFloored = Math.Floor(totalMilliseconds);
            double fractionalMilliseconds = totalMilliseconds - totalMillisecondsFloored;

            int seconds = Convert.ToInt32(totalMilliseconds * 1e-3);
            int nanoseconds = Convert.ToInt32(fractionalMilliseconds * 1e6);

            int[] structure = new int[] { seconds, nanoseconds };

            return structure;
        }
    }
}

