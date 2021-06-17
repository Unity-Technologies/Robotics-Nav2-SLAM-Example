using UnityEngine;

namespace Unity.Robotics.Core
{
    public static class Clock
    {
        // TODO: Think about what this would mean in sim time context?  Probably ties into a sim scheduler...
        public static float DeltaTimeSeconds => Time.deltaTime;

        public static float TimeSeconds => Time.time;

        public static TimeStamp TimeStamp => new TimeStamp(TimeSeconds);
    }
}