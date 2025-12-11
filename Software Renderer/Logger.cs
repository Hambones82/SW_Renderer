using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    using System.Collections.Concurrent;
    using System.Text;

    public enum runningAvgLoggerType
    {
        wholeFrame,
        vertexProcessing,
        rasterizer
    }

    public struct RunningAverageMeasurementFloat
    {
        public int number = 0;
        public float value = 0;

        public RunningAverageMeasurementFloat()
        {
            number = 0;
            value = 0;
        }    
        
        public RunningAverageMeasurementFloat(RunningAverageMeasurementFloat copy)
        {
            number = copy.number;
            value = copy.value;
        }
    }

    public static class Logger
    {
        public const bool runningAvgEnabled = false;
        public const bool enabled = false;
        private static readonly ConcurrentDictionary<string, long> _counts
            = new ConcurrentDictionary<string, long>();
        private static readonly ConcurrentDictionary<runningAvgLoggerType, RunningAverageMeasurementFloat> _runningAverages
            = new ConcurrentDictionary<runningAvgLoggerType, RunningAverageMeasurementFloat>();

        static Logger()
        {
            //create an entry for all enums
            foreach(var t in Enum.GetValues<runningAvgLoggerType>()) //(typeof(runningAvgLoggerType)))
            {
                _runningAverages.TryAdd(t, new RunningAverageMeasurementFloat());
                
            }
        }

        public static void RecordMeasurement(runningAvgLoggerType typ, float measurement)
        {
            _runningAverages.TryGetValue(typ, out RunningAverageMeasurementFloat storedMeasurement);

            RunningAverageMeasurementFloat newMeasurement = new RunningAverageMeasurementFloat(storedMeasurement);
            float expandedValue = storedMeasurement.number * storedMeasurement.value;
            newMeasurement.number++;
            expandedValue += measurement;
            expandedValue /= newMeasurement.number;
            newMeasurement.value = expandedValue;
            _runningAverages.TryUpdate(typ, newMeasurement, storedMeasurement);
        }

        public static void ClearMeasurement(runningAvgLoggerType typ)
        {
            _runningAverages.TryGetValue(typ, out RunningAverageMeasurementFloat storedMeasurement);
            RunningAverageMeasurementFloat newMeasurement = new RunningAverageMeasurementFloat();

            
            _runningAverages.TryUpdate(typ, newMeasurement, storedMeasurement);
        }

        public static float GetMeasurement(runningAvgLoggerType typ)
        {
            _runningAverages.TryGetValue(typ, out RunningAverageMeasurementFloat storedMeasurement);            
            return storedMeasurement.value;
        }

        /// <summary>
        /// Increment count for a specific event key
        /// </summary>
        public static void Inc(string key, long value = 1)
        {
            _counts.AddOrUpdate(key, value, (_, v) => v + value);
        }

        /// <summary>
        /// Get the current count for a key (0 if missing)
        /// </summary>
        public static long Get(string key)
        {
            return _counts.TryGetValue(key, out long v) ? v : 0;
        }

        /// <summary>
        /// Dump all counts into a formatted string
        /// </summary>
        public static string Dump(bool clearAfter = false)
        {
            var sb = new StringBuilder();
            foreach (var kv in _counts)
                sb.AppendLine($"{kv.Key}: {kv.Value}");

            if (clearAfter)
                _counts.Clear();

            return sb.ToString();
        }

        /// <summary>
        /// Reset the counters
        /// </summary>
        public static void Clear() => _counts.Clear();
    }
}
