using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    using System.Collections.Concurrent;
    using System.Text;

    public static class EventCounterLog
    {
        public const bool enabled = false;
        private static readonly ConcurrentDictionary<string, long> _counts
            = new ConcurrentDictionary<string, long>();

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
