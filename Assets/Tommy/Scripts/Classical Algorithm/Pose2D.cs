using System;
using UnityEngine;

namespace Tommy.Scripts.Classical_Algorithm
{
    public struct Pose2D
    {
        public float x, y;
        public float heading;

        public Pose2D(float x, float y, float heading)
        {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        
        /// <summary>
        /// Remaps an angle in radians to the range of [-PI, PI]. Ex: Passing in 2 PI, returns 0.
        /// </summary>
        /// <param name="radian"> The angle in radians. </param>
        /// <returns> An equivalent angle between [-PI, PI] in radians. </returns>
        public static float AngleWrap(float radian)
        {
            const float TAU = 2 * Mathf.PI;
            float angle = radian % TAU;
            angle = (angle + TAU) % TAU;
            if (angle > Mathf.PI)
                angle -= TAU;
            return angle;
        }

        public float DistanceTo(Pose2D other)
        {
            float deltaX = other.x - x;
            float deltaY = other.y - y;
            float deltaHeading = AngleWrap(other.heading - heading);
            return Mathf.Sqrt(deltaX * deltaX + deltaY * deltaY + deltaHeading * deltaHeading);
        }

        /// <summary>
        /// Compares two poses. 2 poses are equivalent if they have the same rounded coordinate, and rounded degree.
        /// </summary>
        /// <param name="other"> The other Pose2D</param>
        /// <returns>true if both Pose2D have the same xy and rounded degree</returns>
        public override bool Equals(object other)
        {
            if (other is not Pose2D v) return false;
            
            bool samePose = Mathf.RoundToInt(x) == Mathf.RoundToInt(v.x) &&
                            Mathf.RoundToInt(y) == Mathf.RoundToInt(v.y);
            samePose &= Mathf.RoundToInt(AngleWrap(heading) * Mathf.Rad2Deg) ==
                        Mathf.RoundToInt(AngleWrap(v.heading) * Mathf.Rad2Deg);
            return samePose;
        }
        
        public override int GetHashCode()
        {
            return Mathf.RoundToInt(x).GetHashCode() ^ (Mathf.RoundToInt(y).GetHashCode() << 4) ^ (Mathf.RoundToInt(y).GetHashCode() >> 28) ^ (Mathf.RoundToInt(AngleWrap(heading) * Mathf.Rad2Deg).GetHashCode() >> 4) ^ (Mathf.RoundToInt(AngleWrap(heading) * Mathf.Rad2Deg).GetHashCode() << 28);
        }


        public override string ToString()
        {
            return $"({x}, {y}) heading: {heading * Mathf.Rad2Deg}";
        }
    }
}