using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using UnityEngine;

namespace kOS
{

    public enum SteeringPatternEnum
    {
        ROLLPITCHFINAL,
        ROLLYAWFINAL
    }
    public enum WayPointType
    {
        DEFAULT,
        FINAL,
        INTERMEDIATE
    }

    public class WayPointQuaternion
    {

        private Quaternion rotation;
        private WayPointType type;

        public WayPointQuaternion(Quaternion rotation, WayPointType type)
        {
            this.rotation = rotation;
            this.type = type;
        }

        public Quaternion getRotation()
        {
            return this.rotation;
        }

    }

    public class SteeringPattern
    {

        private List<WayPointQuaternion> wayPoints;

        private Quaternion qStart;
        private Quaternion qEnd;
        private SteeringPatternEnum pattern;

        private static float    interPolationSteps     = 0.0001f;
        private static float    angleThreshold         = 0.01f;
        private static int      angleRounding          = 3;

        private string debugVerbose = "";

        public List<WayPointQuaternion> GetWayPoints()
        {
            return this.wayPoints;
        }

        public SteeringPattern(Quaternion start, Quaternion end,SteeringPatternEnum pattern)
        {
            this.qStart = start;
            this.qEnd = end;
            this.pattern = pattern;

            switch (pattern)
            {
                case SteeringPatternEnum.ROLLYAWFINAL:
                case SteeringPatternEnum.ROLLPITCHFINAL:
                    pathFindingRollFirst(start, end);
                    break;
            }

        }

        public Quaternion findDistance(Quaternion rollRotation, float singleAxisRotation, Vector3 angleCompareTo)
        {

            Quaternion yawPlus;
            Quaternion yawMinus;
            Quaternion pitchPlus;
            Quaternion pitchMinus;
            float distanceYawPlus = float.PositiveInfinity;
            float distanceYawMinus = float.PositiveInfinity;
            float distancePitchPlus = float.PositiveInfinity;
            float distancePitchMinus = float.PositiveInfinity;    

            yawPlus = rollRotation * Quaternion.Euler(0, 0, singleAxisRotation);
            yawMinus = rollRotation * Quaternion.Euler(0, 0, (-1 * singleAxisRotation));

            pitchPlus = rollRotation * Quaternion.Euler(singleAxisRotation, 0, 0);
            pitchMinus = rollRotation * Quaternion.Euler((-1 * singleAxisRotation), 0, 0);

            distanceYawPlus = Vector3.Angle(yawPlus * Vector3.up, angleCompareTo);
            distanceYawMinus = Vector3.Angle(yawMinus * Vector3.up, angleCompareTo);

            distancePitchPlus = Vector3.Angle(pitchPlus * Vector3.up, angleCompareTo);
            distancePitchMinus = Vector3.Angle(pitchMinus * Vector3.up, angleCompareTo);
            /*
            debugVerbose +="findDistanceDebug_"
                + "distanceYawPlus=" + distanceYawPlus
                 + "distanceYawMinus=" + distanceYawMinus
                  + "distancePitchPlus=" + distancePitchPlus
                   + "distancePitchMinus=" + distancePitchMinus 
                
                + " \n";
            */

            if (this.pattern == SteeringPatternEnum.ROLLYAWFINAL)
            {
                if (distanceYawPlus < distanceYawMinus)
                {
                    if (Math.Round(distanceYawPlus, angleRounding) <= angleThreshold)
                    {
                        return yawPlus;
                    }
                }
                else
                {
                    if (Math.Round(distanceYawMinus, angleRounding) <= angleThreshold)
                    {
                        return yawMinus;
                    }
                }
            }
            if (this.pattern == SteeringPatternEnum.ROLLPITCHFINAL)
            {
                if (distancePitchPlus < distancePitchMinus)
                {
                    if (Math.Round(distancePitchPlus, angleRounding) <= angleThreshold)
                    {
                        return pitchPlus;
                    }
                }
                else
                {
                    if (Math.Round(distancePitchMinus, angleRounding) <= angleThreshold)
                    {
                        return pitchMinus;
                    }
                }
            }

            return Quaternion.identity;


        }

        public void pathFindingRollFirst(Quaternion q1, Quaternion q2)
        {

            //q1 = origin
            //q2 = destination

            //distance metric : 
            var distance = Vector3.Angle(q1 * Vector3.up, q2 * Vector3.up);
            var singleAxisRotation = distance;

            string result = "distance = " + distance + "\n";
            result += "singleAxisRotation = " + singleAxisRotation + "\n";

            Quaternion rollRotation;

            wayPoints = new List<WayPointQuaternion>();

       

            for (float f = 0; f < 1.0f; f += interPolationSteps)
            {

              

                //Quadrant One + Three
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, -90, 0), f);

                Quaternion tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up);

                if (tempResult != Quaternion.identity)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation,WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(tempResult,WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(this.qEnd, WayPointType.DEFAULT));
                    break;
                }


                //Quadrant Two + Four
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, 90, 0), f);

                tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up);
                if (tempResult != Quaternion.identity)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation, WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(tempResult, WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(this.qEnd, WayPointType.DEFAULT));
                    break;
                }



            }





        }


        public Quaternion returnNextWayPoint(Vessel vessel)
        {

            //update steeringpattern accordingly.
            //...
            //if we are not at the final waypoint yet.
            if (wayPoints.Count > 1)
            {
                var angle = Quaternion.Angle(wayPoints[0].getRotation(), vessel.transform.rotation);

                if (Math.Round(angle,angleRounding) < angleThreshold)
                    wayPoints.RemoveAt(0);
            }
            if (wayPoints.Count > 0)
                return wayPoints[0].getRotation();

            KSP.IO.File.AppendAllText<SteeringPattern>("returning identity " + ParameterSingleton.Instance.currentSteeringPattern.GetWayPoints().Count + " \n", "VesselTargetDebug.txt");
            return Quaternion.identity;

        }

    }
}
