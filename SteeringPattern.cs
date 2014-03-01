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
        ROLLYAWFINAL,
        DIRECT
    }
    public enum WayPointType
    {
        DEFAULT,
        FINAL,
        INTERMEDIATE,
        DYNAMIC
    }

    public class WayPointQuaternion
    {

        private Quaternion rotation;
        private WayPointType waypointType;

        public float pitch = 0f;
        public float yaw = 0f;
        public float roll = 0f;

        private Vessel vessel;

        public WayPointQuaternion(float pitch, float yaw, float roll,Vessel vessel)
        {
            this.waypointType = WayPointType.DYNAMIC;
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.vessel = vessel;
        }

        public WayPointQuaternion(Quaternion rotation, WayPointType type)
        {
            this.rotation = rotation;
            this.waypointType = type;
        }

        public Quaternion getRotation()
        {
            //if dynamic, update rotation.
            if (this.GetWaypointType() == WayPointType.DYNAMIC)
            {
                this.rotation = kOS.Utilities.SteeringHelper.GetRotationFromNorth(pitch, yaw, roll, vessel);
            }
            return this.rotation;
        }

        public WayPointType GetWaypointType()
        {
            return this.waypointType;
        }
    }

    public class SteeringPattern
    {

        private List<WayPointQuaternion> wayPoints;

        private Quaternion qStart;
        private Quaternion qEnd;
        private SteeringPatternEnum pattern;

        private static float    interPolationSteps          = 0.0001f;
        private static float    angleThresholdDetection              = 0.01f;
        private static float angleThresholdIntermediate = 2.5f;
        private static float angleThresholdDefault = 1f;
        private static int angleRounding = 3;

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
            wayPoints = new List<WayPointQuaternion>();
            switch (pattern)
            {
                case SteeringPatternEnum.ROLLYAWFINAL:
                case SteeringPatternEnum.ROLLPITCHFINAL:
                    pathFindingRollFirst(start, end);
                    break;
                case SteeringPatternEnum.DIRECT:
                    wayPoints.Add(new WayPointQuaternion(start,WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(end, WayPointType.DEFAULT));
                    break;
            }

        }

        public List<Quaternion> findDistance(Quaternion rollRotation, float singleAxisRotation, Vector3 angleCompareTo, int intermediateSteps = 1)
        {
            List<Quaternion> intermediateQuaternions = new List<Quaternion>();
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
                    if (Math.Round(distanceYawPlus, angleRounding) <= angleThresholdDetection)
                    {
                        var rotationSteps = singleAxisRotation / intermediateSteps; 
                        for (var i = 1; i <= intermediateSteps; i++)
                        {
                            intermediateQuaternions.Add(rollRotation * Quaternion.Euler(0, 0, (rotationSteps * i)));
                        }



                        return intermediateQuaternions;
                    }
                }
                else
                {
                    if (Math.Round(distanceYawMinus, angleRounding) <= angleThresholdDetection)
                    {
                        var rotationSteps = singleAxisRotation / intermediateSteps;
                        for (var i = 1; i <= intermediateSteps; i++)
                        {
                            intermediateQuaternions.Add(rollRotation * Quaternion.Euler(0, 0, (-1 * (rotationSteps * i))));
                        }
                        return intermediateQuaternions;
                    }
                }
            }
            if (this.pattern == SteeringPatternEnum.ROLLPITCHFINAL)
            {
                if (distancePitchPlus < distancePitchMinus)
                {
                    if (Math.Round(distancePitchPlus, angleRounding) <= angleThresholdDetection)
                    {
                        var rotationSteps = singleAxisRotation / intermediateSteps;
                        for (var i = 1; i <= intermediateSteps; i++)
                        {
                            intermediateQuaternions.Add(rollRotation * Quaternion.Euler((rotationSteps * i), 0, 0));
                        }
                        return intermediateQuaternions;
                    }
                }
                else
                {
                    if (Math.Round(distancePitchMinus, angleRounding) <= angleThresholdDetection)
                    {
                        var rotationSteps = singleAxisRotation / intermediateSteps;
                        for (var i = 1; i <= intermediateSteps; i++)
                        {
                            intermediateQuaternions.Add(rollRotation * Quaternion.Euler((-1 * (rotationSteps * i)), 0, 0));
                        }
                        return intermediateQuaternions;
                    }
                }
            }

            return intermediateQuaternions;


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

            

       

            for (float f = 0; f < 1.0f; f += interPolationSteps)
            {

              

                //Quadrant One + Three
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, -90, 0), f);

                List<Quaternion> tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up,5);

                if (tempResult.Count > 0)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation,WayPointType.DEFAULT));
                    foreach (Quaternion q in tempResult)
                    {
                        wayPoints.Add(new WayPointQuaternion(q, WayPointType.INTERMEDIATE));
                    }
                    wayPoints.Add(new WayPointQuaternion(this.qEnd, WayPointType.DEFAULT));
                    break;
                }


                //Quadrant Two + Four
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, 90, 0), f);

                tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up,5);
                if (tempResult.Count >0)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation, WayPointType.DEFAULT));
                    foreach (Quaternion q in tempResult)
                    {
                        wayPoints.Add(new WayPointQuaternion(q, WayPointType.INTERMEDIATE));
                    }
                    wayPoints.Add(new WayPointQuaternion(this.qEnd, WayPointType.DEFAULT));
                    break;
                }



            }





        }


        public Quaternion returnNextWayPoint(Vessel vessel,bool moveToEndIfNotLast = false)
        {

            //update steeringpattern accordingly.
            //...
            //if we are not at the final waypoint yet.
            if (wayPoints.Count > 1)
            {
               


                var angle = Quaternion.Angle(wayPoints[0].getRotation(), vessel.transform.rotation);

    

                    
             

               

                if (Math.Round(angle, angleRounding) < (wayPoints[0].GetWaypointType() == WayPointType.INTERMEDIATE ? angleThresholdIntermediate : angleThresholdDefault))
                {
                    //reached
                    if (moveToEndIfNotLast)
                    {
                        WayPointQuaternion temp = wayPoints[0];
                        wayPoints.RemoveAt(0);
                        if (wayPoints.Count > 0)
                            wayPoints.Add(temp);

                    }
                    else
                    {
                        wayPoints.RemoveAt(0);
                        KSP.IO.File.AppendAllText<SteeringPattern>("removing,count now at " + ParameterSingleton.Instance.currentSteeringPattern.GetWayPoints().Count + " \n", "VesselTargetDebug.txt");
                    }
                }
                   
            }
            if (wayPoints.Count > 0)
            {
                return wayPoints[0].getRotation();
            }
                

            KSP.IO.File.AppendAllText<SteeringPattern>("returning identity " + ParameterSingleton.Instance.currentSteeringPattern.GetWayPoints().Count + " \n", "VesselTargetDebug.txt");
            return Quaternion.identity;

        }

        public void appendQuaternion(Quaternion q,WayPointType type, SteeringPatternEnum pattern = SteeringPatternEnum.DIRECT)
        {

            switch (pattern)
            {
                case SteeringPatternEnum.ROLLYAWFINAL:
                case SteeringPatternEnum.ROLLPITCHFINAL:
                    pathFindingRollFirst(wayPoints[wayPoints.Count-1].getRotation(), q);
                    break;
                case SteeringPatternEnum.DIRECT:
                    this.wayPoints.Add(new WayPointQuaternion(q, type));
                    break;
            }

            
        }

        public void appendQuaternion(float pitch, float yaw, float roll,Vessel vessel, SteeringPatternEnum pattern = SteeringPatternEnum.DIRECT)
        {

            switch (pattern)
            {
                case SteeringPatternEnum.ROLLYAWFINAL:
                case SteeringPatternEnum.ROLLPITCHFINAL:
                    WayPointQuaternion temp = new WayPointQuaternion(pitch, yaw, roll, vessel);
                    pathFindingRollFirst(wayPoints[wayPoints.Count - 1].getRotation(), temp.getRotation());
                    //this.wayPoints.Add(temp);
                    break;
                case SteeringPatternEnum.DIRECT:
                    this.wayPoints.Add(new WayPointQuaternion(pitch, yaw, roll, vessel));
                    break;
            }

            
        }

    }
}
