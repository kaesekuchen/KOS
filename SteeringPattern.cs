using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using kOS.Utilities;
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
        DYNAMICNORTH,
        DYNAMICORBITPROGRADE,
        DIRECTINPUT
    }

    public class WayPointQuaternion
    {

        public Quaternion rotation;
        private WayPointType waypointType;



        public float pitch = 0f;
        public float yaw = 0f;
        public float roll = 0f;
         
        public float fore = 0f; //translation.Z;
        public float starboard = 0f; // translation.X;
        public float top = 0f; //translation.Y;

        public Vessel vessel;

        public SteeringPatternEnum steeringPattern;

        public bool waypointEvaluated = true;
        public bool waypointIgnore = false;

        public RotationApplyOrder rotationApplyOrder = RotationApplyOrder.YAWPITCHROLL;

        public WayPointQuaternion(float pitch, float yaw, float roll, float fore, float starboard, float top,Quaternion rotation, WayPointType wpType, SteeringPatternEnum steeringPattern, Vessel vessel,RotationApplyOrder RAO = RotationApplyOrder.YAWPITCHROLL)
        {
            //assign values
            this.pitch                  = pitch;
            this.yaw                    = yaw;
            this.roll                   = roll;
            this.vessel                 = vessel;
            this.fore                   = fore;
            this.starboard              = starboard;
            this.top                    = top;
            this.rotation               = rotation;
            this.waypointType           = wpType;
            this.steeringPattern        = steeringPattern;

            this.rotationApplyOrder     = RAO;

            //lazy eval ; we don't necessarily know the initilRoation at this point, so we will evaluate the waypoint once we want to reach it.
            this.waypointEvaluated = false;
            /*
            if (wpType == WayPointType.FINAL)
            {

            }
            else
            {
                //TODO: object parent hierarchy preferable to singleton
                ParameterSingleton.Instance.currentSteeringPattern.evaluateWaypoint(this);
            }
             */
        }

        public WayPointQuaternion(float pitch, float yaw, float roll,Vessel vessel)
        {
            this.waypointType = WayPointType.DYNAMICNORTH;
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.vessel = vessel;
        }

        public WayPointQuaternion(Quaternion rotation, WayPointType type,Vessel vessel = null)
        {
            this.rotation = rotation;
            this.waypointType = type;
            if (vessel != null)
                this.vessel = vessel;
        }

        public Quaternion getRotation()
        {
            //if dynamic, update rotation.

            if (vessel == null &&
                (this.GetWaypointType() == WayPointType.DYNAMICORBITPROGRADE || this.GetWaypointType() == WayPointType.DYNAMICNORTH)
                )
            {
                //no vesselReference attached to waypoint.
                throw new kOS.Debug.KOSException("no vesselReference attached to dynamic waypoint.");
            }

            switch (this.GetWaypointType())
            {
                case WayPointType.DYNAMICNORTH:
                    this.rotation = kOS.Utilities.SteeringHelper.GetRotationFromNorth(pitch, yaw, roll, vessel);
                    break;
                case WayPointType.DYNAMICORBITPROGRADE:
                    this.rotation = Quaternion.LookRotation(vessel.obt_velocity) * Quaternion.Euler(90, 0, 0);

                    break;
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
        //private SteeringPatternEnum pattern;

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
            //this.pattern = pattern;
            wayPoints = new List<WayPointQuaternion>();
            switch (pattern)
            {
                case SteeringPatternEnum.ROLLYAWFINAL:
                case SteeringPatternEnum.ROLLPITCHFINAL:
                    pathFindingRollFirst(start, end, pattern);
                    break;
                case SteeringPatternEnum.DIRECT:
                    wayPoints.Add(new WayPointQuaternion(start,WayPointType.DEFAULT));
                    wayPoints.Add(new WayPointQuaternion(end, WayPointType.DEFAULT));
                    break;
            }

        }

        public List<Quaternion> findDistance(Quaternion rollRotation, float singleAxisRotation, Vector3 angleCompareTo, SteeringPatternEnum steeringPattern, int intermediateSteps = 1)
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

            if (steeringPattern == SteeringPatternEnum.ROLLYAWFINAL)
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
            if (steeringPattern == SteeringPatternEnum.ROLLPITCHFINAL)
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

        public void pathFindingRollFirst(Quaternion q1, Quaternion q2, SteeringPatternEnum steeringPattern)
        {
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
                List<Quaternion> tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up, steeringPattern,5);
                if (tempResult.Count > 0)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation,WayPointType.DEFAULT));
                    foreach (Quaternion q in tempResult)
                    {
                        wayPoints.Add(new WayPointQuaternion(q, WayPointType.INTERMEDIATE));
                    }
                    wayPoints.Add(new WayPointQuaternion(q2, WayPointType.DEFAULT));
                    break;
                }
                //Quadrant Two + Four
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, 90, 0), f);
                tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up, steeringPattern, 5);
                if (tempResult.Count >0)
                {
                    wayPoints.Add(new WayPointQuaternion(rollRotation, WayPointType.DEFAULT));
                    foreach (Quaternion q in tempResult)
                    {
                        wayPoints.Add(new WayPointQuaternion(q, WayPointType.INTERMEDIATE));
                    }
                    wayPoints.Add(new WayPointQuaternion(q2, WayPointType.DEFAULT));
                    break;
                }
            }
        }

        public void evaluateWaypoint(WayPointQuaternion wpq)
        {
            //work the magic.
            KSP.IO.File.AppendAllText<SteeringPattern>("SteerShipViaSteeringPattern::Skip Quaternion:identity " + wpq.GetWaypointType()+ " \n", "VesselTargetDebug.txt");


            //find last evaluated waypoint
            WayPointQuaternion lastEvaluatedWaypoint = null;
            Quaternion startRotation = wpq.vessel.transform.rotation;
            List<WayPointQuaternion> wayPointsReversed = new List<WayPointQuaternion>();
            wayPointsReversed.AddRange(wayPoints);
            wayPointsReversed.Reverse();
            foreach (WayPointQuaternion waypointReverse in wayPointsReversed)
            {
                if (waypointReverse.waypointEvaluated && !waypointReverse.waypointIgnore)
                {
                    //found it
                    lastEvaluatedWaypoint = waypointReverse;
                    break;
                }
            }

            if (lastEvaluatedWaypoint != null)
            {
                startRotation = lastEvaluatedWaypoint.getRotation();
                // additionally, we now know, that we have an evaluated waypoint in the list, so we can move this one to the end of the queue
                wayPoints.Remove(wpq);
                wayPoints.Add(wpq);
                return;
            }
               
                



            if (wpq.steeringPattern == SteeringPatternEnum.ROLLPITCHFINAL || wpq.steeringPattern == SteeringPatternEnum.ROLLYAWFINAL)
            {
                if (wpq.GetWaypointType() == WayPointType.FINAL)
                {
                    
                    //use vessel.transform.rotation as start
                    pathFindingRollFirst(
                                         startRotation
                                        , kOS.Utilities.SteeringHelper.GetRotationFromTransform(wpq.pitch, wpq.yaw, wpq.roll, wpq.vessel.transform.rotation, wpq.rotationApplyOrder)
                                        , wpq.steeringPattern
                                        );
                }
                if (wpq.GetWaypointType() == WayPointType.DYNAMICNORTH || wpq.GetWaypointType() == WayPointType.DYNAMICORBITPROGRADE)
                {
                    pathFindingRollFirst(
                      startRotation
                     , wpq.getRotation()
                     , wpq.steeringPattern
                     );

                    //we have to waypointIgnore this one
                    //wpq.waypointIgnore = true;
                    //instead move it to the end of the queue.
                    wayPoints.Remove(wpq);
                    wayPoints.Add(wpq);
                }



                wpq.rotation = Quaternion.identity;
            }



            wpq.waypointEvaluated = true;

            //wayPoints.Remove(wpq);
        }

        public Quaternion returnNextWayPoint(Vessel vessel,bool moveToEndIfNotLast = false)
        {

            //update steeringpattern accordingly.
            //...
            //if we are not at the final waypoint yet.
            if (wayPoints.Count > 1)
            {

                //if the next waypoint isnt evaluated yet, we will move him to the end of the queue, to keep it lazy.
                /*
                if (!wayPoints[0].waypointEvaluated && wayPoints[wayPoints.Count-1].waypointEvaluated)
                {
                    WayPointQuaternion temp = wayPoints[0];
                    wayPoints.RemoveAt(0);
                    wayPoints.Add(temp);
                }
                */
                var angle = Quaternion.Angle(wayPoints[0].getRotation(), vessel.transform.rotation);
                // if the angle matches OR this is an evaluated waypoint with identity for rotation
                if (Math.Round(angle, angleRounding) < (wayPoints[0].GetWaypointType() == WayPointType.INTERMEDIATE ? angleThresholdIntermediate : angleThresholdDefault)
                    || (wayPoints[0].getRotation() == Quaternion.identity && wayPoints[0].waypointEvaluated)
                    || wayPoints[0].waypointIgnore)
                {
                    //reached
                    if (moveToEndIfNotLast)
                    {
                        WayPointQuaternion temp = wayPoints[0];
                        wayPoints.RemoveAt(0);
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
                WayPointQuaternion nextWaypoint = wayPoints[0];
                if (!nextWaypoint.waypointEvaluated)
                {
                    evaluateWaypoint(nextWaypoint);
                    //we should return here, as an unevaluated waypoint might be split up into multiple waypoints
                    //it should be fine to return the identity for the next steering action, since this function will get invoked again asap.
                    return Quaternion.identity;
                }

                return nextWaypoint.getRotation();
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
                    pathFindingRollFirst(wayPoints[wayPoints.Count - 1].getRotation(), q, pattern);
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
                    pathFindingRollFirst(wayPoints[wayPoints.Count - 1].getRotation(), temp.getRotation(), pattern);
                    this.wayPoints.Add(temp);
                    break;
                case SteeringPatternEnum.DIRECT:
                    this.wayPoints.Add(new WayPointQuaternion(pitch, yaw, roll, vessel));
                    break;
            }

            
        }

        public void appendWayPoint(WayPointQuaternion wpq)
        {

            this.wayPoints.Add(wpq);

        }

        public string GetControlInstructions(Quaternion q1, Quaternion q2, SteeringPatternEnum steeringPattern)
        {
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
                List<Quaternion> tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up, steeringPattern, 5);
                if (tempResult.Count > 0)
                {
                    result += "roll by " + (-90 * f) + " then pitch " + singleAxisRotation;
                    return result;
                }
                //Quadrant Two + Four
                rollRotation = Quaternion.Lerp(q1, q1 * Quaternion.Euler(0, 90, 0), f);
                tempResult = findDistance(rollRotation, singleAxisRotation, q2 * Vector3.up, steeringPattern, 5);
                if (tempResult.Count > 0)
                {
                    result += "roll by " + (-90 * f) + " then pitch " + singleAxisRotation;
                    return result;
                }
            }

            return result;
        }

    }
}
