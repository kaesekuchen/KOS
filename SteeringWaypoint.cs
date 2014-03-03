using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
namespace kOS
{
    public class SteeringWaypoint
    {
        private Vessel vessel;
        private Quaternion rotation;
        private SteeringCondition steeringCondition;

        public SteeringWaypoint(Vessel vessel, Quaternion rotation)
        {
            this.vessel = vessel;
            this.rotation = rotation;
            this.steeringCondition = new SteeringCondition(this);
        }

        public SteeringCondition getSteeringCondition()
        {
            return this.steeringCondition;
        }

        public Vessel getVessel()
        {
            return this.vessel;
        }
        public Quaternion getRotation()
        {
            return this.rotation;
        }
    }

    

}
