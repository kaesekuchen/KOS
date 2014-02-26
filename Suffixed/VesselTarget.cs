using UnityEngine;
using kOS.Binding;
using kOS.Context;
using kOS.Utilities;

namespace kOS.Suffixed
{
    public class VesselTarget : SpecialValue
    {
        private readonly IExecutionContext context;

        static VesselTarget()
        {
            ShortCuttableShipSuffixes = new[]
                {
                    "HEADING", "PROGRADE", "RETROGRADE", "FACING", "MAXTHRUST", "VELOCITY", "GEOPOSITION", "LATITUDE",
                    "LONGITUDE",
                    "UP", "NORTH", "BODY", "ANGULARMOMENTUM", "ANGULARVEL", "MASS", "VERTICALSPEED", "SURFACESPEED",
                    "AIRSPEED", "VESSELNAME",
                    "ALTITUDE", "APOAPSIS", "PERIAPSIS", "SENSOR"
                };
        }

        public VesselTarget(Vessel target, IExecutionContext context)
        {
            this.context = context;
            Target = target;
        }

        public Vessel Target { get; private set; }
        public static string[] ShortCuttableShipSuffixes { get; private set; }

        public bool IsInRange(double range)
        {
            return GetDistance() <= range;
        }

        public double GetDistance()
        {
            return Vector3d.Distance(context.Vessel.GetWorldPos3D(), Target.GetWorldPos3D());
        }

        public override string ToString()
        {
            return "VESSEL(\"" + Target.vesselName + "\")";
        }


        public Quaternion GetSurfaceProgradeQ(bool inverse)
        {

            Vector3 vector = Target.srf_velocity;

            if (inverse)
                vector = vector * -1;

            Quaternion qPrograde = Quaternion.LookRotation(vector) * Quaternion.Euler(90, 0, 0);
            return qPrograde;
        }

        public Quaternion GetProgradeQ()
        {

           // orbitVelocity = new Vector(v.obt_velocity);
            //surfaceVelocity = new Vector(v.srf_velocity);

            Quaternion qPrograde = Quaternion.LookRotation(Target.obt_velocity) * Quaternion.Euler(90, 0, 0);
            return qPrograde;
        }
        public Quaternion GetRetrogradeQ()
        {
            Quaternion qRetrograde = Quaternion.LookRotation(Target.obt_velocity * -1) * Quaternion.Euler(90, 0, 0);
            return qRetrograde;
        }

        public Quaternion GetTargetOrbitDifferenceQ(bool inverse)
        {
            var vector = (context.Vessel.obt_velocity - Target.obt_velocity);

            if (inverse)
                vector = vector * -1;
            return Quaternion.LookRotation(vector)* Quaternion.Euler(90, 0, 0);

        }

        public Quaternion GetTargetDirectionQ(bool inverse)
        {

            var vector = (Target.GetWorldPos3D() - context.Vessel.GetWorldPos3D());

            if (inverse)
                vector = vector * -1;

            return Quaternion.LookRotation(vector) * Quaternion.Euler(90, 0, 0);

        }


        public string GetPositionRelativeToNorth()
        {


            var CoM = Target.findWorldCenterOfMass();

            var up = (CoM - Target.mainBody.position).normalized;

            var north = Vector3d.Exclude(up, (Target.mainBody.position + Target.mainBody.transform.up * (float)Target.mainBody.Radius) - CoM).normalized;


            Quaternion rotationSurface = Quaternion.LookRotation(north, up);

            Quaternion gimbal = Quaternion.Inverse(Quaternion.Euler(90, 0, 0) * Quaternion.Inverse(Target.GetTransform().rotation) * rotationSurface);

            return "P" + gimbal.eulerAngles.x.ToString("#.#") + "|" + gimbal.eulerAngles.y.ToString("#.#") + "|" + gimbal.eulerAngles.z.ToString("#.#");
        }


        public Direction GetPrograde()
        {
            var up = (Target.findLocalMOI(Target.findWorldCenterOfMass()) - Target.mainBody.position).normalized;

            var d = new Direction {Rotation = Quaternion.LookRotation(Target.orbit.GetVel().normalized, up)};
            return d;
        }

        public Direction GetRetrograde()
        {
            var up = (Target.findLocalMOI(Target.findWorldCenterOfMass()) - Target.mainBody.position).normalized;

            var d = new Direction {Rotation = Quaternion.LookRotation(Target.orbit.GetVel().normalized*-1, up)};
            return d;
        }

        public Direction GetFacing()
        {
            var facing = Target.transform.up;
            return new Direction(new Vector3d(facing.x, facing.y, facing.z).normalized, false);
        }

        private Quaternion GetRotationFromNorthByParameter()
        {

            float pitch = 0;
            float roll = 0;
            float yaw = 0;

            if (ParameterSingleton.Instance.getParameterStorage().Count >= 3)
            {
                pitch = System.Convert.ToSingle(ParameterSingleton.Instance.getParameterStorage()[0]);
                yaw = System.Convert.ToSingle(ParameterSingleton.Instance.getParameterStorage()[1]);
                roll = System.Convert.ToSingle(ParameterSingleton.Instance.getParameterStorage()[2]);
            }

            if (ParameterSingleton.Instance.getParameterStorage().Count >= 4)
                SteeringHelper.angleSASMinmum = System.Convert.ToSingle(ParameterSingleton.Instance.getParameterStorage()[3]);
            if (ParameterSingleton.Instance.getParameterStorage().Count >= 5)
                SteeringHelper.angleSASMaximum = System.Convert.ToSingle(ParameterSingleton.Instance.getParameterStorage()[4]);


            Quaternion result = SteeringHelper.GetRotationFromNorth(pitch, yaw, roll, Target);

            return result;

        }




        public override bool SetSuffix(string suffixName, object value)
        {
            switch (suffixName)
            {
                case "PACKDISTANCE":
                    var distance = (float) value;
                    Target.distanceLandedPackThreshold = distance;
                    Target.distancePackThreshold = distance;
                    return true;
            }

            return base.SetSuffix(suffixName, value);
        }

        public override object GetSuffix(string suffixName)
        {
            switch (suffixName)
            {
                case "CONTROL":
                    return FlightControlManager.GetControllerByVessel(Target);
                case "DIRECTION":
                    var vector = (Target.GetWorldPos3D() - context.Vessel.GetWorldPos3D());
                    return new Direction(vector, false);
                case "DISTANCE":
                    return (float) GetDistance();
                case "BEARING":
                    return VesselUtils.GetTargetBearing(context.Vessel, Target);
                case "HEADING":
                    return VesselUtils.GetTargetHeading(context.Vessel, Target);
                case "PROGRADE":
                    return GetPrograde();
                case "RETROGRADE":
                    return GetRetrograde();
                case "MAXTHRUST":
                    return VesselUtils.GetMaxThrust(Target);
                case "VELOCITY":
                    return new VesselVelocity(Target);
                case "GEOPOSITION":
                    return new GeoCoordinates(Target);
                case "LATITUDE":
                    return VesselUtils.GetVesselLattitude(Target);
                case "LONGITUDE":
                    return VesselUtils.GetVesselLongitude(Target);
                case "FACING":
                    return GetFacing();
                case "DIRECTIONQUATERNION":
                case "NORTHQ":
                    return GetRotationFromNorthByParameter();
                case "SURFACEPROGRADEQ":
                    return GetSurfaceProgradeQ(false);
                case "SURFACERETROGRADEQ":
                    return GetSurfaceProgradeQ(true);
                case "PROGRADEQ":
                    return GetProgradeQ();
                case "RETROGRADEQ":
                    return GetRetrogradeQ();
                case "TARGETPROGRADEQ":
                    return GetTargetOrbitDifferenceQ(false);
                case "TARGETRETROGRADEQ":
                    return GetTargetOrbitDifferenceQ(true);
                case "TARGETDIRECTIONQ":
                    return GetTargetDirectionQ(false);
                case "TARGETDIRECTIONINVERSEQ":
                    return GetTargetDirectionQ(true);
                case "POSITIONTONORTH":
                    return GetPositionRelativeToNorth();
                case "UP":
                    return new Direction(Target.upAxis, false);
                case "NORTH":
                    return new Direction(VesselUtils.GetNorthVector(Target), false);
                case "BODY":
                    return new BodyTarget(Target.mainBody, Target);
                case "ANGULARMOMENTUM":
                    return new Direction(Target.angularMomentum, true);
                case "ANGULARVEL":
                    return new Direction(Target.angularVelocity, true);
                case "MASS":
                    return Target.GetTotalMass();
                case "VERTICALSPEED":
                    return Target.verticalSpeed;
                case "SURFACESPEED":
                    return Target.horizontalSrfSpeed;
                case "AIRSPEED":
                    return
                        (Target.orbit.GetVel() - FlightGlobals.currentMainBody.getRFrmVel(Target.GetWorldPos3D()))
                            .magnitude; //the velocity of the vessel relative to the air);
                case "VESSELNAME":
                    return Target.vesselName;
                case "ALTITUDE":
                    return Target.altitude;
                case "APOAPSIS":
                    return Target.orbit.ApA;
                case "PERIAPSIS":
                    return Target.orbit.PeA;
                case "SENSORS":
                    return new VesselSensors(Target);
                case "TERMVELOCITY":
                    return VesselUtils.GetTerminalVelocity(Target);
                case "LOADED":
                    return Target.loaded;
                case "ORBIT":
                case "OBT":
                    return new OrbitInfo(Target.orbit, Target);
                case "ANGLETOHEADING":
                    return Quaternion.Angle(SteeringHelper.lastQHeading, Target.transform.rotation);
            }

            // Is this a resource?
            double dblValue;
            if (VesselUtils.TryGetResource(Target, suffixName, out dblValue))
            {
                return dblValue;
            }

            return base.GetSuffix(suffixName);
        }

     
  




        
    }
}