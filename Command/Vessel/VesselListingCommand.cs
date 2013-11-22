﻿using System;
using System.Text.RegularExpressions;

namespace kOS
{
    [AttributeCommand(@"^LIST (PARTS|RESOURCES|ENGINES|TARGETS|BODIES|SENSORS)$")]
    class VesselListingCommand : Command
    {
        public VesselListingCommand(Match regexMatch, ExecutionContext context) : base(regexMatch, context) { }

        public override void Evaluate()
        {
            switch (RegexMatch.Groups[1].Value.ToUpper())
            {
                case "BODIES":
                    StdOut("");
                    StdOut("Name           Distance");
                    StdOut("-------------------------------------");
                    foreach (var body in FlightGlobals.fetch.bodies)
                    {
                        StdOut(body.bodyName.PadLeft(14) + " " + Vector3d.Distance(body.position, Vessel.GetWorldPos3D()));
                    }
                    StdOut("");

                    break;
                

                case "TARGETS":
                    StdOut("");
                    StdOut("Vessel Name              Distance");
                    StdOut("-------------------------------------");

                    double commRange = VesselUtils.GetCommRange(Vessel);

                    foreach (Vessel vessel in FlightGlobals.Vessels)
                    {
                        if (vessel != Vessel)
                        {
                            var vT = new VesselTarget(vessel, this);
                            if (vT.IsInRange(commRange))
                            {
                                StdOut(vT.target.vesselName.PadRight(24) + " " + vT.GetDistance().ToString("0.0").PadLeft(8));
                            }
                        }
                    }

                    StdOut("");

                    break;

                case "RESOURCES":
                    StdOut("");
                    StdOut("Stage      Resource Name               Amount");
                    StdOut("------------------------------------------------");

                    foreach (Part part in Vessel.Parts)
                    {
                        String stageStr = part.inverseStage.ToString();

                        foreach (PartResource resource in part.Resources)
                        {
                            StdOut(part.inverseStage.ToString() + " " + resource.resourceName.PadRight(20) + " " + resource.amount.ToString("0.00").PadLeft(8));
                        }
                    }
                    break;

                case "PARTS":
                    StdOut("------------------------------------------------");

                    foreach (Part part in Vessel.Parts)
                    {
                        StdOut(part.ConstructID + " " + part.partInfo.name);
                    }

                    break;

                case "ENGINES":
                    StdOut("------------------------------------------------");

                    foreach (Part part in VesselUtils.GetListOfActivatedEngines(Vessel))
                    {
                        foreach (PartModule module in part.Modules)
                        {
                            if (module is ModuleEngines)
                            {
                                var engineMod = (ModuleEngines)module;
                                
                                StdOut(part.uid + "  " + part.inverseStage.ToString() + " " + engineMod.moduleName);
                            }
                        }
                    }

                    break;

                case "SENSORS":
                    StdOut("");
                    StdOut("Part Name                             Sensor Type");
                    StdOut("------------------------------------------------");

                    foreach (Part part in Vessel.Parts)
                    {
                        foreach (PartModule module in part.Modules)
                        {
                            ModuleEnviroSensor sensor = module as ModuleEnviroSensor;
                            if (sensor != null)
                            {
                                if (part.partInfo.name.Length > 37)
                                    StdOut(part.partInfo.title.PadRight(34) + "... " + sensor.sensorType);
                                else
                                    StdOut(part.partInfo.title.PadRight(37) + " " + sensor.sensorType);
                            }
                        }
                    }

                    break;
            }

            State = ExecutionState.DONE;
        }
    }
}