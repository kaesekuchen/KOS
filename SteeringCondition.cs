using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace kOS
{

    public enum SteeringConditionStatementType
    {
        KOSSTATEMENT,
        ANGLE
    }

    public enum SteeringkOSStatementType
    {
        CONDITION,
        EXECUTION
    }

    public class SteeringkOSStatement
    {
        private object kosStatement;
        private SteeringkOSStatementType steeringkOSStatementType;

        public SteeringkOSStatement(object kosStatement,SteeringkOSStatementType steeringkOSStatementType)
        {
            this.kosStatement = kosStatement;
            this.steeringkOSStatementType = steeringkOSStatementType;
            this.validateStatement();
        }

        private void validateStatement()
        {
        }


        public bool Execute()
        {
            //execute
            switch (this.steeringkOSStatementType)
            {
                case SteeringkOSStatementType.CONDITION:
                    break;
                case SteeringkOSStatementType.EXECUTION:
                    break;

            }

            return false;
        }

    }

    public class SteeringConditionStatement
    {
        private SteeringConditionStatementType steeringConditionType;

        private float angleMatch = 0.0f;

        private SteeringkOSStatement kosStatement;

        public SteeringConditionStatement(float angleMatch)
        {
            this.steeringConditionType = SteeringConditionStatementType.ANGLE;
            this.angleMatch = angleMatch;
        }

        public SteeringConditionStatement(SteeringkOSStatement kosStatement)
        {
            //placeholder
            this.steeringConditionType = SteeringConditionStatementType.KOSSTATEMENT;
            this.kosStatement = kosStatement;
        }

        public bool evaluateStatement(SteeringWaypoint steeringWaypointParent)
        {
            switch (this.steeringConditionType)
            {
                case SteeringConditionStatementType.KOSSTATEMENT:
                    return evaluateStatementKOSStatement(steeringWaypointParent);
                case SteeringConditionStatementType.ANGLE:
                    return evaluateStatementAngle(steeringWaypointParent);
            }


            return false;
        }

        private bool evaluateStatementKOSStatement(SteeringWaypoint steeringWaypointParent)
        {
            //evaluate statement kos 
            return this.kosStatement.Execute();
        }
        private bool evaluateStatementAngle(SteeringWaypoint steeringWaypointParent)
        {
            //evaluate statement "angle"
            return Quaternion.Angle(steeringWaypointParent.getRotation(), steeringWaypointParent.getVessel().transform.rotation) < this.angleMatch;
        }

    }

    public class SteeringCondition
    {

        private List<SteeringConditionStatement> conditionStatements;
        private List<SteeringkOSStatement> executeStatements;
        private SteeringWaypoint waypointParent;

        public SteeringCondition(SteeringWaypoint waypointParent)
        {
            this.waypointParent = waypointParent;
            this.executeStatements = new List<SteeringkOSStatement>();
            this.conditionStatements = new List<SteeringConditionStatement>();
        }

        public void addConditionStatementAngle(float angle)
        {
            this.conditionStatements.Add(new SteeringConditionStatement(angle));
        }
        public void addConditionStatementKOS(object kosStatement)
        {
            this.conditionStatements.Add(new SteeringConditionStatement(new SteeringkOSStatement(kosStatement,SteeringkOSStatementType.CONDITION)));
        }
        public void addExecuteStatementKOS(object kosStatement)
        {
            this.executeStatements.Add(new SteeringkOSStatement(kosStatement,SteeringkOSStatementType.EXECUTION));
        }

        public bool evaluateCondition()
        {
            foreach (SteeringConditionStatement statement in this.conditionStatements)
            {
                if (!statement.evaluateStatement(this.waypointParent))
                    return false;
            }
            executeStatementsOnConditionMet();
            return true;
        }

        private void executeStatementsOnConditionMet()
        {
            foreach (SteeringkOSStatement executeStatement in this.executeStatements)
            {
                //execute 
                executeStatement.Execute();
            }
        }



    }
}
