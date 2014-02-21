using System.Collections.Generic;
using System.Text.RegularExpressions;
using kOS.Context;
using kOS.Utilities;

namespace kOS.Command.BasicIO
{
    [Command(@"^SETPARAM ( ?\((.*?)\))?$")]
    public class CommandSetParam : Command
    {
        public CommandSetParam(Match regexMatch, IExecutionContext context)
            : base(regexMatch, context)
        {
        }

        public override void Evaluate()
        {
            var parameters = new List<Expression.Expression>();

            if (RegexMatch.Groups.Count > 1)
            {
                var paramstring = RegexMatch.Groups[1].Value;
                if (paramstring.StartsWith("("))
                    paramstring = paramstring.Substring(1);
                if (paramstring.EndsWith(")"))
                    paramstring = paramstring.Substring(0, paramstring.Length - 1);

                string[] paramList = Utils.ProcessParams(paramstring);

                foreach (string par in paramList)
                {
                    parameters.Add(new Expression.Expression(par, ParentContext));
                }
            }

            ParameterSingleton.Instance.getParameterStorage().Clear();

            foreach (Expression.Expression exp in parameters)
            {
                ParameterSingleton.Instance.getParameterStorage().Add(exp.GetValue());
            }

            State = ExecutionState.DONE;
        }


    }
}
