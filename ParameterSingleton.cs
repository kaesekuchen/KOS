using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace kOS
{
    class ParameterSingleton
    {
        private static ParameterSingleton instance;

        private List<object> parameterStorage;

        private ParameterSingleton()
        {
            parameterStorage = new List<object>();
        }

        public List<object> getParameterStorage()
        {
            return parameterStorage;
        }

        public static ParameterSingleton Instance
        {

            get
            {
                if (instance == null)
                    instance = new ParameterSingleton();

                return instance;
            }

        }
    }
}
