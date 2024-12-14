#pragma once

#include "devils/devils.h"

namespace devils
{
    class WackerSystem
    {
    public:
        WackerSystem(ADIPneumatic &pneumatic)
            : pneumatic(pneumatic)
        {
        }

        void setExtended(bool extended)
        {
            pneumatic.setExtended(extended);
        }

    private:
        ADIPneumatic &pneumatic;
    };
}