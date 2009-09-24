#include "hwm_network.hpp"

namespace hwm
{
    bool road::check() const
    {
        return !id.empty() && rep.check();
    }
}
