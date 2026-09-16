#include "mode.h"
#include "Plane.h"

void ModeCruise::update()
{
    plane.update_cruise();
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = plane.nav_yaw_cd;
    return !plane.fbw_state.have_yaw_rate_input;
}
