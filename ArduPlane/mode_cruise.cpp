#include "mode.h"
#include "Plane.h"

void ModeCruise::update()
{
    plane.update_cruise();
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
