#ifdef NDEBUG
#undef NDEBUG
#endif
#include "input/ProportionalGamepadMapper.h"
#include <cassert>
#include <cmath>
using namespace rb;
int main()
{
    ProportionalGamepadMapper m;
    GamepadInput i;
    i.connected = i.enabled = i.authority = true;
    i.depth = DepthEnvelopeState::Normal;
    i.leftY = 1;
    assert(!m.update(i, false, false).start);
    i.leftY = 0;
    m.update(i, false, false);
    i.rightX = 1;
    assert(!m.update(i, false, false).start);
    i.leftY = 1;
    auto o = m.update(i, false, false);
    assert(o.start && o.throttle == 1000 && o.turn == 1000);
    i.leftY = 0;
    i.rightX = 0;
    assert(!m.update(i, false, true).stop); // ACK wait ignores neutral
    assert(m.update(i, true, false).stop);
    i.leftY = 1;
    assert(!m.update(i, false, false).start);
    i.leftY = 0;
    i.rightX = 0;
    m.update(i, false, false);
    i.rightY = 1;
    i.depth = DepthEnvelopeState::Surface;
    assert(!m.update(i, false, false).start);
    i.depth = DepthEnvelopeState::Normal;
    assert(m.update(i, false, false).start);
    i.b = true;
    i.enabled = false;
    auto b = m.update(i, false, false);
    assert(b.stop && !b.disable);
    ProportionalGamepadMapper idle;
    GamepadInput idleInput;
    idleInput.enabled = idleInput.connected = idleInput.authority = true;
    idle.update(idleInput, false, false);
    idleInput.enabled = false;
    assert(!idle.update(idleInput, false, false).stop); // closing does not stop UI motion
    idleInput.enabled = true;
    idle.update(idleInput, false, false);
    idleInput.connected = false;
    assert(idle.update(idleInput, false, true).stop); // pending own Start still stops
    assert(std::abs(ProportionalGamepadMapper::shape(.575, .15, 1.5) - std::pow(.5, 1.5)) < 1e-10);
    assert(ProportionalGamepadMapper::shape(-1, .35, 3) == -1);
}
