#include "input/GamepadCaptureMapper.h"
#include <cstdio>
#include <cstdlib>
using namespace rb;
namespace {
int failures = 0;
void expect(bool ok, const char *name) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", name); ++failures; }
}
void pressFiresOnce() {
    GamepadCaptureMapper m;
    expect(!m.update(true, false, false).snapshot, "idle pad does nothing");
    expect(m.update(true, true, false).snapshot, "X press takes snapshot");
    expect(!m.update(true, true, false).snapshot, "held X does not repeat");
    expect(!m.update(true, false, false).snapshot, "X release does nothing");
    expect(m.update(true, true, false).snapshot, "second X press takes another snapshot");
}
void recordingToggle() {
    GamepadCaptureMapper m;
    m.update(true, false, false);
    expect(m.update(true, false, true).toggleRecording, "Y press toggles recording");
    expect(!m.update(true, false, true).toggleRecording, "held Y does not toggle again");
    m.update(true, false, false);
    expect(m.update(true, false, true).toggleRecording, "next Y press toggles again");
    const auto both = m.update(true, true, false);
    expect(both.snapshot && !both.toggleRecording, "buttons are independent");
}
void heldAcrossConnect() {
    GamepadCaptureMapper m;
    auto out = m.update(true, true, true);
    expect(!out.snapshot && !out.toggleRecording, "buttons held at first sample do not fire");
    m.update(true, false, false);
    expect(m.update(true, true, false).snapshot, "arms after release");
    m.update(false, false, false);
    out = m.update(true, true, true);
    expect(!out.snapshot && !out.toggleRecording, "buttons held across reconnect do not fire");
    expect(!m.update(false, true, true).snapshot, "disconnected pad never fires");
}
} // namespace
int main() {
    pressFiresOnce();
    recordingToggle();
    heldAcrossConnect();
    if (failures) return EXIT_FAILURE;
    std::puts("gamepad_capture_mapper_tests: PASS");
    return EXIT_SUCCESS;
}
