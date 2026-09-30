#include "traindefinition/trainslist.h"
#include "traindefinition/train.h"
#include "traindefinition/locomotive.h"
#include <QCoreApplication>
#include <cmath>
#include <cstdio>

static int failures = 0;
static void check(bool ok, const char *what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++failures;
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (argc < 2)
    {
        std::printf("usage: notch_smoke <trainsFile>\n");
        return 2;
    }

    auto trains = TrainsList::ReadAndGenerateTrains(argv[1]);
    if (trains.empty())
    {
        std::printf("no trains loaded\n");
        return 2;
    }

    Train *t = trains.front().get();
    Locomotive *l = t->locomotives.front().get();
    const int N = l->Nmax;
    std::printf("locomotive: Nmax=%d maxLocNotch=%d isLocOn=%d\n",
                N, l->maxLocNotch, (int)l->isLocOn);

    bool optimize = false;
    double optimum = 1.0;
    const double speeds[] = {0.0, 2.5, 10.0, 22.0};
    double baseline[4];

    std::printf("\n-- default path (notch control OFF) --\n");
    check(!t->hasNotchControl(), "hasNotchControl() is false by default");
    for (int i = 0; i < 4; ++i)
    {
        double s = speeds[i];
        baseline[i] = l->getThrottleLevel(s, optimize, optimum);
        std::printf("   speed=%5.1f  lambda=%.9f\n", speeds[i], baseline[i]);
    }

    // default tractive force at standstill (pre-existing adhesion behaviour)
    double fc = 0.25, sp = 0.0, opt = 1.0;
    bool oz = false;
    double defaultStandstillF = l->getTractiveForce(fc, sp, oz, opt);

    std::printf("\n-- commanded notch = Nmax (%d) --\n", N);
    t->setNotch(N);
    check(t->hasNotchControl(), "hasNotchControl() is true after setNotch");
    check(t->getCurrentNotch() == N, "getCurrentNotch() == Nmax");
    const double expectedFull = std::pow((double)N / N, 2.0);
    for (int i = 0; i < 4; ++i)
    {
        double s = speeds[i];
        double v = l->getThrottleLevel(s, optimize, optimum);
        std::printf("   speed=%5.1f  lambda=%.9f  expected=%.9f\n",
                    speeds[i], v, expectedFull);
        char buf[64];
        std::snprintf(buf, sizeof buf, "notch=Nmax lambda at speed %.1f", speeds[i]);
        check(std::fabs(v - expectedFull) < 1e-12, buf);
    }

    std::printf("\n-- commanded notch = 0 (standstill must not roll) --\n");
    t->setNotch(0);
    check(t->getCurrentNotch() == 0, "getCurrentNotch() == 0");
    for (int i = 0; i < 4; ++i)
    {
        double s = speeds[i];
        double v = l->getThrottleLevel(s, optimize, optimum);
        std::printf("   speed=%5.1f  lambda=%.9f\n", speeds[i], v);
        check(std::fabs(v) < 1e-12, "notch=0 gives lambda=0");
    }
    sp = 0.0;
    double cmdStandstillF = l->getTractiveForce(fc, sp, oz, opt);
    std::printf("   standstill force: default=%.1f N  commanded-notch0=%.1f N\n",
                defaultStandstillF, cmdStandstillF);
    check(cmdStandstillF == 0.0, "notch 0 at standstill produces zero force");
    check(defaultStandstillF > 0.0,
          "uncontrolled standstill still uses adhesion limit (unchanged)");

    std::printf("\n-- commanded notch = N/2 --\n");
    int half = N / 2;
    t->setNotch(half);
    const double expectedHalf = std::pow((double)half / N, 2.0);
    check(t->getCurrentNotch() == half, "getCurrentNotch() == N/2");
    for (int i = 0; i < 4; ++i)
    {
        double s = speeds[i];
        double v = l->getThrottleLevel(s, optimize, optimum);
        std::printf("   speed=%5.1f  lambda=%.9f  expected=%.9f\n",
                    speeds[i], v, expectedHalf);
        char buf[64];
        std::snprintf(buf, sizeof buf, "notch=N/2 lambda at speed %.1f", speeds[i]);
        check(std::fabs(v - expectedHalf) < 1e-12, buf);
    }

    std::printf("\n-- out-of-range notches are clamped --\n");
    t->setNotch(N + 100);
    check(t->getCurrentNotch() == N, "setNotch(N+100) clamps to Nmax");
    t->setNotch(-5);
    check(t->getCurrentNotch() == 0, "setNotch(-5) clamps to 0");
    l->isLocOn = false;
    t->setNotch(N);
    double locoSpeed = 2.5;
    l->updateLocNotch(locoSpeed);
    check(l->currentLocNotch == 0, "powered-off loco reports notch 0");
    l->isLocOn = true;

    std::printf("\n-- restore: clearNotch() returns to the default path --\n");
    t->clearNotch();
    check(!t->hasNotchControl(), "hasNotchControl() is false after clearNotch");
    bool identical = true;
    for (int i = 0; i < 4; ++i)
    {
        double s = speeds[i];
        double v = l->getThrottleLevel(s, optimize, optimum);
        if (std::fabs(v - baseline[i]) > 0.0)
            identical = false;
        std::printf("   speed=%5.1f  lambda=%.9f  baseline=%.9f\n",
                    speeds[i], v, baseline[i]);
    }
    check(identical, "default trajectory is byte-identical after clearNotch()");

    std::printf("\n%s (%d failure(s))\n", failures ? "SMOKE TEST FAILED" : "SMOKE TEST PASSED", failures);
    return failures ? 1 : 0;
}
