#include "road_driving/engine_router.hpp"

#include "road_driving/lane_network.hpp"

#include "autonomous_navigator/NavigationCore.hpp"
#include "core/Graph.hpp"

#include <chrono>
#include <iostream>
#include <streambuf>

namespace road {

namespace {

class CountingNullBuf : public std::streambuf {
public:
    std::uint64_t bytes = 0;

protected:
    int_type overflow(int_type c) override {
        if (!traits_type::eq_int_type(c, traits_type::eof())) {
            ++bytes;
        }
        return traits_type::not_eof(c);
    }
    std::streamsize xsputn(const char*, std::streamsize n) override {
        bytes += static_cast<std::uint64_t>(n);
        return n;
    }
};

// Redirects std::cout for the duration of one engine call and accumulates the wall-clock time.
class EngineCall {
public:
    EngineCall(CountingNullBuf& sink, double& ms) : ms_(ms), old_(std::cout.rdbuf(&sink)), t0_(std::chrono::steady_clock::now()) {}
    ~EngineCall() {
        std::cout.rdbuf(old_);
        ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
    }
    EngineCall(const EngineCall&) = delete;
    EngineCall& operator=(const EngineCall&) = delete;

private:
    double& ms_;
    std::streambuf* old_;
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace

struct EngineRouter::Impl {
    CountingNullBuf sink;
    double ms = 0.0;
    Graph graph;
    std::unique_ptr<NavigationCore> nav;
    std::size_t index = 0;  // index of the current node in NavigationCore's route (which includes the travelled part)
};

EngineRouter::EngineRouter(const LaneNetwork& net, double unitsPerMetre) : impl_(new Impl) {
    EngineCall call(impl_->sink, impl_->ms);
    for (const Element& e : net.elements) {
        // A node sits at its element's end point, so every edge's straight-line length is at most the
        // arc length of the element it enters and the engine's rescaled A* heuristic stays admissible.
        const PathPoint& end = e.pts.back();
        impl_->graph.addNode(e.id, e.name, end.x * unitsPerMetre, end.y * unitsPerMetre);
    }
    for (const Element& e : net.elements) {
        for (int v : e.next) {
            impl_->graph.addEdge(e.id, v, net.cost(v), false);
        }
    }
    impl_->nav.reset(new NavigationCore(&impl_->graph));
}

EngineRouter::~EngineRouter() = default;

bool EngineRouter::start(int from, int goal) {
    EngineCall call(impl_->sink, impl_->ms);
    const bool ok = impl_->nav->startNavigation(from, goal);
    if (ok) {
        impl_->index = 0;
    }
    return ok;
}

bool EngineRouter::step() {
    EngineCall call(impl_->sink, impl_->ms);
    const bool ok = impl_->nav->executeNavigationStep();
    if (ok) {
        ++impl_->index;
    }
    return ok;
}

void EngineRouter::reportClosed(const std::vector<int>& elements) {
    EngineCall call(impl_->sink, impl_->ms);
    impl_->nav->handleObstacles(elements);  // keeps the travelled part, so the tracked index stays valid
}

void EngineRouter::clear(const std::vector<int>& elements) {
    EngineCall call(impl_->sink, impl_->ms);
    impl_->nav->clearObstacles(elements);
}

bool EngineRouter::retry() {
    EngineCall call(impl_->sink, impl_->ms);
    return impl_->nav->retryNavigation();
}

std::vector<int> EngineRouter::routeAhead() const {
    const NavigationState st = impl_->nav->getNavigationState();
    if (st != NavigationState::ACTIVE && st != NavigationState::COMPLETED && st != NavigationState::PAUSED) {
        return {};
    }
    const std::vector<int> path = impl_->nav->getCurrentPath();
    if (impl_->index >= path.size() || path[impl_->index] != impl_->nav->getCurrentPosition()) {
        return {};
    }
    return std::vector<int>(path.begin() + static_cast<std::ptrdiff_t>(impl_->index), path.end());
}

int EngineRouter::position() const { return impl_->nav->getCurrentPosition(); }

std::string EngineRouter::state() const { return navigationStateName(impl_->nav->getNavigationState()); }

bool EngineRouter::active() const { return impl_->nav->getNavigationState() == NavigationState::ACTIVE; }

bool EngineRouter::blocked() const { return impl_->nav->getNavigationState() == NavigationState::BLOCKED; }

bool EngineRouter::consistent() const {
    const std::vector<int> path = impl_->nav->getCurrentPath();
    return impl_->index < path.size() && path[impl_->index] == impl_->nav->getCurrentPosition();
}

double EngineRouter::engineMilliseconds() const { return impl_->ms; }

std::uint64_t EngineRouter::silencedBytes() const { return impl_->sink.bytes; }

std::size_t EngineRouter::nodeCount() const { return impl_->graph.getNodeCount(); }

std::size_t EngineRouter::edgeCount() const { return impl_->graph.getEdgeCount(); }

}  // namespace road
