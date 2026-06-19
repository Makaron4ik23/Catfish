#pragma once

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

namespace led_controller
{

class LedController :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    LedController() = default;
    ~LedController() override = default;

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:
    void OnLedCmd(const gz::msgs::StringMsg &_msg);
    void UpdateVisualState(gz::sim::Entity _visEntity, const gz::math::Color &_color, gz::sim::EntityComponentManager &_ecm);

    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    std::vector<gz::sim::Entity> ledVisualEntities;

    gz::transport::Node node;
    
    std::mutex mutex;
    std::string currentCommand{"ON"};
    bool stateChanged{false};

    // --- NEW TIMING TRACKING VARIABLES ---
    std::chrono::steady_clock::duration lastFlippedSimTime{std::chrono::steady_clock::duration::zero()};
    std::chrono::steady_clock::duration blinkInterval{std::chrono::milliseconds(50)}; // Default 0.05s (50ms)
    bool internalBlinkState{false};
    std::chrono::steady_clock::duration lastEvaluationSimTime{std::chrono::steady_clock::duration::zero()};
};

} // namespace led_controller