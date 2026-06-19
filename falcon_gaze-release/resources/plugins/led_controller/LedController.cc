#include "LedController.hh"
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/VisualCmd.hh> 
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <algorithm>

using namespace led_controller;

void LedController::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
    this->modelEntity = _entity;

    std::string modelName = gz::sim::scopedName(this->modelEntity, _ecm, "/");
    size_t lastSlash = modelName.find_last_of('/');
    if (lastSlash != std::string::npos) {
        modelName = modelName.substr(lastSlash + 1);
    }

    this->ledVisualEntities.clear();
    std::vector<std::pair<std::string, gz::sim::Entity>> foundLenses;

    _ecm.Each<gz::sim::components::Visual, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_visEntity, const gz::sim::components::Visual *, const gz::sim::components::Name *_nameComp) -> bool
        {
            std::string name = _nameComp->Data();
            if (name.find("led_lens") != std::string::npos && gz::sim::topLevelModel(_visEntity, _ecm) == this->modelEntity)
            {
                foundLenses.push_back({name, _visEntity});
            }
            return true; 
        });

    std::sort(foundLenses.begin(), foundLenses.end());
    for (const auto &pair : foundLenses) {
        this->ledVisualEntities.push_back(pair.second);
    }

    if (this->ledVisualEntities.empty()) {
        gzerr << "LedController failed to find any visual entities named 'led_lens_*' for model " << modelName << "!" << std::endl;
        return;
    }

    std::string topic = "/model/" + modelName + "/led_cmd";
    if (!this->node.Subscribe(topic, &LedController::OnLedCmd, this)) {
        gzerr << "Error subscribing to topic [" << topic << "]" << std::endl;
        return;
    }

    gzmsg << "LedController loaded for " << modelName << " with binary string support." << std::endl;
}

void LedController::OnLedCmd(const gz::msgs::StringMsg &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    this->currentCommand = _msg.data();
    this->stateChanged = true;
}

void LedController::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    bool brandNewCommand = false;
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        brandNewCommand = this->stateChanged;
    }

    if (!brandNewCommand) {
        auto timeSinceLastCheck = _info.simTime - this->lastEvaluationSimTime;
        if (timeSinceLastCheck < std::chrono::milliseconds(10)) {
            return; 
        }
    }
    this->lastEvaluationSimTime = _info.simTime;

    std::string cmd;
    bool shouldUpdateLenses = false;
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (_info.iterations < 20 || this->ledVisualEntities.empty()) {
            return;
        }
        cmd = this->currentCommand;
        shouldUpdateLenses = this->stateChanged;
        this->stateChanged = false;
    }

    gz::math::Color greenColor(0.0f, 1.0f, 0.0f, 1.0f);
    gz::math::Color offColor(0.0f, 0.0f, 0.0f, 0.0f);

    // --- NEW BINARY MASK PARSER LOGIC ---
    if (cmd.length() == 4 && (cmd[0] == '1' || cmd[0] == '0')) 
    {
        // Parse individual states based on the binary mask string positions
        for (size_t i = 0; i < this->ledVisualEntities.size() && i < 4; ++i) 
        {
            gz::math::Color targetedColor = (cmd[i] == '1') ? greenColor : offColor;
            this->UpdateVisualState(this->ledVisualEntities[i], targetedColor, _ecm);
        }
    }
    // --- FALLBACK TO NATIVE STANDARD MACROS ("ON"/"OFF") ---
    else 
    {
        if (cmd == "BLINK") 
        {
            auto elapsed = _info.simTime - this->lastFlippedSimTime;
            if (elapsed >= this->blinkInterval) 
            {
                this->internalBlinkState = !this->internalBlinkState;
                this->lastFlippedSimTime = _info.simTime;
                shouldUpdateLenses = true;
            }
            cmd = this->internalBlinkState ? "ON" : "OFF";
        }

        if (shouldUpdateLenses || cmd == "ON" || cmd == "OFF") 
        {
            gz::math::Color targetedColor = (cmd == "ON") ? greenColor : offColor;
            for (const auto &visEntity : this->ledVisualEntities) {
                this->UpdateVisualState(visEntity, targetedColor, _ecm);
            }
        }
    }
}
 
void LedController::UpdateVisualState(gz::sim::Entity _visEntity, const gz::math::Color &_color, gz::sim::EntityComponentManager &_ecm)
{
    auto matComp = _ecm.Component<gz::sim::components::Material>(_visEntity);
    if (!matComp) {
        sdf::Material materialData;
        _ecm.CreateComponent(_visEntity, gz::sim::components::Material(materialData));
        matComp = _ecm.Component<gz::sim::components::Material>(_visEntity);
    }

    if (matComp) {
        matComp->Data().SetEmissive(_color);
        matComp->Data().SetDiffuse(_color);
        matComp->Data().SetAmbient(_color);
        matComp->Data().SetSpecular(_color);
        _ecm.SetChanged(_visEntity, gz::sim::components::Material::typeId, gz::sim::ComponentState::PeriodicChange);
    }

    auto cmdComp = _ecm.Component<gz::sim::components::VisualCmd>(_visEntity);
    if (!cmdComp) {
        gz::msgs::Visual msg;
        msg.set_name(gz::sim::scopedName(_visEntity, _ecm));
        auto *matMsg = msg.mutable_material();
        matMsg->mutable_diffuse()->set_r(_color.R()); matMsg->mutable_diffuse()->set_g(_color.G()); matMsg->mutable_diffuse()->set_b(_color.B()); matMsg->mutable_diffuse()->set_a(_color.A());
        matMsg->mutable_ambient()->set_r(_color.R()); matMsg->mutable_ambient()->set_g(_color.G()); matMsg->mutable_ambient()->set_b(_color.B()); matMsg->mutable_ambient()->set_a(_color.A());
        matMsg->mutable_specular()->set_r(_color.R()); matMsg->mutable_specular()->set_g(_color.G()); matMsg->mutable_specular()->set_b(_color.B()); matMsg->mutable_specular()->set_a(_color.A());
        matMsg->mutable_emissive()->set_r(_color.R()); matMsg->mutable_emissive()->set_g(_color.G()); matMsg->mutable_emissive()->set_b(_color.B()); matMsg->mutable_emissive()->set_a(_color.A());
        _ecm.CreateComponent(_visEntity, gz::sim::components::VisualCmd(msg));
    } 
    else {
        auto &msg = cmdComp->Data();
        auto *matMsg = msg.mutable_material();
        matMsg->mutable_diffuse()->set_r(_color.R()); matMsg->mutable_diffuse()->set_g(_color.G()); matMsg->mutable_diffuse()->set_b(_color.B()); matMsg->mutable_diffuse()->set_a(_color.A());
        matMsg->mutable_ambient()->set_r(_color.R()); matMsg->mutable_ambient()->set_g(_color.G()); matMsg->mutable_ambient()->set_b(_color.B()); matMsg->mutable_ambient()->set_a(_color.A());
        matMsg->mutable_specular()->set_r(_color.R()); matMsg->mutable_specular()->set_g(_color.G()); matMsg->mutable_specular()->set_b(_color.B()); matMsg->mutable_specular()->set_a(_color.A());
        matMsg->mutable_emissive()->set_r(_color.R()); matMsg->mutable_emissive()->set_g(_color.G()); matMsg->mutable_emissive()->set_b(_color.B()); matMsg->mutable_emissive()->set_a(_color.A());
        _ecm.SetChanged(_visEntity, gz::sim::components::VisualCmd::typeId, gz::sim::ComponentState::PeriodicChange);
    }
}

GZ_ADD_PLUGIN(led_controller::LedController, gz::sim::System, led_controller::LedController::ISystemConfigure, led_controller::LedController::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(led_controller::LedController, "led_controller::LedController")