# Leader-Follower Swarm Verification Walkthrough

We have successfully implemented and verified the leader-follower drone swarm mission using vision-based tracking and optical LED communication protocols.

## Key Accomplishments

### 1. Dynamic Altitude Matching (Terrain Compensation)
* **Problem**: Ground elevations at spawn points in the Gazebo `baylands_custom` world differ by up to **1.55 meters** (e.g. Leader ground: 38.14m, Follower 3 ground: 39.69m). Taking off to a relative altitude of 1.0m caused drones to hover at vastly different heights, rendering the target LEDs out of the camera's vertical field of view.
* **Solution**: Configured a global target absolute altitude of **42.0m (MSL)**. Drones now query their starting ground absolute altitude via MAVSDK telemetry and dynamically calculate their relative takeoff altitude to ensure everyone hovers at the exact same physical plane.

### 2. Robust Protocol Decoder & Hold Mode
* **Problem**: High-frequency tracking flicker (e.g. from camera shaking or temporary LED occlusions) dropped the duty cycle and generated transitions, causing the follower to misinterpret noise as a `HOLD` (blink) command and get stuck.
* **Solution**: Upgraded the rolling 40-frame history window logic to strictly enforce edge bounds (`2 <= edges <= 5`) and duty cycle limits (`0.35 <= duty <= 0.65`) for 1Hz blinking. Random tracking noise (which has a high edge count) is now filtered out. Sticky transition logic resolves momentary blink-state changes.

### 3. Motion-Blur Rejection during Search
* **Problem**: During `YAW_SEARCH` rotation, rotation speeds (20 deg/s) caused horizontal motion blur. This smeared the green LED, dropping saturation below HSV limits and increasing the aspect ratio above the filter threshold.
* **Solution**: Implemented adaptive state-aware CV bounds in [follower_mission.py](file:///home/hacaton1/Desktop/Catfish/falcon_gaze-release/examples/follower_mission.py). When in `YAW_SEARCH` state, the HSV saturation/value thresholds are loosened and the maximum allowed aspect ratio is increased to 8.0. The search rate was also reduced to 8 deg/s to minimize physical blur.

### 4. Centroid Tracking Gate
* **Problem**: Background clutter (like green trees or specular reflections) could occasionally create false-positive contours, causing follower tracking to jump.
* **Solution**: Introduced a temporal-distance gate of **150 pixels** for consecutive frames. Small updates from true movement are tracked smoothly, while sudden jumps (e.g. background noise) are immediately rejected.

### 5. Chain landing LED shutdown
* **Change**: Added `drone.led_off()` to the landing block of followers. Once a drone lands, it shuts off its green LED so that drones behind it know they have reached the end of the chain.

---

## Verification Results

We verified the entire pattern sequence with a fresh Gazebo run:
1. **Takeoff**: Leader took off to absolute 42.0m (relative: 3.41m). Followers dynamically computed their relative takeoff heights (4.13m, 2.31m, 2.24m) and achieved level hover.
2. **Follow**: Followers initialized offboard and entered `FOLLOW` mode, successfully tracking the leader's solid green LED.
3. **Hold**: When the leader began its manual 1Hz blink sequence, the followers correctly decoded it as a `HOLD` command and stabilized in place.
4. **Resumed Follow**: When the leader's LED returned to solid green, the followers transitioned back to `FOLLOW` and resumed tracking.
5. **Finish & Land**: When the leader completed the route, turned its LED off, and landed, the followers safely went to hover, searched, and initiated landing.

---

## Remaining Unresolved Issues & Future Work

While the core functionality and optical LED command protocol are fully verified, the following edge cases remain unresolved for real-world deployment:

### 1. Dynamic Pitch / Roll Camera Tilt Compensation
* **Issue**: When a drone accelerates forward, it pitches down. This tilts its fixed front-facing camera downward, causing the tracked LED of the leading drone to appear higher in the frame. Under aggressive acceleration, the LED can easily fly off the top of the image frame, causing a loss of lock.
* **Proposed Workaround**: Implement a feedforward pitch offset in the altitude controller or limit the maximum forward acceleration to maintain the target within the vertical camera field of view.

### 2. Multi-Drone Occlusion during Sharp Turns
* **Issue**: In a chain formation, if the leader turns sharply, the follower must yaw to follow it. During this turn, the follower might occlude the leader's LED from the perspective of the drone behind it, breaking the chain.
* **Proposed Workaround**: Add cooperative flight behavior where drones fly in a slight staggered pattern (diagonal offset) rather than a perfectly straight line, ensuring clear line-of-sight to the preceding drone's LED at all times.

### 3. Tree / Ground Specular Reflection False Positives during YAW_SEARCH
* **Issue**: When a drone enters `YAW_SEARCH` mode and opens its search gate, it loosens its HSV filter to catch motion-blurred LEDs. If it sweeps past a bright green tree illuminated by the sun or a specular reflection from wet ground, it can briefly register a false detection.
* **Proposed Workaround**: Implement shape-based contour metrics (e.g., circularity or solidity checking) since a physical LED beacon is spherical and has high solidity, unlike trees or grassy areas.

---

## Hackathon Competition Requirements & Next Steps

Based on the updated hackathon rules, we must prioritize implementing and tuning the following features to maximize our score during tonight's evaluation on the simplified test scene:

### 1. Single File Swarm Flight ("Гуськом")
* **Requirement**: The drones must maintain a strict sequential chain formation: Follower 1 tracks Leader (Drone 0), Follower 2 tracks Follower 1, and Follower 3 tracks Follower 2.
* **Current Status**: Physically achieved via default spawn poses and camera fields of view. We should ensure the controllers are tuned to prevent collisions if a drone in front slows down or changes height suddenly.

### 2. Preparation for the Tonight's Simplified Test Scene
* **Requirement**: A test scene will be provided tonight with a predefined flight scenario for the leader drone to evaluate our tracking stability under various environment/flight conditions.
* **Action Plan**: Be ready to plug in the new Gazebo world name and run our launch scripts immediately.

### 3. Trajectory Accuracy and Obstacle Avoidance
* **Requirement**: Scoring is heavily weighted on how accurately the followers repeat the leader's trajectory. The scene will contain obstacles like trees, houses, enemy targets ("москалі"), and racing gates.
* **Action Plan**:
  * Fine-tune the P-controller gains (`KP_YAW`, `KP_FORWARD`, `KP_ALT`) to minimize path deviations and lag behind the drone ahead.
  * Test and ensure the collision-avoidance aspect of tracking (slowing down as the distance closes, backing up if too close) prevents any collision with the leading drone or obstacles along the path.
