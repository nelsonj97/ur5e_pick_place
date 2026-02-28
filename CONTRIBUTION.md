# Contributing to Vision-Based Pick-and-Place System

Thank you for your interest in contributing to this project! This repository was originally developed as a thesis project (Studienarbeit) by **Nelson Jorvany**([@nelsonj97](https://github.com/nelsonj97)) and is open to community contributions under the MIT License.

---

## 👤 Author

**Nelson Jorvany** (Chouobou Tsafack VI Nelson Jorvany)

- GitHub: [@nelsonj97](https://github.com/nelsonj97)
- Email: nelsonjorvany@gmail.com

---

## 🤝 How to Contribute

### Reporting Bugs

1. Go to [Issues](https://github.com/nelsonj97/ur5e_pick_place/issues)
2. Click "New Issue"
3. Select "Bug Report"
4. Fill in the template:
   - Description of the bug
   - Steps to reproduce
   - Expected vs actual behavior
   - System information (OS, ROS2 version, etc.)

### Suggesting Features

1. Go to [Issues](https://github.com/nelsonj97/ur5e_pick_place/issues)
2. Click "New Issue"
3. Select "Feature Request"
4. Describe:
   - What feature you want
   - Why it would be useful
   - How it could be implemented

### Submitting Code

#### Step 1: Fork the Repository

\`\`\`bash

# Click "Fork" on GitHub, then:

git clone https://github.com/[your-username]/ur5e_pick_place.git
cd ur5e_pick_place
\`\`\`

#### Step 2: Create a Branch

\`\`\`bash

# Use descriptive branch names

git checkout -b feature/multi-object-detection
git checkout -b fix/ik-timeout-issue
git checkout -b docs/improve-setup-guide
\`\`\`

#### Step 3: Make Changes

\`\`\`bash

# Follow coding standards (see below)

# Test your changes

# Update documentation if needed

\`\`\`

#### Step 4: Commit

\`\`\`bash

# Use clear commit messages

git add .
git commit -m "feat: add multi-object detection support"
git commit -m "fix: resolve IK timeout in simulation"
git commit -m "docs: update installation instructions"
\`\`\`

#### Step 5: Push and Pull Request

\`\`\`bash
git push origin feature/multi-object-detection
\`\`\`
Then open a Pull Request on GitHub.

---

## 📋 Coding Standards

### Python Style

\`\`\`python

# Follow PEP 8

# Use descriptive variable names

# Add docstrings to functions

def detect_white_ball(self, image):
    """
    Detect white ball using HSV color segmentation.

    Args:
        image: BGR image from camera (numpy array)

    Returns:
        Tuple (x, y, radius) or None if not detected
    """
    pass\`\`\`

### ROS2 Conventions

\`\`\`python

# Use ROS2 logging (not print)

self.get_logger().info("✅ Ball detected")
self.get_logger().warn("⚠️ Retrying detection")
self.get_logger().error("❌ IK failed")

# Use descriptive topic names

'/overhead/depth_sensor/image_raw'
'/joint_trajectory_controller/follow_joint_trajectory'
\`\`\`

### Commit Message Format

\`\`\`
type: short description (max 50 chars)

Types:
feat:  New feature
fix:   Bug fix
docs:  Documentation
test:  Tests
refactor: Code restructure
perf:  Performance improvement
\`\`\`

---

## 🧪 Testing

### Before Submitting

\`\`\`bash

# 1. Build successfully

colcon build --symlink-install

# 2. Launch simulation

ros2 launch ur5e_golf_pick_place golf_pick_place.launch.py

# 3. Run system

ros2 run ur5e_golf_pick_place vision_based_pick_and_place

# 4. Verify:

# ✅ No build errors

# ✅ System launches

# ✅ Detection works

# ✅ Pick-place completes

\`\`\`

---

## 📁 Project Structure

\`\`\`
ur5e_pick_place/
├── ur5e_golf_pick_place/
│   ├── pick_place_exercise/
│   │   └── vision_based_pick_and_place.py  ← Main code
├── launch/
│   └── golf_pick_place.launch.py           ← Launch file
├── config/
│   ├── ur5e_controllers.yaml               ← Controllers
│   └── moveit_config.yaml                  ← MoveIt config
├── worlds/
│   └── golf_pick_place.world               ← Gazebo world
├── urdf/
│   └── ur5e_robotiq85.urdf.xacro          ← Robot model
├── docs/
│   └── thesis.pdf                          ← Full thesis
├── LICENSE                                 ← MIT License
├── CITATION.cff                            ← Citation info
├── CONTRIBUTING.md                         ← This file
└── README.md                               ← Project overview
\`\`\`

---

## 🌟 Areas for Contribution

### Beginner Friendly

- Improve documentation
- Add code comments
- Fix typos
- Add examples

### Intermediate

- Add unit tests
- Improve error messages
- Add new object detection colors
- Improve visualization

### Advanced

- Implement deep learning detection (YOLO)
- Add multi-object support
- Real hardware deployment
- Trajectory optimization

---

## 📜 License

By contributing, you agree that your contributions
will be licensed under the
[MIT License](LICENSE).

---

## 📧 Contact

For questions, reach out to:

- **Nelson Jorvany**
- Email: nelsonjorvany@gmail.com
- GitHub: [@nelsonj97](https://github.com/nelsonj97)

---

*Thank you for helping improve this project!* 🤖✨
