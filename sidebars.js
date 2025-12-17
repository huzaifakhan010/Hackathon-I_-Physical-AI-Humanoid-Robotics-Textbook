// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Physical AI & Humanoid Robotics modules sidebar
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 – Robotic Nervous System',
      items: [
        'module-1-ros2-arch/index',
        {
          type: 'category',
          label: 'Chapter 1: What is ROS 2 and Why Robots Need It',
          items: [
            'module-1-ros2-arch/chapter-1-what-is-ros2/index',
            'module-1-ros2-arch/chapter-1-what-is-ros2/middleware-analogies',
            'module-1-ros2-arch/chapter-1-what-is-ros2/architecture-overview',
            'module-1-ros2-arch/chapter-1-what-is-ros2/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Communication in ROS 2 (Nodes, Topics, Services)',
          items: [
            'module-1-ros2-arch/chapter-2-communication-patterns/index',
            'module-1-ros2-arch/chapter-2-communication-patterns/nodes-behavior',
            'module-1-ros2-arch/chapter-2-communication-patterns/topics-data-flow',
            'module-1-ros2-arch/chapter-2-communication-patterns/services-request-response',
            'module-1-ros2-arch/chapter-2-communication-patterns/message-flow-examples',
            'module-1-ros2-arch/chapter-2-communication-patterns/beginner-mistakes',
            'module-1-ros2-arch/chapter-2-communication-patterns/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Python Control and Robot Description',
          items: [
            'module-1-ros2-arch/chapter-3-python-control-urdf/index',
            'module-1-ros2-arch/chapter-3-python-control-urdf/python-rclpy-integration',
            'module-1-ros2-arch/chapter-3-python-control-urdf/ai-robot-bridging',
            'module-1-ros2-arch/chapter-3-python-control-urdf/urdf-introduction',
            'module-1-ros2-arch/chapter-3-python-control-urdf/joints-links-sensors',
            'module-1-ros2-arch/chapter-3-python-control-urdf/simulation-connection',
            'module-1-ros2-arch/chapter-3-python-control-urdf/exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-gazebo-unity/index',
        {
          type: 'category',
          label: 'Chapter 1: Digital Twin Fundamentals',
          items: [
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/index',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/digital-twin-concepts',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/simulation-overview',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/gazebo-unity-roles',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Physics & Environment Simulation with Gazebo',
          items: [
            'module-2-gazebo-unity/chapter-2-physics-simulation/index',
            'module-2-gazebo-unity/chapter-2-physics-simulation/physics-concepts',
            'module-2-gazebo-unity/chapter-2-physics-simulation/gravity-friction-collisions',
            'module-2-gazebo-unity/chapter-2-physics-simulation/environment-building',
            'module-2-gazebo-unity/chapter-2-physics-simulation/sensor-simulation',
            'module-2-gazebo-unity/chapter-2-physics-simulation/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Visual Realism & Human Interaction with Unity',
          items: [
            'module-2-gazebo-unity/chapter-3-visual-realism/index',
            'module-2-gazebo-unity/chapter-3-visual-realism/rendering-concepts',
            'module-2-gazebo-unity/chapter-3-visual-realism/human-robot-interaction',
            'module-2-gazebo-unity/chapter-3-visual-realism/ai-perception-sync',
            'module-2-gazebo-unity/chapter-3-visual-realism/exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac/index',
        {
          type: 'category',
          label: 'Chapter 1: AI Perception Fundamentals for Robots',
          items: [
            'module-3-isaac/chapter-1-perception-fundamentals/index',
            'module-3-isaac/chapter-1-perception-fundamentals/perception-concepts',
            'module-3-isaac/chapter-1-perception-fundamentals/sensor-types-overview',
            'module-3-isaac/chapter-1-perception-fundamentals/why-simulation-needed',
            'module-3-isaac/chapter-1-perception-fundamentals/real-world-analogies',
            'module-3-isaac/chapter-1-perception-fundamentals/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: NVIDIA Isaac Sim & Synthetic Data',
          items: [
            'module-3-isaac/chapter-2-isaac-sim/index',
            'module-3-isaac/chapter-2-isaac-sim/isaac-sim-concepts',
            'module-3-isaac/chapter-2-isaac-sim/synthetic-data-gen',
            'module-3-isaac/chapter-2-isaac-sim/safe-training-simulation',
            'module-3-isaac/chapter-2-isaac-sim/isaac-sim-diagrams',
            'module-3-isaac/chapter-2-isaac-sim/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Robot Navigation with Isaac ROS & Nav2',
          items: [
            'module-3-isaac/chapter-3-navigation/index',
            'module-3-isaac/chapter-3-navigation/vslam-concepts',
            'module-3-isaac/chapter-3-navigation/isaac-ros-pipelines',
            'module-3-isaac/chapter-3-navigation/nav2-path-planning',
            'module-3-isaac/chapter-3-navigation/isaac-ros-nav2-integration',
            'module-3-isaac/chapter-3-navigation/exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        {
          type: 'category',
          label: 'Chapter 1: Voice-to-Action with OpenAI Whisper',
          items: [
            'module-4-vla/chapter-1-voice-to-action/index',
            'module-4-vla/chapter-1-voice-to-action/voice-recognition-concepts',
            'module-4-vla/chapter-1-voice-to-action/whisper-integration',
            'module-4-vla/chapter-1-voice-to-action/voice-to-action-pipeline',
            'module-4-vla/chapter-1-voice-to-action/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Cognitive Planning and ROS 2 Action Sequencing',
          items: [
            'module-4-vla/chapter-2-cognitive-planning/index',
            'module-4-vla/chapter-2-cognitive-planning/llm-integration',
            'module-4-vla/chapter-2-cognitive-planning/action-sequencing',
            'module-4-vla/chapter-2-cognitive-planning/cognitive-planning-implementation',
            'module-4-vla/chapter-2-cognitive-planning/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone Project – Autonomous Humanoid Integration',
          items: [
            'module-4-vla/chapter-3-capstone/index',
            'module-4-vla/chapter-3-capstone/capstone-implementation',
            'module-4-vla/chapter-3-capstone/autonomous-behavior',
            'module-4-vla/chapter-3-capstone/exercises',
          ],
        },
      ],
    },
  ],

  // Manual sidebar for ROS 2 module
  ros2Sidebar: [
    {
      type: 'category',
      label: 'Module 1: Architecture and the Robotic Nervous System',
      items: [
        'module-1-ros2-arch/index',
        {
          type: 'category',
          label: 'Chapter 1: What is ROS 2 and Why Robots Need It',
          items: [
            'module-1-ros2-arch/chapter-1-what-is-ros2/index',
            'module-1-ros2-arch/chapter-1-what-is-ros2/middleware-analogies',
            'module-1-ros2-arch/chapter-1-what-is-ros2/architecture-overview',
            'module-1-ros2-arch/chapter-1-what-is-ros2/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Communication in ROS 2 (Nodes, Topics, Services)',
          items: [
            'module-1-ros2-arch/chapter-2-communication-patterns/index',
            'module-1-ros2-arch/chapter-2-communication-patterns/nodes-behavior',
            'module-1-ros2-arch/chapter-2-communication-patterns/topics-data-flow',
            'module-1-ros2-arch/chapter-2-communication-patterns/services-request-response',
            'module-1-ros2-arch/chapter-2-communication-patterns/message-flow-examples',
            'module-1-ros2-arch/chapter-2-communication-patterns/beginner-mistakes',
            'module-1-ros2-arch/chapter-2-communication-patterns/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Python Control and Robot Description',
          items: [
            'module-1-ros2-arch/chapter-3-python-control-urdf/index',
            'module-1-ros2-arch/chapter-3-python-control-urdf/python-rclpy-integration',
            'module-1-ros2-arch/chapter-3-python-control-urdf/ai-robot-bridging',
            'module-1-ros2-arch/chapter-3-python-control-urdf/urdf-introduction',
            'module-1-ros2-arch/chapter-3-python-control-urdf/joints-links-sensors',
            'module-1-ros2-arch/chapter-3-python-control-urdf/simulation-connection',
            'module-1-ros2-arch/chapter-3-python-control-urdf/exercises',
          ],
        },
      ],
    },
  ],

  // Manual sidebar for Digital Twin module
  digitalTwinSidebar: [
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-gazebo-unity/index',
        {
          type: 'category',
          label: 'Chapter 1: Digital Twin Fundamentals',
          items: [
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/index',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/digital-twin-concepts',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/simulation-overview',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/gazebo-unity-roles',
            'module-2-gazebo-unity/chapter-1-digital-twin-fundamentals/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Physics & Environment Simulation with Gazebo',
          items: [
            'module-2-gazebo-unity/chapter-2-physics-simulation/index',
            'module-2-gazebo-unity/chapter-2-physics-simulation/physics-concepts',
            'module-2-gazebo-unity/chapter-2-physics-simulation/gravity-friction-collisions',
            'module-2-gazebo-unity/chapter-2-physics-simulation/environment-building',
            'module-2-gazebo-unity/chapter-2-physics-simulation/sensor-simulation',
            'module-2-gazebo-unity/chapter-2-physics-simulation/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Visual Realism & Human Interaction with Unity',
          items: [
            'module-2-gazebo-unity/chapter-3-visual-realism/index',
            'module-2-gazebo-unity/chapter-3-visual-realism/rendering-concepts',
            'module-2-gazebo-unity/chapter-3-visual-realism/human-robot-interaction',
            'module-2-gazebo-unity/chapter-3-visual-realism/ai-perception-sync',
            'module-2-gazebo-unity/chapter-3-visual-realism/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Reference Materials',
          items: [
            'module-2-gazebo-unity/reference/gazebo-api-reference',
            'module-2-gazebo-unity/reference/unity-visualization-guide',
            'module-2-gazebo-unity/reference/simulation-best-practices',
          ],
        },
        {
          type: 'category',
          label: 'Tutorials',
          items: [
            'tutorials/digital-twin-examples/basic-gazebo-simulation',
            'tutorials/digital-twin-examples/unity-visualization-basics',
            'tutorials/digital-twin-examples/sensor-integration-workflow',
          ],
        },
      ],
    },
  ],

  // Manual sidebar for VLA module
  // Manual sidebar for Isaac module
  isaacSidebar: [
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac/index',
        {
          type: 'category',
          label: 'Chapter 1: AI Perception Fundamentals for Robots',
          items: [
            'module-3-isaac/chapter-1-perception-fundamentals/index',
            'module-3-isaac/chapter-1-perception-fundamentals/perception-concepts',
            'module-3-isaac/chapter-1-perception-fundamentals/sensor-types-overview',
            'module-3-isaac/chapter-1-perception-fundamentals/why-simulation-needed',
            'module-3-isaac/chapter-1-perception-fundamentals/real-world-analogies',
            'module-3-isaac/chapter-1-perception-fundamentals/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: NVIDIA Isaac Sim & Synthetic Data',
          items: [
            'module-3-isaac/chapter-2-isaac-sim/index',
            'module-3-isaac/chapter-2-isaac-sim/isaac-sim-concepts',
            'module-3-isaac/chapter-2-isaac-sim/synthetic-data-gen',
            'module-3-isaac/chapter-2-isaac-sim/safe-training-simulation',
            'module-3-isaac/chapter-2-isaac-sim/isaac-sim-diagrams',
            'module-3-isaac/chapter-2-isaac-sim/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Robot Navigation with Isaac ROS & Nav2',
          items: [
            'module-3-isaac/chapter-3-navigation/index',
            'module-3-isaac/chapter-3-navigation/vslam-concepts',
            'module-3-isaac/chapter-3-navigation/isaac-ros-pipelines',
            'module-3-isaac/chapter-3-navigation/nav2-path-planning',
            'module-3-isaac/chapter-3-navigation/isaac-ros-nav2-integration',
            'module-3-isaac/chapter-3-navigation/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Reference Materials',
          items: [
            'module-3-isaac/reference/isaac-sim-reference',
            'module-3-isaac/reference/isaac-ros-reference',
            'module-3-isaac/reference/nav2-humanoid-reference',
          ],
        },
        {
          type: 'category',
          label: 'Tutorials',
          items: [
            'module-3-isaac/tutorials/isaac-sim-basics/index',
            'module-3-isaac/tutorials/perception-workflows/index',
            'module-3-isaac/tutorials/navigation-humanoid/index',
          ],
        },
      ],
    },
  ],

  // Manual sidebar for VLA module
  vlaSidebar: [
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Integration',
      items: [
        'module-4-vla/index',
        {
          type: 'category',
          label: 'Chapter 1: Voice-to-Action with OpenAI Whisper',
          items: [
            'module-4-vla/chapter-1-voice-to-action/index',
            'module-4-vla/chapter-1-voice-to-action/voice-recognition-concepts',
            'module-4-vla/chapter-1-voice-to-action/whisper-integration',
            'module-4-vla/chapter-1-voice-to-action/voice-to-action-pipeline',
            'module-4-vla/chapter-1-voice-to-action/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Cognitive Planning and ROS 2 Action Sequencing',
          items: [
            'module-4-vla/chapter-2-cognitive-planning/index',
            'module-4-vla/chapter-2-cognitive-planning/llm-integration',
            'module-4-vla/chapter-2-cognitive-planning/action-sequencing',
            'module-4-vla/chapter-2-cognitive-planning/cognitive-planning-implementation',
            'module-4-vla/chapter-2-cognitive-planning/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone Project – Autonomous Humanoid Integration',
          items: [
            'module-4-vla/chapter-3-capstone/index',
            'module-4-vla/chapter-3-capstone/capstone-implementation',
            'module-4-vla/chapter-3-capstone/autonomous-behavior',
            'module-4-vla/chapter-3-capstone/exercises',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
