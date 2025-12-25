import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      link: {
        type: 'generated-index',
        title: 'Getting Started',
        description: 'Welcome to Physical AI & Humanoid Robotics',
        slug: '/intro',
      },
      items: ['intro/index', 'intro/prerequisites', 'intro/setup'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: {
        type: 'generated-index',
        title: 'The Robotic Nervous System',
        description: 'Learn ROS 2 middleware architecture for robot control',
        slug: '/module-1-ros2',
      },
      items: [
        'module-1-ros2/index',
        'module-1-ros2/nodes-and-topics',
        'module-1-ros2/services-and-actions',
        'module-1-ros2/urdf-basics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      link: {
        type: 'generated-index',
        title: 'The Digital Twin',
        description: 'Build physics simulations with Gazebo and Unity',
        slug: '/module-2-simulation',
      },
      items: [
        'module-2-simulation/index',
        'module-2-simulation/gazebo-basics',
        'module-2-simulation/unity-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'generated-index',
        title: 'The AI-Robot Brain',
        description: 'Leverage NVIDIA Isaac for photorealistic simulation',
        slug: '/module-3-isaac',
      },
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-sim-setup',
        'module-3-isaac/perception-pipelines',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      link: {
        type: 'generated-index',
        title: 'Vision-Language-Action',
        description: 'Integrate voice commands and LLMs for robot control',
        slug: '/module-4-vla',
      },
      items: [
        'module-4-vla/index',
        'module-4-vla/voice-interfaces',
        'module-4-vla/llm-planning',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'generated-index',
        title: 'Build Your Humanoid',
        description: 'Complete capstone project integrating all skills',
        slug: '/capstone',
      },
      items: [
        'capstone/index',
        'capstone/milestone-1',
        'capstone/milestone-2',
        'capstone/milestone-3',
        'capstone/milestone-4',
        'capstone/milestone-5',
      ],
    },
  ],
};

export default sidebars;
