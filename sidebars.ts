import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '📚 Fundamentals',
      items: [
        'chapters/physical-ai-intro/index',
        'chapters/embodied-intelligence/index',
      ],
    },
    {
      type: 'category',
      label: '🤖 ROS 2 Development',
      items: [
        'chapters/ros2-fundamentals/index',
        'chapters/ros2-communication/index',
      ],
    },
    {
      type: 'category',
      label: '🎮 Simulation',
      items: [
        'chapters/simulation-intro/index',
        'chapters/gazebo-basics/index',
        'chapters/isaac-sim/index',
      ],
    },
    {
      type: 'category',
      label: '🧠 Advanced AI',
      items: [
        'chapters/vla-models/index',
        'chapters/sim-to-real/index',
      ],
    },
    {
      type: 'category',
      label: '🛠️ Production & Integration',
      items: [
        'chapters/error-handling/index',
        'chapters/capstone/index',
      ],
    },
    {
      type: 'category',
      label: '📖 References',
      items: [
        'bibliography',
      ],
    },
  ],
};

export default sidebars;
