import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Humanoid Control',
      link: {
        type: 'generated-index',
        title: 'Module 1 Overview',
        description: 'An introduction to ROS 2 for humanoid robot control.',
        keywords: ['ros2', 'humanoid', 'robotics'],
      },
      items: [
        'module1-ros2-humanoid-control/chapter1',
        'module1-ros2-humanoid-control/chapter2',
        'module1-ros2-humanoid-control/chapter3',
        'module1-ros2-humanoid-control/chapter4',
        'module1-ros2-humanoid-control/chapter5',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'generated-index',
        title: 'Module 2 Overview',
        description: 'An introduction to digital twin simulation with Gazebo and Unity.',
        keywords: ['digital twin', 'simulation', 'gazebo', 'unity'],
      },
      items: [
        'module2-digital-twin/chapter1',
        'module2-digital-twin/chapter2',
        'module2-digital-twin/chapter3',
        'module2-digital-twin/chapter4',
        'module2-digital-twin/chapter5',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {
        type: 'generated-index',
        title: 'Module 3 Overview',
        description: 'Implement advanced perception, navigation, and AI-driven humanoid control using NVIDIA Isaac tools.',
        keywords: ['nvidia isaac', 'isaac sim', 'isaac ros', 'vslam', 'nav2', 'humanoid', 'robotics'],
      },
      items: [
        'module3-ai-robot-brain/chapter1',
        'module3-ai-robot-brain/chapter2',
        'module3-ai-robot-brain/chapter3',
        'module3-ai-robot-brain/chapter4',
        'module3-ai-robot-brain/chapter5',
      ],
    },

  ],
};

export default sidebars;
