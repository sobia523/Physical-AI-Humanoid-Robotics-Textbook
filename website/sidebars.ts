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

  ],
};

export default sidebars;
