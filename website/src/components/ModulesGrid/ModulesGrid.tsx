import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './ModulesGrid.module.css'; // Assuming a CSS module for styling

// Define a type for module data for better type-checking
interface ModuleItem {
  id: string;
  title: string;
  subtitle: string;
  icon: string; // This would typically be a Lucide-react component name or path
  tags: string[];
  link: string;
}

// Placeholder module data (as per T045)
const modules: ModuleItem[] = [
  {
    id: 'ros2',
    title: 'Module 1: ROS 2',
    subtitle: 'The Robotic Nervous System',
    icon: 'Brain', // Placeholder icon name
    tags: ['ROS 2', 'Robotics', 'Middleware'],
    link: '/docs/module1-ros2-humanoid-control/chapter1',
  },
  {
    id: 'digital-twin',
    title: 'Module 2: Digital Twin',
    subtitle: 'Simulation & Virtual Environments',
    icon: 'Feather', // Placeholder icon name
    tags: ['Gazebo', 'Unity', 'Simulation'],
    link: '/',
  },
  {
    id: 'ai-robot-brain',
    title: 'Module 3: AI-Robot Brain',
    subtitle: 'Perception & Intelligence',
    icon: 'Zap', // Placeholder icon name
    tags: ['NVIDIA Isaac', 'AI', 'Perception'],
    link: '/',
  },
  {
    id: 'vla',
    title: 'Module 4: VLA',
    subtitle: 'Vision-Language-Action Models',
    icon: 'Speech', // Placeholder icon name
    tags: ['LLMs', 'VLM', 'Humanoids'],
    link: '/',
  },
];


function ModulesGrid(): JSX.Element {
  return (
    <section className={clsx('modules-section', styles.modulesSection)}>
      <div className={clsx('container', styles.modulesContainer)}>
        <h2 className={styles.sectionTitle}>Explore the Modules</h2>
        <div className={styles.grid}>
          {modules.map((module) => (
            <Link to={module.link} key={module.id} className={styles.cardLink}>
              <div className={clsx('card', styles.moduleCard)}>
                <div className={styles.cardIconPlaceholder}>{module.icon}</div> {/* Placeholder for actual icon component */}
                <h3 className={styles.cardTitle}>{module.title}</h3>
                <p className={styles.cardSubtitle}>{module.subtitle}</p>
                <div className={styles.cardTags}>
                  {module.tags.map((tag) => (
                    <span key={tag} className={styles.tag}>{tag}</span>
                  ))}
                </div>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default ModulesGrid;
