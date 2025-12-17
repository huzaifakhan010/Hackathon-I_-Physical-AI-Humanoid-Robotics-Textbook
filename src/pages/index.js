import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
// Comment out HomepageFeatures since we're replacing with module cards
// import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ModuleCard from '@site/src/components/ModuleCard';

import Heading from '@theme/Heading';
import styles from './index.module.css';

// Module data configuration based on data-model.md
const modules = [
  {
    id: "module-1-ros2",
    title: "ROS 2 – Robotic Nervous System",
    description: "Module 1 – Using ROS 2 as middleware to control humanoid robots, bridging AI agents with physical actuators",
    link: "/docs/module-1-ros2-arch/chapter-1-what-is-ros2",
    order: 1
  },
  {
    id: "module-2-gazebo",
    title: "Digital Twin (Gazebo & Unity)",
    description: "Module 2 – Creating photorealistic simulation environments for humanoid robot testing and development",
    link: "/docs/module-2-gazebo-unity/chapter-1-digital-twin-fundamentals",
    order: 2
  },
  {
    id: "module-3-isaac",
    title: "AI-Robot Brain (NVIDIA Isaac)",
    description: "Module 3 – Advanced perception, simulation, and navigation for humanoid robots using NVIDIA Isaac",
    link: "/docs/module-3-isaac/",
    order: 3
  },
  {
    id: "module-4-vla",
    title: "Vision-Language-Action (VLA)",
    description: "Module 4 – Integration of LLMs with robotics for voice-driven cognitive planning and autonomous humanoid behavior",
    link: "/docs/module-4-vla/chapter-1-voice-to-action",
    order: 4
  }
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Learn Robotics Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book">
      <HomepageHeader />
      <main>
        {/* Replace HomepageFeatures with module navigation */}
        <section className={styles.modulesSection}>
          <div className="container">
            <div className="row">
              <h2 className={styles.sectionTitle}>Course Modules :</h2>
              <p className={styles.sectionDescription}>
                <strong>&nbsp;&nbsp;Select a module to begin your journey into Physical AI and Humanoid Robotics</strong>
              </p>
            </div>
            <div className="row">
              {modules.map((module) => (
                <ModuleCard
                  key={module.id}
                  id={module.id}
                  title={module.title}
                  description={module.description}
                  link={module.link}
                  order={module.order}
                />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
