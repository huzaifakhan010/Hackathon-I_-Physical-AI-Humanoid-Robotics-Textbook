import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './ModuleCard.module.css';

/**
 * ModuleCard component displays a card with title, description, and link
 * @param {Object} props - Component properties
 * @param {string} props.id - Unique identifier for the module
 * @param {string} props.title - Display name of the module
 * @param {string} props.description - Brief description of module content
 * @param {string} props.link - URL path to first chapter of the module
 * @param {number} props.order - Position in the grid layout
 */
function ModuleCard({ id, title, description, link, order }) {
  return (
    <div className={clsx('col col--3', styles.moduleCard)}>
      <Link
        to={link}
        className={styles.moduleCardLink}
        aria-label={`${title} - ${description}`}
      >
        <div className={styles.moduleCardHeader}>
          <h3 className={styles.moduleCardTitle} id={`${id}-title`}>{title}</h3>
        </div>
        <div className={styles.moduleCardBody}>
          <p className={styles.moduleCardDescription} id={`${id}-description`}>{description}</p>
        </div>
        <div className={styles.moduleCardFooter}>
          <span className={styles.moduleCardButton} aria-hidden="true">Start Learning</span>
        </div>
      </Link>
    </div>
  );
}

export default ModuleCard;