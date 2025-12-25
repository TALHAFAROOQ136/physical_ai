/**
 * Dashboard page - DISABLED
 * This feature requires a backend server.
 */

import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function DashboardPage(): JSX.Element {
  return (
    <Layout title="Dashboard">
      <main style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '50vh',
        padding: '2rem',
        textAlign: 'center',
      }}>
        <h1>Dashboard Coming Soon</h1>
        <p style={{ color: 'var(--ifm-color-emphasis-600)', maxWidth: '500px' }}>
          The progress tracking dashboard requires a backend server which is not
          currently configured. For now, enjoy the course content!
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/intro"
          style={{ marginTop: '1rem' }}
        >
          Start Learning
        </Link>
      </main>
    </Layout>
  );
}
