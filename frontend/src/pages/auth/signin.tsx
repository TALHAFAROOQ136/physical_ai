/**
 * Sign in page - DISABLED
 * This feature requires a backend server.
 */

import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function SignInPage(): JSX.Element {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <main
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '50vh',
          padding: '2rem',
          textAlign: 'center',
        }}
      >
        <h1>Authentication Coming Soon</h1>
        <p style={{ color: 'var(--ifm-color-emphasis-600)', maxWidth: '500px' }}>
          User authentication requires a backend server which is not currently
          configured. You can still access all course content without signing in!
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
