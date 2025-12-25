/**
 * Sign up page - DISABLED
 * This feature requires a backend server.
 */

import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function SignUpPage(): JSX.Element {
  return (
    <Layout title="Create Account" description="Create a new account">
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
        <h1>Registration Coming Soon</h1>
        <p style={{ color: 'var(--ifm-color-emphasis-600)', maxWidth: '500px' }}>
          User registration requires a backend server which is not currently
          configured. You can still access all course content without an account!
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
