/**
 * Password reset page - DISABLED
 * This feature requires a backend server.
 */

import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function ResetPasswordPage(): JSX.Element {
  return (
    <Layout title="Reset Password" description="Reset your password">
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
        <h1>Password Reset Coming Soon</h1>
        <p style={{ color: 'var(--ifm-color-emphasis-600)', maxWidth: '500px' }}>
          Password reset requires a backend server which is not currently
          configured. You can still access all course content!
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
