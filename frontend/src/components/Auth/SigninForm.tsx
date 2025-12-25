/**
 * Signin form component.
 */

import React, { useState, FormEvent } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './AuthForm.module.css';

interface SigninFormProps {
  onSuccess?: () => void;
  onSwitchToSignup?: () => void;
  onForgotPassword?: () => void;
}

export function SigninForm({
  onSuccess,
  onSwitchToSignup,
  onForgotPassword,
}: SigninFormProps): JSX.Element {
  const { signin } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      await signin(email, password);
      onSuccess?.();
    } catch (err: unknown) {
      const errorMessage =
        err instanceof Error ? err.message : 'Invalid email or password';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.form}>
      <h2 className={styles.title}>Sign In</h2>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.field}>
        <label htmlFor="email" className={styles.label}>
          Email
        </label>
        <input
          type="email"
          id="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          className={styles.input}
          placeholder="you@example.com"
          required
        />
      </div>

      <div className={styles.field}>
        <label htmlFor="password" className={styles.label}>
          Password
        </label>
        <input
          type="password"
          id="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          className={styles.input}
          placeholder="Your password"
          required
        />
      </div>

      <div className={styles.forgotPasswordContainer}>
        <button
          type="button"
          className={styles.forgotPasswordLink}
          onClick={onForgotPassword}
        >
          Forgot password?
        </button>
      </div>

      <button type="submit" className={styles.submitButton} disabled={loading}>
        {loading ? 'Signing in...' : 'Sign In'}
      </button>

      <p className={styles.switchText}>
        Don't have an account?{' '}
        <button
          type="button"
          className={styles.switchLink}
          onClick={onSwitchToSignup}
        >
          Create one
        </button>
      </p>
    </form>
  );
}

export default SigninForm;
