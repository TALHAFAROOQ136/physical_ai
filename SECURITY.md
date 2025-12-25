# Security Documentation

This document describes the security architecture, authentication flow, and data handling practices for the Physical AI & Humanoid Robotics Textbook Platform.

## Authentication Flow

### User Registration

1. User submits email and password to `POST /v1/auth/signup`
2. Password is validated:
   - Minimum 8 characters
   - Contains at least 1 number
   - Contains at least 1 special character
3. Password is hashed using Argon2id (memory-hard algorithm)
4. User record created in PostgreSQL
5. Session created and secure cookie returned

### User Login

1. User submits credentials to `POST /v1/auth/signin`
2. Email lookup in database
3. Password verified against Argon2id hash
4. Previous session invalidated (if exists)
5. New session created with secure cookie
6. Session stored in database with expiration

### Session Management

- **Cookie Settings**:
  - `HttpOnly`: Prevents JavaScript access
  - `Secure`: HTTPS only in production
  - `SameSite: Lax`: CSRF protection
  - `Max-Age`: 24 hours default

- **Session Rotation**: Sessions are regenerated on login
- **Session Invalidation**: Explicit logout clears session

## API Security

### CORS Configuration

```python
# Production settings
allow_origins: List of specific frontend domains (no wildcards)
allow_methods: ["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"]
allow_headers: ["Authorization", "Content-Type", "X-Requested-With"]
allow_credentials: True
```

### Security Headers

All API responses include:

| Header | Value | Purpose |
|--------|-------|---------|
| `X-Content-Type-Options` | `nosniff` | Prevent MIME sniffing |
| `X-Frame-Options` | `DENY` | Prevent clickjacking |
| `X-XSS-Protection` | `1; mode=block` | XSS filter |
| `Referrer-Policy` | `strict-origin-when-cross-origin` | Limit referrer info |
| `Strict-Transport-Security` | `max-age=31536000` | Force HTTPS (production) |

### Rate Limiting

- **Default**: 100 requests per minute per user
- **Chatbot**: Additional limits to prevent abuse
- **Headers**: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

## Data Protection

### Sensitive Data Handling

| Data Type | Storage | Encryption |
|-----------|---------|------------|
| Passwords | PostgreSQL | Argon2id hash |
| Sessions | PostgreSQL | Encrypted tokens |
| API Keys | Environment vars | Not stored in DB |
| User PII | PostgreSQL | Encrypted at rest (Neon) |

### Data Retention

| Data Type | Retention | Rationale |
|-----------|-----------|-----------|
| User accounts | Until deletion request | Core functionality |
| Progress data | Tied to user account | User value |
| Conversations | 90 days | Storage optimization |
| Sessions | 30 days | Security best practice |

### Data Deletion

User deletion cascades to:
- Progress records
- Conversation history
- Messages
- Active sessions

## Environment Variables

### Required in Production

```bash
# Database (use connection pooling)
DATABASE_URL=postgresql+asyncpg://...?sslmode=require

# Auth (minimum 32 characters)
BETTER_AUTH_SECRET=your-secure-secret-at-least-32-chars

# OpenAI (never commit)
OPENAI_API_KEY=sk-...

# Qdrant (if using cloud)
QDRANT_API_KEY=...

# Environment
ENVIRONMENT=production
DEBUG=false
```

### Never Commit

- `.env` files
- API keys
- Database credentials
- Auth secrets
- Private keys

## Vulnerability Prevention

### OWASP Top 10 Mitigations

| Risk | Mitigation |
|------|------------|
| Injection | Parameterized queries (SQLAlchemy), input validation |
| Broken Auth | Secure sessions, password hashing, rate limiting |
| Sensitive Data Exposure | HTTPS, encryption at rest, secure headers |
| XXE | No XML parsing in use |
| Broken Access Control | Session-based auth, role checks |
| Security Misconfiguration | Environment-specific configs, secure defaults |
| XSS | Content Security Policy, output encoding |
| Insecure Deserialization | Pydantic validation |
| Using Components with Vulnerabilities | Dependabot, regular updates |
| Insufficient Logging | Structured logging, audit trails |

### Input Validation

All API inputs validated using Pydantic models:
- Type checking
- Length limits
- Format validation
- Sanitization

### Output Encoding

- JSON responses properly encoded
- User content sanitized before display
- HTML entities escaped in frontend

## Incident Response

### Security Contact

Report security vulnerabilities to: security@your-domain.com

### Response Process

1. Acknowledge receipt within 24 hours
2. Assess severity and impact
3. Develop and test fix
4. Deploy fix
5. Notify affected users if data was exposed
6. Post-mortem documentation

## Compliance

### Data Privacy

- Users can request data export
- Users can request account deletion
- Minimal data collection
- Clear privacy policy

### Audit Logging

Logged events:
- Authentication attempts (success/failure)
- Password changes
- Session creation/invalidation
- Administrative actions

## Security Checklist for Deployment

- [ ] All secrets in environment variables
- [ ] HTTPS enabled
- [ ] CORS origins restricted
- [ ] Rate limiting configured
- [ ] Secure cookie settings
- [ ] Database connection uses SSL
- [ ] Debug mode disabled
- [ ] Error messages don't leak info
- [ ] Dependencies up to date
- [ ] Monitoring/alerting configured
