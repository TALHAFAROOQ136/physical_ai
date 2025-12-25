# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Build an AI-native educational platform for Physical AI & Humanoid Robotics using Docusaurus for content delivery, FastAPI for the backend API, Better-Auth for authentication, and a RAG chatbot powered by OpenAI with Qdrant vector storage. The platform supports English and Urdu languages, personalized learning paths, and progress tracking.

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11 (backend)
**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, Better-Auth client
- Backend: FastAPI, SQLAlchemy, OpenAI SDK, Qdrant client, Better-Auth
**Storage**: Neon Serverless PostgreSQL (users, progress, conversations), Qdrant Cloud (vector embeddings)
**Testing**: Vitest (frontend), pytest (backend)
**Target Platform**: GitHub Pages (static site), Serverless (API on Vercel/Railway)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Page load <3s, chatbot response <5s, 1000 concurrent users
**Constraints**: Serverless cold start <2s, API rate limit 100 req/min per user
**Scale/Scope**: 4 modules, ~15 chapters, 1000+ concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Plan Compliance | Status |
|-----------|-------------|-----------------|--------|
| 1. Production-Ready Code Quality | PEP 8, tested code, error handling | Python code linted with ruff, TypeScript with ESLint. Pytest + Vitest for testing. | PASS |
| 2. Pedagogical Effectiveness | Learning objectives, scaffolded progression, 3-5 exercises per module | Docusaurus supports MDX for structured content with frontmatter for objectives. | PASS |
| 3. Documentation Excellence | Clear explanations, diagrams, troubleshooting | Docusaurus native Mermaid support, admonitions for troubleshooting. | PASS |
| 4. Accessibility & Inclusivity | Heading hierarchy, alt text, responsive, Urdu support | Docusaurus i18n plugin for Urdu. Lighthouse accessibility checks in CI. | PASS |
| 5. Practical Hands-On Learning | Exercises with starter code, validation criteria | MDX components for collapsible solutions with expected outputs. | PASS |
| 6. Technical Accuracy & Currency | Version-specific, links to official docs | Content review workflow with version badges. | PASS |
| 7. Personalization & Adaptability | Background assessment, adaptive paths, personalized chatbot | User profile stores background; chatbot context includes user level. | PASS |
| 8. Security & Best Practices | Secure auth, rate limiting, env vars, no hardcoded secrets | Better-Auth with secure sessions, API rate limiting, .env for secrets. | PASS |

**Gate Status**: ALL PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (OpenAPI specs)
│   ├── auth.yaml
│   ├── users.yaml
│   ├── progress.yaml
│   └── chatbot.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Web application structure
frontend/
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Navigation structure
├── src/
│   ├── components/
│   │   ├── Chatbot/         # RAG chatbot widget
│   │   ├── ProgressTracker/ # Chapter completion UI
│   │   ├── CodeBlock/       # Enhanced code blocks
│   │   └── BackgroundAssessment/
│   ├── pages/
│   │   ├── index.tsx        # Landing page
│   │   ├── dashboard.tsx    # Progress dashboard
│   │   └── profile.tsx      # User settings
│   ├── services/
│   │   ├── api.ts           # Backend API client
│   │   ├── auth.ts          # Better-Auth client
│   │   └── chatbot.ts       # Chatbot service
│   └── theme/               # Docusaurus theme customization
├── docs/                    # Textbook content (MDX)
│   ├── intro/
│   ├── module-1-ros2/
│   ├── module-2-simulation/
│   ├── module-3-isaac/
│   ├── module-4-vla/
│   └── capstone/
├── i18n/
│   └── ur/                  # Urdu translations
│       └── docusaurus-plugin-content-docs/
└── tests/
    ├── unit/
    └── e2e/

backend/
├── src/
│   ├── main.py              # FastAPI app entry
│   ├── models/
│   │   ├── user.py
│   │   ├── progress.py
│   │   └── conversation.py
│   ├── services/
│   │   ├── auth_service.py
│   │   ├── progress_service.py
│   │   ├── chatbot_service.py
│   │   └── embedding_service.py
│   ├── api/
│   │   ├── auth.py
│   │   ├── users.py
│   │   ├── progress.py
│   │   └── chatbot.py
│   └── lib/
│       ├── database.py      # Neon connection
│       ├── qdrant.py        # Vector DB client
│       └── rate_limiter.py
├── tests/
│   ├── contract/
│   ├── integration/
│   └── unit/
└── alembic/                 # Database migrations
```

**Structure Decision**: Web application with separate frontend (Docusaurus static site) and backend (FastAPI serverless). Frontend deployed to GitHub Pages, backend to Vercel Serverless Functions or Railway.

## Complexity Tracking

> No constitution violations requiring justification.

| Decision | Rationale | Simpler Alternative Considered |
|----------|-----------|-------------------------------|
| Separate frontend/backend | Docusaurus generates static HTML; dynamic features need API | Single SSR app would require different framework |
| Qdrant Cloud (not self-hosted) | Serverless model, free tier available | Self-hosted requires infrastructure management |
| Better-Auth (not custom) | Production-ready auth, less security risk | Custom auth is error-prone |

## Technology Decisions Summary

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Static Site Generator | Docusaurus 3.x | MDX support, i18n built-in, excellent for technical docs |
| Backend Framework | FastAPI | Async Python, OpenAPI generation, serverless-friendly |
| Authentication | Better-Auth | Type-safe, supports Postgres, session-based auth |
| Database | Neon Serverless PostgreSQL | Serverless scaling, generous free tier |
| Vector Database | Qdrant Cloud | Free tier, good Python SDK, hosted solution |
| LLM Provider | OpenAI GPT-4 | Best-in-class for educational Q&A, function calling |
| Deployment (Frontend) | GitHub Pages | Free, integrated with repo, CDN |
| Deployment (Backend) | Vercel Serverless / Railway | Serverless scaling, easy deployment |

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Chatbot hallucination | Medium | High | Strict RAG with source citations, temperature=0 |
| Urdu translation quality | Medium | Medium | Human review pipeline, technical term glossary |
| Cold start latency | Low | Medium | Connection pooling, edge functions |
| Vector DB cost scaling | Low | Medium | Chunking strategy, embedding reuse |

## Next Steps

1. **Phase 0 Complete**: research.md generated with technology decisions
2. **Phase 1 Complete**: data-model.md, contracts/, quickstart.md generated
3. **Next**: Run `/sp.tasks` to generate implementation tasks
