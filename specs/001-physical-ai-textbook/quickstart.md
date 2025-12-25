# Quickstart: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-23
**Phase**: 1 - Design & Contracts

This guide gets you from zero to a running development environment.

---

## Prerequisites

### Required Software

| Software | Version | Purpose |
|----------|---------|---------|
| Node.js | 20.x LTS | Frontend (Docusaurus) |
| Python | 3.11+ | Backend (FastAPI) |
| pnpm | 8.x+ | Node package manager |
| Poetry | 1.7+ | Python dependency manager |
| Git | 2.40+ | Version control |

### Accounts Needed

| Service | Purpose | Free Tier |
|---------|---------|-----------|
| [Neon](https://neon.tech) | PostgreSQL database | 0.5 GB |
| [Qdrant Cloud](https://cloud.qdrant.io) | Vector database | 1 GB |
| [OpenAI](https://platform.openai.com) | LLM + Embeddings | Pay-as-you-go |
| GitHub | Source control + Pages | Free |

---

## 1. Clone Repository

```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
git checkout 001-physical-ai-textbook
```

---

## 2. Environment Setup

### Create Environment Files

```bash
# Frontend (.env.local)
cp frontend/.env.example frontend/.env.local

# Backend (.env)
cp backend/.env.example backend/.env
```

### Configure Environment Variables

**frontend/.env.local**:
```env
# API URL (backend)
NEXT_PUBLIC_API_URL=http://localhost:8000/v1

# Better-Auth (client)
BETTER_AUTH_URL=http://localhost:8000
```

**backend/.env**:
```env
# Database (Neon)
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/textbook?sslmode=require

# Vector DB (Qdrant)
QDRANT_URL=https://xxx.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-key

# Better-Auth
BETTER_AUTH_SECRET=your-32-char-secret-minimum
BETTER_AUTH_URL=http://localhost:8000

# Environment
ENVIRONMENT=development
DEBUG=true
```

---

## 3. Backend Setup

```bash
cd backend

# Install dependencies
poetry install

# Activate virtual environment
poetry shell

# Run database migrations
alembic upgrade head

# Seed initial data (modules, chapters metadata)
python scripts/seed_data.py

# Start development server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify**: Open http://localhost:8000/docs to see OpenAPI documentation.

---

## 4. Frontend Setup

```bash
cd frontend

# Install dependencies
pnpm install

# Start development server
pnpm start
```

**Verify**: Open http://localhost:3000 to see the textbook site.

---

## 5. Vector Database Setup

### Index Textbook Content

Once backend is running and you have content in `frontend/docs/`:

```bash
cd backend
poetry shell

# Index all chapters into Qdrant
python scripts/index_content.py --docs-path ../frontend/docs

# Verify indexing
python scripts/verify_index.py
```

Expected output:
```
Indexed 15 chapters
Total chunks: 342
Collection 'textbook_chunks' ready
```

---

## 6. Verify Full Stack

### Test Authentication

```bash
# Create a test user
curl -X POST http://localhost:8000/v1/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "Test123!@#"}'
```

### Test Chatbot

```bash
# Create conversation and send message
curl -X POST http://localhost:8000/v1/chatbot/conversations \
  -H "Content-Type: application/json" \
  -d '{"chapterContext": "ros2-nodes"}'

# Use returned conversation ID
curl -X POST http://localhost:8000/v1/chatbot/conversations/{id}/messages \
  -H "Content-Type: application/json" \
  -d '{"content": "What is a ROS 2 node?", "stream": false}'
```

---

## 7. Development Workflow

### Frontend Development

```bash
cd frontend

# Start with hot reload
pnpm start

# Build for production
pnpm build

# Preview production build
pnpm serve

# Run tests
pnpm test
```

### Backend Development

```bash
cd backend
poetry shell

# Start with auto-reload
uvicorn src.main:app --reload

# Run tests
pytest

# Run with coverage
pytest --cov=src

# Format code
ruff format .

# Lint code
ruff check .
```

### Database Migrations

```bash
cd backend
poetry shell

# Create new migration
alembic revision --autogenerate -m "description"

# Apply migrations
alembic upgrade head

# Rollback one migration
alembic downgrade -1
```

---

## 8. Project Structure Overview

```
physical-ai-textbook/
├── frontend/                 # Docusaurus site
│   ├── docs/                 # Textbook content (MDX)
│   │   ├── intro/
│   │   ├── module-1-ros2/
│   │   ├── module-2-simulation/
│   │   ├── module-3-isaac/
│   │   ├── module-4-vla/
│   │   └── capstone/
│   ├── src/
│   │   ├── components/       # React components
│   │   ├── pages/            # Custom pages
│   │   └── services/         # API clients
│   ├── i18n/ur/              # Urdu translations
│   └── docusaurus.config.ts
│
├── backend/                  # FastAPI server
│   ├── src/
│   │   ├── api/              # Route handlers
│   │   ├── models/           # SQLAlchemy models
│   │   ├── services/         # Business logic
│   │   └── lib/              # Utilities
│   ├── tests/
│   ├── alembic/              # Migrations
│   └── pyproject.toml
│
├── specs/                    # SDD artifacts
│   └── 001-physical-ai-textbook/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md     # This file
│       ├── contracts/
│       └── tasks.md
│
└── .specify/                 # SDD tooling
```

---

## 9. Common Tasks

### Add New Chapter

1. Create MDX file in `frontend/docs/module-X/chapter-name.mdx`
2. Add chapter metadata to `backend/scripts/seed_data.py`
3. Run `python scripts/seed_data.py --chapters-only`
4. Re-index content: `python scripts/index_content.py`

### Add Urdu Translation

1. Run `cd frontend && pnpm write-translations --locale ur`
2. Edit generated JSON files in `frontend/i18n/ur/`
3. Have translation reviewed by native speaker
4. Commit and deploy

### Update API Contracts

1. Edit YAML files in `specs/001-physical-ai-textbook/contracts/`
2. Regenerate types: `cd backend && python scripts/generate_types.py`
3. Update implementation to match

---

## 10. Deployment

### Frontend (GitHub Pages)

```bash
cd frontend

# Build static site
pnpm build

# Deploy to GitHub Pages (via GitHub Actions)
git push origin 001-physical-ai-textbook
```

GitHub Action (`.github/workflows/deploy.yml`) handles deployment.

### Backend (Vercel)

```bash
# Install Vercel CLI
pnpm add -g vercel

# Deploy
cd backend
vercel --prod
```

Or connect GitHub repo to Vercel for auto-deployment.

---

## Troubleshooting

### Database Connection Issues

```bash
# Test Neon connection
psql "postgresql://user:pass@host/db?sslmode=require" -c "SELECT 1"

# Check pooler settings for serverless
# Use connection string ending with ?sslmode=require&pgbouncer=true
```

### Qdrant Issues

```bash
# Test Qdrant connection
curl -H "api-key: YOUR_KEY" https://xxx.cloud.qdrant.io/collections

# Recreate collection if needed
python scripts/recreate_collection.py
```

### OpenAI Rate Limits

```bash
# Check current usage
curl https://api.openai.com/v1/usage \
  -H "Authorization: Bearer $OPENAI_API_KEY"

# Implement exponential backoff in code (already in chatbot_service.py)
```

---

## Next Steps

After completing this quickstart:

1. **Write content**: Start creating chapters in `frontend/docs/`
2. **Customize theme**: Modify `frontend/src/theme/` for branding
3. **Add exercises**: Use MDX components for interactive exercises
4. **Run `/sp.tasks`**: Generate implementation task list
5. **Review contracts**: Ensure API contracts match requirements

---

## Support

- **Documentation**: See `specs/001-physical-ai-textbook/` for all design docs
- **Issues**: Open GitHub issue for bugs
- **Constitution**: Check `.specify/memory/constitution.md` for principles
