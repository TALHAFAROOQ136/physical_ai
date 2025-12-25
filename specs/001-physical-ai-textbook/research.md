# Research: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-23
**Phase**: 0 - Outline & Research

## Overview

This document captures technology research and decisions for implementing the Physical AI & Humanoid Robotics Textbook Platform. Each decision follows the format: Decision → Rationale → Alternatives Considered.

---

## 1. Static Site Generator

### Decision: Docusaurus 3.x

### Rationale
- **MDX Support**: Native support for JSX in Markdown, essential for interactive code examples and custom components
- **i18n Built-in**: First-class internationalization support with `@docusaurus/plugin-content-docs` for English/Urdu
- **Documentation Focus**: Purpose-built for technical documentation with versioning, search, and navigation
- **React Ecosystem**: Enables custom React components for chatbot, progress tracking, and interactive elements
- **Static Output**: Generates pure HTML/JS for GitHub Pages deployment with excellent SEO
- **Plugin Ecosystem**: Code highlighting (Prism), Mermaid diagrams, Algolia search integration

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Next.js | SSR complexity unnecessary for static content; would require custom doc infrastructure |
| VitePress | Vue-based; team more familiar with React; fewer plugins for interactive features |
| GitBook | Proprietary; limited customization for embedded chatbot |
| Astro | Newer ecosystem; less mature i18n support; smaller community |
| MkDocs | Python-based; limited interactive component support; no native React integration |

---

## 2. Backend Framework

### Decision: FastAPI (Python 3.11)

### Rationale
- **Async Native**: Built on Starlette for async I/O, critical for chatbot streaming responses
- **OpenAPI Generation**: Automatic API documentation and schema generation
- **Type Safety**: Pydantic models for request/response validation
- **Python Ecosystem**: Native integration with OpenAI SDK, SQLAlchemy, Qdrant client
- **Serverless Compatible**: Works well with Vercel, AWS Lambda, and Railway
- **Learning Alignment**: Python aligns with textbook content (ROS 2, AI libraries)

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Express.js (Node) | Would require TypeScript setup; Python better for AI/ML integration |
| Django | More opinionated; async support less mature; heavier for serverless |
| Flask | No native async; manual OpenAPI setup; less type safety |
| Go (Gin/Echo) | Team Python-focused; less AI library support |

---

## 3. Authentication

### Decision: Better-Auth

### Rationale
- **Type Safety**: Full TypeScript support with generated types
- **Database Adapters**: Native PostgreSQL/Neon support via Drizzle or Prisma
- **Session Management**: Secure session-based auth (not JWT-only)
- **Framework Agnostic**: Works with any frontend; REST API for FastAPI backend
- **Modern Security**: CSRF protection, secure cookies, session invalidation
- **Self-Hosted**: No vendor lock-in; runs on our infrastructure

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| NextAuth.js | Tightly coupled to Next.js; doesn't fit Docusaurus architecture |
| Lucia Auth | Being deprecated/maintenance mode as of 2024 |
| Auth0 | Vendor lock-in; cost at scale; overkill for educational site |
| Clerk | Paid service; less control over user data |
| Custom JWT | High security risk; reinventing solved problems |
| Supabase Auth | Would require Supabase stack; prefer Neon for database |

---

## 4. Database

### Decision: Neon Serverless PostgreSQL

### Rationale
- **Serverless Architecture**: Auto-scaling, pay-per-use, no connection management overhead
- **PostgreSQL Compatibility**: Full PostgreSQL features, JSONB for flexible schemas
- **Branching**: Database branching for development/staging/production
- **Free Tier**: Generous free tier (0.5 GB storage, 190 compute hours/month)
- **Connection Pooling**: Built-in pgbouncer for serverless function connections
- **Geographic Distribution**: Edge deployment for low latency

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Supabase | Good, but Neon has better serverless model and branching |
| PlanetScale | MySQL-based; prefer PostgreSQL for JSON support |
| MongoDB Atlas | NoSQL less suitable for relational user/progress data |
| SQLite (Turso) | Limited concurrent write support for user data |
| AWS RDS | Traditional server model; complex for serverless |

---

## 5. Vector Database

### Decision: Qdrant Cloud

### Rationale
- **Free Tier**: 1GB storage free, sufficient for textbook content embeddings
- **Python SDK**: Excellent `qdrant-client` library for FastAPI integration
- **Filtering**: Advanced metadata filtering for chapter/section scoping
- **Performance**: Optimized for semantic search with HNSW indexing
- **Managed Service**: No infrastructure to maintain
- **Payload Storage**: Can store source citations alongside vectors

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Pinecone | More expensive; complex pricing model |
| Weaviate Cloud | Less intuitive API; smaller community |
| Milvus | Self-hosted complexity; cloud offering newer |
| Chroma | Primarily for local/embedded use; cloud offering immature |
| pgvector | Would work, but Qdrant specialized for vector ops |

---

## 6. LLM Provider

### Decision: OpenAI GPT-4 (gpt-4-turbo)

### Rationale
- **Quality**: Best-in-class for educational explanations and Q&A
- **Function Calling**: Native support for structured outputs (source citations)
- **Streaming**: Server-sent events for real-time response streaming
- **Context Window**: 128K tokens for long conversation context
- **Reliability**: Most stable API; excellent uptime
- **Multi-language**: Strong Urdu language support

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Claude (Anthropic) | Excellent quality, but OpenAI has better function calling |
| Gemini (Google) | API less mature; quality inconsistent |
| Llama (self-hosted) | Infrastructure overhead; quality gap for education |
| Mixtral | Good for cost, but quality gap for nuanced explanations |

### Cost Considerations

| Model | Input Cost | Output Cost | Estimated Monthly |
|-------|------------|-------------|-------------------|
| gpt-4-turbo | $10/1M tokens | $30/1M tokens | ~$50-100 at 1000 users |
| gpt-3.5-turbo | $0.50/1M tokens | $1.50/1M tokens | ~$5-10 (fallback option) |

**Strategy**: Use gpt-4-turbo for primary responses; consider gpt-3.5-turbo for simple queries to manage costs.

---

## 7. Deployment Architecture

### Decision: GitHub Pages (Frontend) + Vercel Serverless (Backend)

### Frontend (Docusaurus → GitHub Pages)
- **Rationale**: Free hosting, automatic deployment from repo, global CDN
- **CI/CD**: GitHub Actions for build and deploy
- **Custom Domain**: physicalai-textbook.github.io or custom domain

### Backend (FastAPI → Vercel Serverless)
- **Rationale**: Automatic scaling, generous free tier, Python support
- **Alternative**: Railway (if Vercel Python limitations arise)
- **Edge Functions**: For rate limiting and auth validation

### Alternatives Considered

| Alternative | Why Rejected |
|-------------|--------------|
| Vercel (both) | Docusaurus on Vercel works but GH Pages simpler |
| Netlify | Similar to GH Pages; less integrated with repo |
| AWS (Amplify/Lambda) | More complex setup; overkill for MVP |
| Fly.io | Good, but Vercel serverless simpler for Python |
| Render | Good alternative; Vercel has better cold start |

---

## 8. Translation Strategy

### Decision: Docusaurus i18n with Human-Reviewed Machine Translation

### Workflow
1. **Content authoring**: Write chapters in English (MDX)
2. **Machine translation**: Use DeepL/GPT-4 for initial Urdu translation
3. **Human review**: Pakistani contributor reviews for accuracy
4. **Technical glossary**: Maintain English→Urdu term mapping
5. **Code preservation**: MDX parser extracts code blocks (keep English)

### Rationale
- **Docusaurus i18n**: Native support with `write-translations` CLI
- **Locale switching**: Built-in locale dropdown in navbar
- **Separate files**: `/i18n/ur/` directory for translated content
- **Fallback**: Missing translations display English with indicator

### Technical Terms Strategy

```
| English Term | Urdu Display | Notes |
|--------------|--------------|-------|
| URDF | URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) | Keep acronym, add explanation |
| ROS 2 | ROS 2 | Keep as-is (industry standard) |
| Topic | ٹاپک (Topic) | Transliterate with English parenthetical |
```

---

## 9. Embedding Strategy (RAG)

### Decision: OpenAI text-embedding-3-small with Chunk-by-Section

### Rationale
- **Model**: `text-embedding-3-small` (1536 dimensions) - good quality/cost ratio
- **Chunking**: By logical section (not fixed character count)
- **Metadata**: Store chapter_id, section_id, heading hierarchy
- **Overlap**: 100 token overlap between chunks for context

### Embedding Pipeline

```
1. Parse MDX → Extract prose (strip code blocks)
2. Split by ## headings → Create chunks
3. Add metadata: {chapter, section, module, heading_path}
4. Generate embeddings → Store in Qdrant
5. On query: Embed query → Search top-5 → Include in LLM context
```

### Cost Estimate

| Content | Estimated Tokens | Embedding Cost |
|---------|------------------|----------------|
| 4 modules × ~15 chapters | ~500,000 tokens | ~$0.01 total |
| Re-embedding (updates) | ~50,000 tokens/month | ~$0.001/month |

---

## 10. Chatbot Architecture

### Decision: Streaming RAG with Source Citations

### Architecture

```
User Query
    ↓
[Embed Query] → text-embedding-3-small
    ↓
[Vector Search] → Qdrant (top-5 chunks)
    ↓
[Context Assembly] → User background + Retrieved chunks + Conversation history
    ↓
[LLM Generation] → GPT-4 with streaming
    ↓
[Parse Response] → Extract citations, format links
    ↓
Streamed Response with Source Links
```

### Prompt Template (Simplified)

```
You are a helpful tutor for a Physical AI & Humanoid Robotics textbook.
User's background: {user_background}
Language preference: {language}

Relevant textbook content:
---
{retrieved_chunks_with_citations}
---

Answer the user's question. Always cite sources using [Chapter X.Y] format.
If the question is outside the textbook scope, politely redirect.
Adjust explanation depth based on user's stated experience level.
```

### Features
- **Streaming**: Server-sent events for real-time response
- **Citations**: Parse `[Chapter X.Y]` → link to actual chapter
- **Context window**: Last 10 messages + retrieved chunks
- **Fallback**: Graceful handling when no relevant content found

---

## Summary of Decisions

| Component | Decision | Key Rationale |
|-----------|----------|---------------|
| Static Site | Docusaurus 3.x | MDX, i18n, React ecosystem |
| Backend | FastAPI (Python 3.11) | Async, OpenAPI, AI libraries |
| Auth | Better-Auth | Type-safe, self-hosted, PostgreSQL |
| Database | Neon Serverless PostgreSQL | Serverless, branching, free tier |
| Vector DB | Qdrant Cloud | Free tier, Python SDK, filtering |
| LLM | OpenAI GPT-4 | Quality, function calling, streaming |
| Frontend Deploy | GitHub Pages | Free, integrated, CDN |
| Backend Deploy | Vercel Serverless | Scaling, free tier, Python support |
| Translation | Docusaurus i18n + human review | Native support, quality control |
| Embeddings | OpenAI text-embedding-3-small | Cost-effective, quality |

---

## Open Questions (for /sp.tasks)

1. **Content creation workflow**: Who writes chapters? What's the review process?
2. **Urdu reviewers**: Who are the Pakistani contributors for translation review?
3. **Domain name**: Custom domain or GitHub Pages default?
4. **Analytics**: What user behavior tracking is needed?
5. **Error tracking**: Sentry or similar for production monitoring?

These will be addressed during implementation planning.
