# Physical AI & Humanoid Robotics Textbook Platform

An AI-native educational platform for learning Physical AI and Humanoid Robotics. Built with Docusaurus, FastAPI, and a RAG-powered chatbot.

## Features

- **Interactive Textbook**: 4 modules covering ROS 2, Simulation, NVIDIA Isaac, and Vision-Language-Action
- **AI Chatbot**: RAG-powered assistant that answers questions with textbook citations
- **Progress Tracking**: Track your learning progress across chapters and modules
- **Bilingual Support**: Available in English and Urdu
- **Personalized Learning**: Adaptive content recommendations based on your background
- **Capstone Project**: Build your own humanoid robot system with milestone tracking

## Tech Stack

### Frontend
- **Docusaurus 3.x** - Static site generator for documentation
- **React 18** - UI components
- **TypeScript** - Type-safe JavaScript

### Backend
- **FastAPI** - Async Python web framework
- **SQLAlchemy** - ORM for database operations
- **OpenAI GPT-4** - LLM for chatbot responses
- **Qdrant** - Vector database for RAG

### Infrastructure
- **Neon** - Serverless PostgreSQL
- **GitHub Pages** - Frontend hosting
- **Vercel** - Backend serverless deployment

## Getting Started

### Prerequisites

- Node.js 20.x LTS
- Python 3.11+
- pnpm 8.x+
- Poetry 1.7+

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/your-org/physical-ai-textbook.git
   cd physical-ai-textbook
   ```

2. **Install frontend dependencies**
   ```bash
   cd frontend
   pnpm install
   ```

3. **Install backend dependencies**
   ```bash
   cd backend
   poetry install
   ```

4. **Configure environment variables**
   ```bash
   # Frontend
   cp frontend/.env.example frontend/.env.local

   # Backend
   cp backend/.env.example backend/.env
   ```

5. **Start development servers**
   ```bash
   # Terminal 1 - Frontend
   cd frontend
   pnpm start

   # Terminal 2 - Backend
   cd backend
   poetry run uvicorn src.main:app --reload
   ```

Visit http://localhost:3000 for the frontend and http://localhost:8000/docs for the API documentation.

## Project Structure

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
│   └── i18n/ur/              # Urdu translations
│
├── backend/                  # FastAPI server
│   ├── src/
│   │   ├── api/              # Route handlers
│   │   ├── models/           # SQLAlchemy models
│   │   ├── services/         # Business logic
│   │   └── lib/              # Utilities
│   ├── scripts/              # Utility scripts
│   └── alembic/              # Database migrations
│
└── specs/                    # Design documents
    └── 001-physical-ai-textbook/
        ├── spec.md
        ├── plan.md
        ├── tasks.md
        └── contracts/
```

## Course Modules

| Module | Topic | Duration |
|--------|-------|----------|
| 1 | ROS 2 - The Robotic Nervous System | 2 weeks |
| 2 | Simulation - The Digital Twin | 2 weeks |
| 3 | NVIDIA Isaac - The AI-Robot Brain | 2 weeks |
| 4 | VLA - Vision-Language-Action | 2 weeks |
| Capstone | Build Your Humanoid | 4 weeks |

## Development

### Running Tests

```bash
# Frontend tests
cd frontend
pnpm test

# Backend tests
cd backend
poetry run pytest
```

### Code Quality

```bash
# Frontend linting
cd frontend
pnpm lint

# Backend linting
cd backend
poetry run ruff check .
poetry run ruff format .
```

### Database Migrations

```bash
cd backend
poetry shell

# Create migration
alembic revision --autogenerate -m "description"

# Apply migrations
alembic upgrade head
```

### Indexing Content

```bash
cd backend
poetry shell

# Index all textbook content
python -m scripts.index_content

# Index capstone troubleshooting content
python -m scripts.index_capstone
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 Community
- NVIDIA Isaac Team
- OpenAI
- Docusaurus Team
