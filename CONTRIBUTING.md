# Contributing to Physical AI Textbook Platform

Thank you for your interest in contributing to the Physical AI & Humanoid Robotics Textbook Platform!

## Getting Started

1. Fork the repository
2. Clone your fork locally
3. Set up the development environment (see README.md)
4. Create a feature branch from `main`

## Development Guidelines

### Code Style

#### Python (Backend)
- Follow PEP 8 style guide
- Use type hints for all function signatures
- Use `ruff` for linting and formatting
- Maximum line length: 88 characters

```bash
cd backend
poetry run ruff check .
poetry run ruff format .
```

#### TypeScript (Frontend)
- Follow ESLint configuration
- Use TypeScript strict mode
- Use Prettier for formatting

```bash
cd frontend
pnpm lint
pnpm format
```

### Commit Messages

Follow conventional commits format:

```
type(scope): description

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks

Examples:
```
feat(chatbot): add Urdu language support
fix(auth): resolve session expiration issue
docs(readme): update installation instructions
```

### Branch Naming

Use descriptive branch names:
- `feature/add-urdu-translations`
- `fix/chatbot-response-timeout`
- `docs/update-setup-guide`

## Pull Request Process

1. **Before submitting:**
   - Ensure all tests pass
   - Run linting and fix any issues
   - Update documentation if needed
   - Add tests for new features

2. **PR Description:**
   - Clearly describe the changes
   - Reference related issues
   - Include screenshots for UI changes
   - List any breaking changes

3. **Review Process:**
   - At least one approval required
   - All CI checks must pass
   - Address reviewer feedback

## Adding Content

### New Chapters

1. Create MDX file in `frontend/docs/`
2. Add frontmatter with title, description, sidebar_position
3. Update sidebar configuration if needed
4. Add chapter metadata to `backend/scripts/index_content.py`
5. Run content indexing script

### Translations

1. Run Docusaurus translation extraction:
   ```bash
   cd frontend
   pnpm write-translations --locale ur
   ```
2. Edit generated files in `frontend/i18n/ur/`
3. Keep code blocks in English
4. Use glossary for technical terms

### Components

1. Create component in `frontend/src/components/`
2. Export from `index.tsx`
3. Add types with TypeScript
4. Include JSDoc comments
5. Add usage example in documentation

## Testing

### Backend Tests

```bash
cd backend
poetry run pytest                    # Run all tests
poetry run pytest -v                 # Verbose output
poetry run pytest --cov=src          # With coverage
poetry run pytest -k "test_name"     # Run specific test
```

### Frontend Tests

```bash
cd frontend
pnpm test                            # Run all tests
pnpm test -- --watch                 # Watch mode
pnpm test -- --coverage              # With coverage
```

## Database Changes

1. Make model changes in `backend/src/models/`
2. Generate migration:
   ```bash
   alembic revision --autogenerate -m "description"
   ```
3. Review generated migration file
4. Test migration:
   ```bash
   alembic upgrade head
   alembic downgrade -1
   alembic upgrade head
   ```
5. Include migration in PR

## API Changes

1. Update OpenAPI contract in `specs/001-physical-ai-textbook/contracts/`
2. Implement changes in `backend/src/api/`
3. Update frontend client in `frontend/src/services/`
4. Add/update tests
5. Update API documentation

## Security

- Never commit secrets or API keys
- Use environment variables for configuration
- Report security issues privately
- Follow OWASP guidelines for web security

## Questions?

- Open a GitHub issue for bugs or feature requests
- Check existing issues before creating new ones
- Use discussions for general questions

## Code of Conduct

Be respectful and inclusive in all interactions. We're all here to learn and build together.
