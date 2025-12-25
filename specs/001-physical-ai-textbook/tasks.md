# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested in specification. Test tasks excluded per template guidelines.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Frontend: Docusaurus static site with React components
- Backend: FastAPI with SQLAlchemy models

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both frontend and backend

- [x] T001 Create monorepo project structure with frontend/ and backend/ directories
- [x] T002 [P] Initialize Docusaurus 3.x project in frontend/ with TypeScript configuration
- [x] T003 [P] Initialize Python 3.11 project in backend/ with Poetry and pyproject.toml
- [x] T004 [P] Configure ESLint and Prettier for frontend in frontend/.eslintrc.js and frontend/.prettierrc
- [x] T005 [P] Configure ruff linter for backend in backend/pyproject.toml
- [x] T006 [P] Create .env.example files in both frontend/.env.example and backend/.env.example
- [x] T007 [P] Setup GitHub Actions CI workflow in .github/workflows/ci.yml
- [x] T008 Create shared documentation structure in frontend/docs/ with module directories

**Checkpoint**: Project scaffolding complete - ready for foundational infrastructure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & Backend Core

- [x] T009 Configure Neon PostgreSQL connection in backend/src/lib/database.py with async SQLAlchemy
- [x] T010 Setup Alembic migrations framework in backend/alembic/
- [x] T011 Create base SQLAlchemy model with UUID primary key in backend/src/models/base.py
- [x] T012 [P] Create Module model in backend/src/models/module.py per data-model.md
- [x] T013 [P] Create Chapter model in backend/src/models/chapter.py per data-model.md
- [x] T014 Create initial migration 001_initial_schema.py in backend/alembic/versions/
- [x] T015 Create seed data script for modules and chapters in backend/scripts/seed_data.py

### API Infrastructure

- [x] T016 Setup FastAPI application in backend/src/main.py with CORS, middleware
- [x] T017 Create rate limiter middleware in backend/src/lib/rate_limiter.py (100 req/min per user)
- [x] T018 Setup error handling and logging in backend/src/lib/errors.py
- [x] T019 Create API router structure in backend/src/api/__init__.py

### Vector Database Setup

- [x] T020 Configure Qdrant Cloud client in backend/src/lib/qdrant.py
- [x] T021 Create collection 'textbook_chunks' initialization script in backend/scripts/init_qdrant.py

### Frontend Core

- [x] T022 Configure docusaurus.config.ts with site metadata, theme, and plugins
- [x] T023 Setup sidebars.ts with module/chapter navigation structure
- [x] T024 Create API client service in frontend/src/services/api.ts with fetch wrapper
- [x] T025 Configure i18n for English and Urdu in frontend/docusaurus.config.ts

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Browse and Read Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Students can browse modules, read chapters, view code examples with syntax highlighting, and navigate seamlessly

**Independent Test**: Navigate to site, browse to Module 1, view a chapter with code examples, verify copy button works, test sidebar navigation

### Content Structure

- [x] T026 [P] [US1] Create intro chapter MDX files in frontend/docs/intro/
- [x] T027 [P] [US1] Create Module 1 (ROS 2) chapter structure in frontend/docs/module-1-ros2/
- [x] T028 [P] [US1] Create Module 2 (Simulation) placeholder chapters in frontend/docs/module-2-simulation/
- [x] T029 [P] [US1] Create Module 3 (Isaac) placeholder chapters in frontend/docs/module-3-isaac/
- [x] T030 [P] [US1] Create Module 4 (VLA) placeholder chapters in frontend/docs/module-4-vla/
- [x] T031 [P] [US1] Create Capstone chapter structure in frontend/docs/capstone/

### Enhanced Components

- [x] T032 [US1] Create enhanced CodeBlock component with copy button in frontend/src/components/CodeBlock/index.tsx
- [x] T033 [US1] Create collapsible Solution component for exercises in frontend/src/components/Solution/index.tsx
- [x] T034 [US1] Create LearningObjectives component for chapter headers in frontend/src/components/LearningObjectives/index.tsx
- [x] T035 [US1] Create EstimatedTime component showing reading duration in frontend/src/components/EstimatedTime/index.tsx

### Theme Customization

- [x] T036 [US1] Customize Docusaurus theme colors and typography in frontend/src/css/custom.css
- [x] T037 [US1] Create responsive sidebar styles in frontend/src/theme/DocSidebar/index.tsx
- [x] T038 [US1] Add syntax highlighting themes for Python, YAML, bash in frontend/docusaurus.config.ts

### Landing Page

- [x] T039 [US1] Create landing page with module overview in frontend/src/pages/index.tsx
- [x] T040 [US1] Add hero section with course description in frontend/src/components/Hero/index.tsx
- [x] T041 [US1] Create module cards component in frontend/src/components/ModuleCard/index.tsx

**Checkpoint**: US1 complete - site is browsable with all content structure, code highlighting, navigation working

---

## Phase 4: User Story 2 - Ask Questions via RAG Chatbot (Priority: P2)

**Goal**: Students can ask questions and receive contextually relevant answers with source citations

**Independent Test**: Open chatbot, ask "What is a ROS 2 node?", verify response cites textbook chapter, test "Explain this" on selected text

### Backend - Embedding Service

- [x] T042 [US2] Create embedding service with OpenAI text-embedding-3-small in backend/src/services/embedding_service.py
- [x] T043 [US2] Create content indexing script to chunk and embed MDX files in backend/scripts/index_content.py
- [x] T044 [US2] Create verify indexing script in backend/scripts/verify_index.py

### Backend - Chatbot Service

- [x] T045 [US2] Create Conversation model in backend/src/models/conversation.py per data-model.md
- [x] T046 [US2] Create Message model in backend/src/models/message.py per data-model.md
- [x] T047 [US2] Create migration 002_conversations.py in backend/alembic/versions/
- [x] T048 [US2] Implement chatbot service with RAG in backend/src/services/chatbot_service.py
- [x] T049 [US2] Add streaming response support with SSE in backend/src/services/chatbot_service.py

### Backend - Chatbot API

- [x] T050 [US2] Create chatbot API router per contracts/chatbot.yaml in backend/src/api/chatbot.py
- [x] T051 [US2] Implement POST /chatbot/conversations endpoint in backend/src/api/chatbot.py
- [x] T052 [US2] Implement POST /chatbot/conversations/{id}/messages endpoint with streaming in backend/src/api/chatbot.py
- [x] T053 [US2] Implement POST /chatbot/explain endpoint for selected text in backend/src/api/chatbot.py
- [x] T054 [US2] Implement POST /chatbot/feedback endpoint in backend/src/api/chatbot.py

### Frontend - Chatbot Widget

- [x] T055 [US2] Create chatbot service client in frontend/src/services/chatbot.ts
- [x] T056 [US2] Create Chatbot container component in frontend/src/components/Chatbot/Chatbot.tsx
- [x] T057 [US2] Create ChatMessage component in frontend/src/components/Chatbot/ChatMessage.tsx
- [x] T058 [US2] Create ChatInput component with send button in frontend/src/components/Chatbot/ChatInput.tsx
- [x] T059 [US2] Create Citation component for source links (integrated in ChatMessage.tsx)
- [x] T060 [US2] Implement streaming response display in frontend/src/components/Chatbot/Chatbot.tsx
- [x] T061 [US2] Add "Explain this" context menu on text selection in frontend/src/components/Chatbot/ExplainThis.tsx
- [x] T062 [US2] Integrate chatbot widget into Docusaurus theme in frontend/src/theme/Root/index.tsx

**Checkpoint**: US2 complete - chatbot responds to questions with citations, streaming works, "Explain this" functional

---

## Phase 5: User Story 3 - Create Account and Track Progress (Priority: P3)

**Goal**: Users can register, login, complete background assessment, and track chapter completion

**Independent Test**: Create account, complete background assessment, mark chapter complete, verify progress appears on dashboard

### Backend - User & Auth

- [x] T063 [US3] Create User model in backend/src/models/user.py per data-model.md
- [x] T064 [US3] Create Progress model in backend/src/models/progress.py per data-model.md
- [x] T065 [US3] Create migration 003_users_progress.py in backend/alembic/versions/
- [x] T066 [US3] Implement auth service with Better-Auth in backend/src/services/auth_service.py
- [x] T067 [US3] Create auth API router per contracts/auth.yaml in backend/src/api/auth.py
- [x] T068 [US3] Implement signup, signin, signout endpoints in backend/src/api/auth.py
- [x] T069 [US3] Implement password reset flow in backend/src/api/auth.py

### Backend - Users API

- [x] T070 [US3] Create users API router per contracts/users.yaml in backend/src/api/users.py
- [x] T071 [US3] Implement GET/PATCH /users/me endpoints in backend/src/api/users.py
- [x] T072 [US3] Implement GET/PUT /users/me/background endpoints in backend/src/api/users.py

### Backend - Progress API

- [x] T073 [US3] Create progress service in backend/src/services/progress_service.py
- [x] T074 [US3] Create progress API router per contracts/progress.yaml in backend/src/api/progress.py
- [x] T075 [US3] Implement GET /progress endpoint in backend/src/api/progress.py
- [x] T076 [US3] Implement PATCH /progress/chapters/{id} endpoint in backend/src/api/progress.py
- [x] T077 [US3] Implement POST /progress/chapters/{id}/complete endpoint in backend/src/api/progress.py
- [x] T078 [US3] Implement GET /progress/dashboard endpoint in backend/src/api/progress.py
- [x] T079 [US3] Implement POST /progress/heartbeat for reading activity in backend/src/api/progress.py

### Frontend - Auth

- [x] T080 [US3] Create auth service client with Better-Auth in frontend/src/services/auth.ts
- [x] T081 [US3] Create AuthProvider context in frontend/src/context/AuthContext.tsx
- [x] T082 [US3] Create SignupForm component in frontend/src/components/Auth/SignupForm.tsx
- [x] T083 [US3] Create SigninForm component in frontend/src/components/Auth/SigninForm.tsx
- [x] T084 [US3] Create auth pages (login, signup, reset) in frontend/src/pages/auth/

### Frontend - Background Assessment

- [x] T085 [US3] Create BackgroundAssessment wizard component in frontend/src/components/BackgroundAssessment/index.tsx
- [x] T086 [US3] Create skill level selector component in frontend/src/components/BackgroundAssessment/SkillSelector.tsx
- [x] T087 [US3] Create learning goals selector in frontend/src/components/BackgroundAssessment/GoalsSelector.tsx

### Frontend - Progress Tracking

- [x] T088 [US3] Create ProgressTracker component for chapter completion in frontend/src/components/ProgressTracker/index.tsx
- [x] T089 [US3] Create progress dashboard page in frontend/src/pages/dashboard.tsx
- [x] T090 [US3] Create ProgressBar component showing module completion in frontend/src/components/ProgressBar/index.tsx
- [x] T091 [US3] Integrate "Mark Complete" button into chapter layout in frontend/src/theme/DocItem/index.tsx
- [x] T092 [US3] Implement reading heartbeat (auto-save every 30s) in frontend/src/hooks/useReadingProgress.ts

**Checkpoint**: US3 complete - users can register, track progress, progress syncs across devices

---

## Phase 6: User Story 4 - Receive Personalized Content Recommendations (Priority: P4)

**Goal**: System recommends content paths based on user background, chatbot adjusts explanation depth

**Independent Test**: Create account with "Advanced Python, No ROS", verify skip recommendations appear, test chatbot adjusts responses

### Backend - Recommendations

- [x] T093 [US4] Implement recommendation engine in backend/src/services/recommendation_service.py
- [x] T094 [US4] Implement GET /progress/recommendations endpoint in backend/src/api/progress.py
- [x] T095 [US4] Update chatbot service to include user background in context in backend/src/services/chatbot_service.py

### Frontend - Recommendations

- [x] T096 [US4] Create SkipRecommendation banner component in frontend/src/components/Recommendations/SkipBanner.tsx
- [x] T097 [US4] Create NextChapter recommendation component in frontend/src/components/Recommendations/NextChapter.tsx
- [x] T098 [US4] Integrate recommendations into chapter sidebar in frontend/src/theme/DocSidebar/index.tsx
- [x] T099 [US4] Add recommendation section to dashboard in frontend/src/pages/dashboard.tsx

**Checkpoint**: US4 complete - personalized recommendations shown, chatbot adapts to user level

---

## Phase 7: User Story 5 - Read Content in Urdu (Priority: P5)

**Goal**: Pakistani students can read prose content in Urdu while code remains in English

**Independent Test**: Switch to Urdu, verify prose translates, code blocks stay English, chatbot responds in Urdu

### Translation Infrastructure

- [x] T100 [US5] Run Docusaurus write-translations to generate translation files
- [x] T101 [P] [US5] Create Urdu locale configuration in frontend/i18n/ur/
- [x] T102 [P] [US5] Create technical term glossary (English→Urdu) in frontend/i18n/ur/glossary.json
- [x] T103 [US5] Translate site chrome (navbar, footer, buttons) in frontend/i18n/ur/docusaurus-theme-classic/

### Content Translation

- [x] T104 [P] [US5] Translate intro chapters to Urdu in frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro/
- [x] T105 [P] [US5] Translate Module 1 chapters to Urdu in frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/
- [x] T106 [US5] Create translation status indicator component in frontend/src/components/TranslationStatus/index.tsx

### Chatbot Translation

- [x] T107 [US5] Update chatbot service to detect and respond in user's language in backend/src/services/chatbot_service.py
- [x] T108 [US5] Add language preference to conversation context in backend/src/services/chatbot_service.py

### Language Switcher

- [x] T109 [US5] Create language switcher component in frontend/src/components/LanguageSwitcher/index.tsx
- [x] T110 [US5] Persist language preference to user profile in frontend/src/services/auth.ts

**Checkpoint**: US5 complete - site available in Urdu, chatbot responds in Urdu, technical terms handled correctly

---

## Phase 8: User Story 6 - Complete Capstone Project (Priority: P6)

**Goal**: Advanced students complete capstone project with milestone tracking and chatbot assistance

**Independent Test**: Navigate to capstone, view milestones, mark milestone complete, ask chatbot for capstone-specific help

### Capstone Content

- [x] T111 [US6] Create capstone project overview MDX in frontend/docs/capstone/index.mdx
- [x] T112 [P] [US6] Create Milestone 1 content (ROS 2 setup) in frontend/docs/capstone/milestone-1.mdx
- [x] T113 [P] [US6] Create Milestone 2 content (Gazebo simulation) in frontend/docs/capstone/milestone-2.mdx
- [x] T114 [P] [US6] Create Milestone 3 content (Isaac integration) in frontend/docs/capstone/milestone-3.mdx
- [x] T115 [P] [US6] Create Milestone 4 content (voice interface) in frontend/docs/capstone/milestone-4.mdx
- [x] T116 [P] [US6] Create Milestone 5 content (integration) in frontend/docs/capstone/milestone-5.mdx

### Capstone Components

- [x] T117 [US6] Create MilestoneTracker component in frontend/src/components/Capstone/MilestoneTracker.tsx
- [x] T118 [US6] Create PrerequisiteChecklist component in frontend/src/components/Capstone/PrerequisiteChecklist.tsx
- [x] T119 [US6] Create ValidationScript component for testing progress in frontend/src/components/Capstone/ValidationScript.tsx

### Chatbot Capstone Support

- [x] T120 [US6] Add capstone-specific context to chatbot in backend/src/services/chatbot_service.py
- [x] T121 [US6] Create capstone troubleshooting embeddings in backend/scripts/index_capstone.py

**Checkpoint**: US6 complete - capstone walkthrough available with milestone tracking, chatbot assists with troubleshooting

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Documentation & Deployment

- [x] T122 [P] Update README.md with project overview and quickstart
- [x] T123 [P] Create CONTRIBUTING.md with development guidelines
- [x] T124 Configure GitHub Pages deployment in .github/workflows/deploy.yml
- [x] T125 Configure Vercel deployment for backend in backend/vercel.json
- [ ] T126 Run quickstart.md validation to verify setup instructions

### Performance & Optimization

- [x] T127 Configure Docusaurus build optimization (preload, prefetch)
- [x] T128 Implement API response caching for static data
- [x] T129 Add Lighthouse CI checks to GitHub Actions

### Security Hardening

- [x] T130 Review and update CORS configuration in backend/src/main.py
- [x] T131 Add rate limiting headers to API responses
- [x] T132 Implement session security best practices (rotation, secure cookies)
- [x] T133 Create security.md documenting auth flow and data handling

### Final Validation

> **Note**: These tasks require a running environment and are marked for manual execution.

- [ ] T134 End-to-end test: Complete US1-US6 flow manually
- [ ] T135 Accessibility audit with Lighthouse
- [ ] T136 Mobile responsiveness testing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational phase - MVP
- **User Story 2 (Phase 4)**: Depends on Foundational + content from US1
- **User Story 3 (Phase 5)**: Depends on Foundational phase
- **User Story 4 (Phase 6)**: Depends on US3 (requires user accounts)
- **User Story 5 (Phase 7)**: Depends on US1 (requires English content to translate)
- **User Story 6 (Phase 8)**: Depends on US1-US3 (requires core platform)
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

```
Phase 2 (Foundation)
       │
       ▼
   ┌───┴───┐
   │       │
   ▼       ▼
 US1     US3
 (P1)    (P3)
   │       │
   │       ▼
   │     US4
   │     (P4)
   │
   ├──────┐
   │      │
   ▼      ▼
 US2    US5
 (P2)   (P5)
   │
   └──────┐
          │
          ▼
        US6
        (P6)
```

### Parallel Opportunities

- **Phase 1**: T002, T003, T004, T005, T006, T007 can run in parallel
- **Phase 2**: T012, T013 can run in parallel after T011
- **Phase 3**: T026-T031 (content) can run in parallel; T032-T035 (components) can run in parallel
- **Phase 4**: T045, T046 can run in parallel; T055-T062 (frontend) can run after API complete
- **Phase 5**: T063, T064 can run in parallel; frontend components can run after API complete
- **Phase 6**: US4 can run in parallel with US2 and US5 (after US3)
- **Phase 7**: T101-T105 translation files can run in parallel
- **Phase 8**: T112-T116 milestone content can run in parallel

---

## Parallel Example: Phase 3 (User Story 1)

```bash
# Launch all content creation tasks together:
Task T026: "Create intro chapter MDX files in frontend/docs/intro/"
Task T027: "Create Module 1 chapters in frontend/docs/module-1-ros2/"
Task T028: "Create Module 2 placeholders in frontend/docs/module-2-simulation/"
Task T029: "Create Module 3 placeholders in frontend/docs/module-3-isaac/"
Task T030: "Create Module 4 placeholders in frontend/docs/module-4-vla/"
Task T031: "Create Capstone structure in frontend/docs/capstone/"

# Then launch all component creation tasks together:
Task T032: "Create CodeBlock component in frontend/src/components/CodeBlock/"
Task T033: "Create Solution component in frontend/src/components/Solution/"
Task T034: "Create LearningObjectives component in frontend/src/components/LearningObjectives/"
Task T035: "Create EstimatedTime component in frontend/src/components/EstimatedTime/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test US1 independently - can browse content, code highlights, navigation works
5. Deploy to GitHub Pages for early feedback

### Incremental Delivery

1. **MVP (US1)**: Browsable textbook with code highlighting → Deploy/Demo
2. **+US2**: Add RAG chatbot → Deploy/Demo (major differentiation)
3. **+US3**: Add user accounts and progress → Deploy/Demo
4. **+US4**: Add personalization → Deploy/Demo
5. **+US5**: Add Urdu translation → Deploy/Demo
6. **+US6**: Add capstone project → Deploy/Demo

### Suggested Team Assignment

| Developer | User Stories | Rationale |
|-----------|--------------|-----------|
| Dev A | US1, US5 | Content + Translation |
| Dev B | US2, US6 | Chatbot + Capstone |
| Dev C | US3, US4 | Auth + Personalization |

---

## Summary

| Phase | User Story | Tasks | Parallel Opportunities |
|-------|------------|-------|------------------------|
| 1 | Setup | 8 | 6 |
| 2 | Foundational | 17 | 4 |
| 3 | US1 - Browse Content (P1) 🎯 MVP | 16 | 10 |
| 4 | US2 - RAG Chatbot (P2) | 21 | 6 |
| 5 | US3 - Auth & Progress (P3) | 30 | 8 |
| 6 | US4 - Personalization (P4) | 7 | 0 |
| 7 | US5 - Urdu Translation (P5) | 11 | 5 |
| 8 | US6 - Capstone (P6) | 11 | 5 |
| 9 | Polish | 15 | 4 |
| **Total** | | **136** | **48** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- MVP = User Story 1 only (browsable textbook)
- Tests not included (not explicitly requested in spec)
