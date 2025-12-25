# Data Model: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-23
**Phase**: 1 - Design & Contracts

## Overview

This document defines the data model for the textbook platform. The model supports user authentication, progress tracking, personalization, and chatbot conversations.

**Database**: Neon Serverless PostgreSQL
**ORM**: SQLAlchemy 2.x with async support
**Migrations**: Alembic

---

## Entity Relationship Diagram

```
┌─────────────┐       ┌─────────────┐       ┌─────────────┐
│    User     │       │   Module    │       │   Chapter   │
├─────────────┤       ├─────────────┤       ├─────────────┤
│ id (PK)     │       │ id (PK)     │       │ id (PK)     │
│ email       │       │ title       │       │ module_id(FK)│
│ password_h  │       │ description │       │ title       │
│ background  │       │ order_seq   │       │ slug        │
│ language    │       │ objectives  │       │ order_seq   │
│ created_at  │       └──────┬──────┘       │ est_time_min│
│ updated_at  │              │              │ objectives  │
└──────┬──────┘              └──────────────┴──────┬──────┘
       │                                           │
       │  ┌────────────────────────────────────────┘
       │  │
       ▼  ▼
┌─────────────────┐
│    Progress     │
├─────────────────┤
│ id (PK)         │
│ user_id (FK)    │
│ chapter_id (FK) │
│ status          │
│ completed_at    │
│ reading_time_s  │
│ last_position   │
└─────────────────┘

┌─────────────┐       ┌─────────────┐
│Conversation │       │   Message   │
├─────────────┤       ├─────────────┤
│ id (PK)     │◄──────│ id (PK)     │
│ user_id(FK) │       │ conv_id(FK) │
│ context     │       │ role        │
│ created_at  │       │ content     │
│ updated_at  │       │ citations   │
└─────────────┘       │ created_at  │
                      └─────────────┘

┌─────────────┐
│   Session   │ (Better-Auth managed)
├─────────────┤
│ id (PK)     │
│ user_id(FK) │
│ token       │
│ expires_at  │
│ user_agent  │
│ ip_address  │
└─────────────┘
```

---

## Entities

### 1. User

Represents a registered learner on the platform.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, DEFAULT uuid_generate_v4() | Unique identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL | User's email address |
| `password_hash` | VARCHAR(255) | NOT NULL | Argon2id hashed password |
| `display_name` | VARCHAR(100) | NULL | Optional display name |
| `background` | JSONB | NOT NULL, DEFAULT '{}' | Background assessment data |
| `language_pref` | VARCHAR(10) | NOT NULL, DEFAULT 'en' | 'en' or 'ur' |
| `email_verified` | BOOLEAN | NOT NULL, DEFAULT false | Email verification status |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Account creation time |
| `updated_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last profile update |

**Background JSONB Schema**:
```json
{
  "python_level": "beginner|intermediate|advanced",
  "ros_experience": "none|basic|experienced",
  "hardware_access": "simulation_only|jetson|full_kit",
  "learning_goals": ["ros2", "simulation", "vla", "capstone"],
  "completed_assessment": true
}
```

**Indexes**:
- `idx_users_email` on `email` (unique)
- `idx_users_created` on `created_at`

---

### 2. Module

Groups related chapters into a learning unit.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | VARCHAR(50) | PK | Stable identifier (e.g., 'ros2', 'simulation') |
| `title` | VARCHAR(200) | NOT NULL | Display title |
| `description` | TEXT | NOT NULL | Module overview |
| `order_sequence` | INTEGER | NOT NULL, UNIQUE | Display order (1-4) |
| `objectives` | JSONB | NOT NULL | Learning objectives array |
| `icon` | VARCHAR(50) | NULL | Icon identifier for UI |

**Note**: Modules are seeded data, not user-created. Changes require migration.

**Sample Data**:
```sql
INSERT INTO modules (id, title, order_sequence) VALUES
('ros2', 'The Robotic Nervous System (ROS 2)', 1),
('simulation', 'The Digital Twin (Gazebo & Unity)', 2),
('isaac', 'The AI-Robot Brain (NVIDIA Isaac)', 3),
('vla', 'Vision-Language-Action (VLA)', 4);
```

---

### 3. Chapter

A unit of textbook content within a module.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | VARCHAR(100) | PK | Stable identifier (e.g., 'ros2-nodes') |
| `module_id` | VARCHAR(50) | FK → modules.id, NOT NULL | Parent module |
| `title` | VARCHAR(200) | NOT NULL | Chapter display title |
| `slug` | VARCHAR(100) | NOT NULL, UNIQUE | URL-friendly slug |
| `order_sequence` | INTEGER | NOT NULL | Order within module |
| `estimated_time_min` | INTEGER | NOT NULL | Estimated reading time |
| `objectives` | JSONB | NOT NULL | Chapter learning objectives |
| `prerequisites` | JSONB | DEFAULT '[]' | Required prior chapters |
| `difficulty` | VARCHAR(20) | NOT NULL | 'beginner', 'intermediate', 'advanced' |

**Indexes**:
- `idx_chapters_module` on `module_id`
- `idx_chapters_slug` on `slug` (unique)
- `idx_chapters_order` on `(module_id, order_sequence)`

**Note**: Chapter content lives in MDX files (docs/), not in database. This table holds metadata for progress tracking and navigation.

---

### 4. Progress

Tracks user completion status for chapters.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, DEFAULT uuid_generate_v4() | Unique identifier |
| `user_id` | UUID | FK → users.id, NOT NULL | User reference |
| `chapter_id` | VARCHAR(100) | FK → chapters.id, NOT NULL | Chapter reference |
| `status` | VARCHAR(20) | NOT NULL, DEFAULT 'not_started' | Progress status |
| `completed_at` | TIMESTAMPTZ | NULL | When marked complete |
| `reading_time_seconds` | INTEGER | DEFAULT 0 | Accumulated reading time |
| `last_position` | VARCHAR(100) | NULL | Last scroll position/section |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | First interaction time |
| `updated_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last update time |

**Status Values**:
- `not_started`: User hasn't viewed chapter
- `in_progress`: User has started but not completed
- `completed`: User marked as complete

**Constraints**:
- `UNIQUE(user_id, chapter_id)` - One progress record per user-chapter pair

**Indexes**:
- `idx_progress_user` on `user_id`
- `idx_progress_user_chapter` on `(user_id, chapter_id)` (unique)
- `idx_progress_status` on `(user_id, status)`

---

### 5. Conversation

Stores chatbot conversation sessions.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, DEFAULT uuid_generate_v4() | Unique identifier |
| `user_id` | UUID | FK → users.id, NULL | User reference (NULL for anonymous) |
| `context` | JSONB | DEFAULT '{}' | Conversation context |
| `chapter_context` | VARCHAR(100) | NULL | Chapter user was viewing |
| `message_count` | INTEGER | DEFAULT 0 | Number of messages |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Session start |
| `updated_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last message time |

**Context JSONB Schema**:
```json
{
  "user_background": { ... },
  "current_module": "ros2",
  "current_chapter": "ros2-nodes",
  "selected_text": "..."
}
```

**Indexes**:
- `idx_conversations_user` on `user_id`
- `idx_conversations_updated` on `updated_at`

---

### 6. Message

Individual messages within a conversation.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, DEFAULT uuid_generate_v4() | Unique identifier |
| `conversation_id` | UUID | FK → conversations.id, NOT NULL | Parent conversation |
| `role` | VARCHAR(20) | NOT NULL | 'user' or 'assistant' |
| `content` | TEXT | NOT NULL | Message text content |
| `citations` | JSONB | DEFAULT '[]' | Source citations array |
| `metadata` | JSONB | DEFAULT '{}' | Additional metadata |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Message timestamp |

**Citations JSONB Schema**:
```json
[
  {
    "chapter_id": "ros2-nodes",
    "section": "2.3",
    "title": "Creating Your First Node",
    "relevance_score": 0.92
  }
]
```

**Indexes**:
- `idx_messages_conversation` on `conversation_id`
- `idx_messages_created` on `(conversation_id, created_at)`

---

### 7. Session (Better-Auth Managed)

Managed by Better-Auth library. Schema for reference:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | VARCHAR(255) | PK | Session token |
| `user_id` | UUID | FK → users.id, NOT NULL | User reference |
| `expires_at` | TIMESTAMPTZ | NOT NULL | Session expiration |
| `user_agent` | TEXT | NULL | Browser user agent |
| `ip_address` | VARCHAR(45) | NULL | Client IP address |
| `created_at` | TIMESTAMPTZ | NOT NULL | Session creation |

**Note**: Better-Auth handles session table creation and management.

---

## Vector Store (Qdrant)

Not in PostgreSQL - stored in Qdrant Cloud.

### Collection: `textbook_chunks`

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Unique chunk identifier |
| `vector` | float[1536] | text-embedding-3-small vector |
| `payload.chapter_id` | string | Source chapter ID |
| `payload.section_id` | string | Section identifier |
| `payload.heading_path` | string[] | Heading hierarchy |
| `payload.content` | string | Chunk text content |
| `payload.module_id` | string | Parent module ID |
| `payload.chunk_index` | int | Position in chapter |

**Index Configuration**:
```json
{
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "optimizers_config": {
    "default_segment_number": 2
  },
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  }
}
```

---

## Migrations

### Initial Migration (001_initial_schema.py)

```python
# Alembic migration - creates all tables
def upgrade():
    # Enable UUID extension
    op.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp"')

    # Create modules table
    op.create_table('modules', ...)

    # Create chapters table
    op.create_table('chapters', ...)

    # Create users table
    op.create_table('users', ...)

    # Create progress table
    op.create_table('progress', ...)

    # Create conversations table
    op.create_table('conversations', ...)

    # Create messages table
    op.create_table('messages', ...)

    # Seed modules data
    op.bulk_insert(modules_table, [...])
```

---

## Validation Rules

### User
- Email: Valid email format, max 255 chars
- Password: Min 8 chars, 1 number, 1 special char (validated pre-hash)
- Display name: Max 100 chars, alphanumeric + spaces
- Language: Must be 'en' or 'ur'

### Progress
- Status: Must be one of ['not_started', 'in_progress', 'completed']
- Reading time: Non-negative integer
- completed_at: Required when status = 'completed'

### Message
- Role: Must be 'user' or 'assistant'
- Content: Max 10,000 chars
- Citations: Valid JSON array

---

## State Transitions

### Progress States

```
                    ┌─────────────┐
                    │ not_started │
                    └──────┬──────┘
                           │ view_chapter()
                           ▼
                    ┌─────────────┐
        ┌──────────►│ in_progress │◄──────────┐
        │           └──────┬──────┘           │
        │                  │                  │
        │ resume()         │ mark_complete()  │ unmark()
        │                  ▼                  │
        │           ┌─────────────┐           │
        └───────────│  completed  ├───────────┘
                    └─────────────┘
```

### Conversation Lifecycle

```
create_conversation() → ACTIVE
    ↓
add_message() [repeats]
    ↓
[30 min inactivity] → STALE
    ↓
[user returns] → ACTIVE (new conversation)
```

---

## Data Retention

| Data Type | Retention | Rationale |
|-----------|-----------|-----------|
| User accounts | Indefinite (until deletion request) | Core functionality |
| Progress data | Tied to user account | User value |
| Conversations | 90 days | Storage optimization |
| Messages | 90 days | Tied to conversation |
| Sessions | 30 days | Security best practice |

**Deletion Cascade**:
- User deletion → Progress, Conversations, Messages, Sessions

---

## Performance Considerations

1. **Progress queries**: Most frequent query is "get all progress for user"
   - Index on `user_id` is critical
   - Consider materialized view for dashboard

2. **Chapter listing**: Cached at CDN level (static content)

3. **Conversation history**: Limit to last 10 messages in context
   - Paginate older messages

4. **Vector search**: Qdrant handles indexing
   - Monitor query latency in production
