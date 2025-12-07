---
id: 1
title: Create Complete Constitution for Physical AI Backend
stage: constitution
date: 2025-12-06
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: fix-github-integration-and-update-constitution
user: claude
command: /sp.constitution
labels: ["constitution", "backend", "fastapi", "qdrant", "neon", "ingestion", "api-limits"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

# Create Complete Constitution for Physical AI Backend

## User Prompt
```
/sp.constitution
Create a complete Constitution for building the backend of the
"Physical AI & Humanoid Robotics" book platform.

The system MUST include:
- Full FastAPI backend
- RAG chatbot
- Qdrant vector search
- Neon serverless Postgres
- Ingestion pipelines
- API limits for a free tier
- Cost-optimized OpenAI usage

################################################################
1. SYSTEM-WIDE REQUIREMENTS (UPDATED WITH INGESTION + FREE API)
################################################################

### A. Backend Stack Requirements
- FastAPI (Python, fully typed)
- Async architecture with httpx + asyncpg
- Neon Serverless Postgres (primary DB)
- Qdrant Cloud Free Tier (vector DB)
- OpenAI ChatKit/Agents for LLM logic
- Supports Windows + WSL2 development

### B. Document Ingestion System (MANDATORY)
The backend must include a full ingestion pipeline:

1. **Admin-only upload** of:
   - Markdown / MDX files
   - HTML sections
   - PDF pages (optional)
   - Raw text

2. Ingestion steps:
   a. Split content into chunks (300–500 tokens)
   b. Store chunk metadata in Postgres
   c. Generate embeddings (OpenAI text-embedding-3-small recommended)
   d. Store vectors + metadata in Qdrant
   e. Create full-text searchable index

3. Automatic versioning of ingested content
4. Re-ingest only updated lessons
5. Queue-based ingestion process with:
   - Background worker
   - Retry logic
   - Ingestion status logs

6. **Safety:**
   - Prevent duplicate documents
   - Prevent stale versions
   - Ensure chunk order is preserved

### C. Free API Tier Limits (MANDATORY)
Define a **"Free Tier"** with enforceable limits:

1. **Chatbot API:**
   - Max 30 queries/day per user
   - Max 5 RAG queries/minute
   - Max 5k tokens per response
   - Prevent long-running queries > 10 sec

2. **Search API:**
   - 50 search queries/day

3. **User Data Storage:**
   - 50 notes
   - 50 highlights
   - 10 bookmarks

4. **Rate Limits (Global):**
   - 100 requests/minute petions referencing lesson + chunk index
- Strict hallucination prevention rules

### B. Content Management
CRUD for:
- Modules
- Chapters
- Lessons
- Lesson sections
- Glossary
- Exercises
- Media assets

All content stored in Markdown/MDX with frontmatter.

### C. User Personalization
- Save notes, bookmarks, highlights
- Track progress per lesson
- Sync across devices

### D. Admin Panel Support (API only)
- Upload lesson files
- Trigger ingestion
- Check ingestion logs
- Rebuild embeddings button

################################################################
3. DATA MODELS (UPDATED)
################################################################

### Mandatory Models
- User
- Chapter
- Lesson
- LessonSection
- EmbeddingDocument
- RAGQuery
- ChatHistory
- Highlight
- Note
- APIUsageQuota
- IngestionJob
- BookMetadata

Every model must include:
- Pydantic schema
- DB table schema
- CRUD operations
- Validation rules

################################################################
4. API REQUIREMENTS (UPDATED)
################################################################

### Ingestion APIs
(ADMIN ONLY)
- POST /ingest/upload
- POST /ingest/run
- GET /ingest/status
- DELETE /ingest/remove/:id

### RAG APIs
- POST /rag/query
- POST /rag/query-selected (text provided by user)
- POST /rag/search
- POST /rag/semantic-search
- GET /rag/metadata

### Free Tier Controls (MIDDLEWARE)
Middleware must:
- Count daily usage
- Enforce quotas
- Return 429 rate limit message
- Log overuse

### User APIs
- /auth/login
- /auth/signup
- /auth/refresh
- /user/notes
- /user/highlights
- /user/bookmarks
- /user/progress

### Book APIs
- /book/chapters
- /book/lessons/:id
- /book/search

################################################################
5. DEVELOPER EXPERIENCE (UPDATED)
################################################################

Backend must include:
- Perfect folder structure for FastAPI
- Environment variable templates
- Local dev using Uvicorn + hot reload
- Integration guide for Docusaurus:
  - Chatbot widget
  - "Ask Questions About This Lesson" button
  - Selected-text → RAG API call workflow

################################################################
6. OUTPUT REQUIREMENTS
################################################################

The Constitution must enforce:
- High-level rules
- Architecture priorities
- Code quality gates
- Documentation requirements
- Versioning rules
- Ingestion safety
- Free API enforcement

################################################################
7. FINAL GOAL
################################################################

Produce a Constitution that enables /sp.spec and /sp.tasks to generate:

A **complete**, **secure**, **cost-efficient**,
production-ready backend with:

- FastAPI
- Neon serverless Postgres
- Qdrant vector store
- Ingestion system
- Free API usage limits
- RAG chatbot
- Full Docusaurus integration
```

## Response Summary
I have successfully updated the constitution file (.specify/memory/constitution.md) to include all the backend requirements you specified. The updated constitution now includes:

- Complete backend stack requirements (FastAPI, Neon Postgres, Qdrant)
- Comprehensive document ingestion system requirements
- Free API tier limits with enforceable quotas
- Cost optimization requirements for OpenAI usage
- All mandatory data models with Pydantic schemas and CRUD operations
- Complete API requirements for ingestion, RAG, user management, and book content
- Developer experience requirements for Docusaurus integration
- Architecture principles and quality assurance standards

The constitution has been updated from version 1.0.0 to 2.0.0 to reflect the addition of backend architecture, ingestion pipeline, API limits, and cost optimization principles.

## Outcome
The Physical AI and Humanoid Robotics Book Platform Constitution has been successfully updated to version 2.0.0 with comprehensive backend requirements that will enable /sp.spec and /sp.tasks to generate a complete, secure, cost-efficient, production-ready backend with all the specified features.