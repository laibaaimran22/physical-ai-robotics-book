<!-- Sync Impact Report:
Version change: 1.0.0 -> 2.0.0
List of modified principles: Added backend architecture, ingestion pipeline, API limits, and cost optimization principles
Added sections: System-wide Requirements, Architecture Principles, Data Models, API Requirements, Developer Experience
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (review for alignment with new principles)
- .specify/templates/spec-template.md: ⚠ pending (review for alignment with new principles)
- .specify/templates/tasks-template.md: ⚠ pending (review for alignment with new principles)
- .specify/templates/commands/*.md: ⚠ pending (review for outdated references)
- README.md: ⚠ pending (review for references to principles)
Follow-up TODOs: None
-->

# Physical AI and Humanoid Robotics Book Platform Constitution

## Vision
To provide an accessible, engaging, and technically advanced learning resource for beginners to intermediate learners in Physical AI and Humanoid Robotics, emphasizing practical, hands-on application with a robust backend system supporting intelligent content delivery and personalized learning experiences.

## Core Principles

### I. Accessibility & Clarity
Content must be clear, concise, and understandable for beginners, while offering depth for intermediate learners. Complex concepts are broken down with clear explanations and examples.

### II. Hands-on Learning
Prioritize practical exercises, code examples, and projects that enable readers to build and experiment directly. Theoretical concepts are reinforced with immediate application.

### III. Docusaurus-driven Documentation
All content and examples will be structured and delivered using Docusaurus, leveraging its features for maintainability, searchability, and versioning.

### IV. Accuracy & Up-to-dateness
Information presented must be technically accurate and reflect current best practices in Physical AI and Humanoid Robotics. Content will be regularly reviewed and updated.

### V. Backend Architecture Excellence
Build a robust, scalable, and secure backend using FastAPI, Neon serverless Postgres, and Qdrant vector search to support intelligent content delivery and user interactions.

### VI. Intelligent Content Ingestion
Implement a comprehensive document ingestion system that automatically processes various content formats, generates embeddings, and maintains content freshness with version control.

### VII. Free Tier & Resource Management
Implement resource management for free tier users through efficient caching, rate limiting, and intelligent processing to ensure sustainable operation of free services.

## System-wide Requirements

### A. Backend Stack Requirements
- FastAPI (Python, fully typed)
- Async architecture with httpx + asyncpg
- Neon Serverless Postgres (primary DB)
- Qdrant Cloud Free Tier (vector DB)
- Free LLM alternative for RAG (Ollama/Hugging Face/local model)
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
   - 100 requests/minute per user

### D. Resource Optimization Requirements
- Implement caching for frequently accessed content
- Optimize processing for efficient resource usage
- Monitor and alert on resource consumption
- Use efficient embedding strategies

## Architecture Principles

### 1. Async-First Design
All API endpoints and data operations must be asynchronous to handle concurrent users efficiently and prevent blocking operations.

### 2. Separation of Concerns
- API Layer: FastAPI endpoints with request/response validation
- Service Layer: Business logic and cross-cutting concerns
- Data Layer: Database operations and vector store interactions
- Integration Layer: External service communication (OpenAI, etc.)

### 3. Type Safety
All code must use proper type hints, Pydantic models for data validation, and strict typing to prevent runtime errors.

### 4. Security-First
- Authentication and authorization on all user-facing endpoints
- Rate limiting to prevent abuse
- Input sanitization to prevent injection attacks
- Secure handling of API keys and sensitive data

### 5. Observability
- Comprehensive logging for debugging and monitoring
- Metrics collection for performance monitoring
- Error tracking and alerting

## Data Models

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

### Model Relationships
- Users have many Notes, Highlights, and Bookmarks
- Chapters contain many Lessons
- Lessons contain many LessonSections
- EmbeddingDocuments link to specific LessonSections
- RAGQueries and ChatHistory track user interactions

## API Requirements

### Ingestion APIs (ADMIN ONLY)
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

## Content Management

CRUD for:
- Modules
- Chapters
- Lessons
- Lesson sections
- Glossary
- Exercises
- Media assets

All content stored in Markdown/MDX with frontmatter.

## User Personalization
- Save notes, bookmarks, highlights
- Track progress per lesson
- Sync across devices

## Admin Panel Support (API only)
- Upload lesson files
- Trigger ingestion
- Check ingestion logs
- Rebuild embeddings button

## Developer Experience

Backend must include:
- Perfect folder structure for FastAPI
- Environment variable templates
- Local dev using Uvicorn + hot reload
- Integration guide for Docusaurus:
  - Chatbot widget
  - "Ask Questions About This Lesson" button
  - Selected-text → RAG API call workflow using free LLM

## Quality Assurance

### Code Quality Gates
- 90% test coverage minimum
- Static type checking (mypy)
- Linting (flake8, black)
- Security scanning (bandit)

### Performance Requirements
- API response times under 2 seconds for 95th percentile
- Support 1000+ concurrent users
- Efficient embedding queries (sub-500ms response)

### Security Requirements
- All API keys stored in environment variables
- Input validation on all endpoints
- SQL injection prevention
- Rate limiting to prevent abuse

## Governance
This Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All Pull Requests (PRs) and reviews must verify compliance. Complexity must be justified.

**Version**: 2.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06