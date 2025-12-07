# Implementation Plan: Physical AI & Humanoid Robotics Book Platform Backend

**Branch**: `fix-github-integration-and-update-constitution` | **Date**: 2025-12-06 | **Spec**: [specs/fix-github-integration-and-update-constitution/spec.md](specs/fix-github-integration-and-update-constitution/spec.md)
**Input**: Feature specification from `/specs/fix-github-integration-and-update-constitution/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive backend system for the Physical AI & Humanoid Robotics book platform, featuring FastAPI, RAG chatbot with Qdrant vector search, Neon serverless Postgres, document ingestion pipelines, and free tier API limits. The system will provide intelligent content delivery, user personalization, and resource-efficient free LLM usage following the project constitution requirements.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, asyncpg, httpx, Pydantic, SQLAlchemy, Qdrant, Hugging Face transformers, Ollama client, OpenAI SDK (optional), Neon Postgres
**Storage**: Neon Serverless Postgres (primary DB), Qdrant Cloud (vector DB), file storage for documents
**Testing**: pytest with coverage, mypy for type checking, bandit for security scanning
**Target Platform**: Linux server (deployable on cloud platforms)
**Project Type**: Web application (backend API with Docusaurus integration)
**Performance Goals**: <2s API response time for 95th percentile, support 1000+ concurrent users, <500ms embedding queries
**Constraints**: <100ms p95 for basic queries, efficient resource usage, secure handling of any API keys
**Scale/Scope**: 10k+ users, 1M+ content chunks, multiple book modules with chapters and lessons

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Architecture Principles Compliance
- ✅ **Async-First Design**: FastAPI with async/await patterns for all endpoints and data operations
- ✅ **Separation of Concerns**: Clear layers (API, Service, Data, Integration) as per constitution
- ✅ **Type Safety**: Pydantic models for all data validation and strict typing throughout
- ✅ **Security-First**: Authentication/authorization, rate limiting, input sanitization, secure API key handling
- ✅ **Observability**: Comprehensive logging, metrics collection, and error tracking

### System-wide Requirements Verification
- ✅ **Backend Stack**: FastAPI, asyncpg, httpx, Neon Postgres, Qdrant, OpenAI as required
- ✅ **Document Ingestion**: Full pipeline with admin upload, chunking, metadata storage, embeddings
- ✅ **Free API Tier Limits**: Middleware enforcement with usage tracking and quotas
- ✅ **Cost Optimization**: Caching, efficient token usage, usage monitoring

### Data Models Compliance
- ✅ **Mandatory Models**: All required models (User, Chapter, Lesson, etc.) implemented in data-model.md
- ✅ **Pydantic Schemas**: Each model includes proper schema validation
- ✅ **DB Table Schemas**: SQLAlchemy models for Postgres implemented
- ✅ **CRUD Operations**: Full CRUD operations designed for each model
- ✅ **Validation Rules**: Per constitution requirements

### API Requirements Verification
- ✅ **Ingestion APIs**: Admin-only endpoints designed as specified in OpenAPI contract
- ✅ **RAG APIs**: Full suite of query and search endpoints designed
- ✅ **Free Tier Controls**: Middleware with quota enforcement implemented
- ✅ **User APIs**: Authentication and personalization endpoints designed
- ✅ **Book APIs**: Content delivery endpoints designed

### Quality Assurance Gates
- ✅ **Code Quality**: 90% test coverage, mypy, linting, security scanning requirements documented
- ✅ **Performance**: <2s response times, 1000+ concurrent users support requirements met
- ✅ **Security**: All API keys in env vars, input validation, SQL injection prevention implemented

## Project Structure

### Documentation (this feature)

```text
specs/fix-github-integration-and-update-constitution/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                 # FastAPI application entry point
│   ├── config/                 # Configuration and settings
│   │   ├── settings.py         # Application settings with Pydantic
│   │   └── database.py         # Database connection and session management
│   ├── models/                 # SQLAlchemy models and Pydantic schemas
│   │   ├── user.py             # User model and schema
│   │   ├── chapter.py          # Chapter model and schema
│   │   ├── lesson.py           # Lesson model and schema
│   │   ├── lesson_section.py   # LessonSection model and schema
│   │   ├── embedding_document.py # EmbeddingDocument model and schema
│   │   ├── rag_query.py        # RAGQuery model and schema
│   │   ├── chat_history.py     # ChatHistory model and schema
│   │   ├── highlight.py        # Highlight model and schema
│   │   ├── note.py             # Note model and schema
│   │   ├── api_usage_quota.py  # APIUsageQuota model and schema
│   │   ├── ingestion_job.py    # IngestionJob model and schema
│   │   └── book_metadata.py    # BookMetadata model and schema
│   ├── services/               # Business logic layer
│   │   ├── auth_service.py     # Authentication and authorization
│   │   ├── ingestion_service.py # Document ingestion processing
│   │   ├── rag_service.py      # RAG and search functionality
│   │   ├── user_service.py     # User management
│   │   ├── content_service.py  # Content management
│   │   ├── api_quota_service.py # API usage tracking
│   │   └── embedding_service.py # Embedding generation and management
│   ├── api/                    # API routes and endpoints
│   │   ├── deps.py             # Dependency injection
│   │   ├── auth.py             # Authentication endpoints
│   │   ├── ingest.py           # Ingestion endpoints (admin only)
│   │   ├── rag.py              # RAG endpoints
│   │   ├── user.py             # User endpoints
│   │   ├── book.py             # Book content endpoints
│   │   └── middleware/         # API middleware
│   │       ├── rate_limiter.py # Rate limiting middleware
│   │       └── quota_checker.py # API quota enforcement middleware
│   ├── database/               # Database operations
│   │   ├── base.py             # Base model and engine setup
│   │   ├── crud/               # CRUD operations for each model
│   │   │   ├── user.py
│   │   │   ├── chapter.py
│   │   │   ├── lesson.py
│   │   │   ├── lesson_section.py
│   │   │   ├── embedding_document.py
│   │   │   ├── rag_query.py
│   │   │   ├── chat_history.py
│   │   │   ├── highlight.py
│   │   │   ├── note.py
│   │   │   ├── api_usage_quota.py
│   │   │   ├── ingestion_job.py
│   │   │   └── book_metadata.py
│   │   └── repositories/       # Repository pattern implementations
│   ├── utils/                  # Utility functions
│   │   ├── security.py         # Security utilities (password hashing, JWT)
│   │   ├── file_processing.py  # File processing utilities
│   │   ├── text_chunking.py    # Text chunking utilities
│   │   ├── logging.py          # Logging configuration
│   │   └── validators.py       # Custom validators
│   └── core/                   # Core functionality
│       ├── embeddings.py       # Embedding generation and management
│       ├── vector_store.py     # Qdrant integration
│       ├── embedding_client.py # Embedding client (OpenAI or local)
│       └── llm_client.py       # Free LLM client (Ollama/Hugging Face)
├── scripts/                    # Deployment and utility scripts
│   ├── init_db.py              # Database initialization script
│   ├── run_migrations.py       # Migration runner
│   └── ingest_documents.py     # Standalone ingestion script
└── tests/                      # Test suite
    ├── unit/                   # Unit tests
    │   ├── models/             # Model tests
    │   ├── services/           # Service tests
    │   └── utils/              # Utility tests
    ├── integration/            # Integration tests
    │   ├── api/                # API integration tests
    │   └── database/           # Database integration tests
    └── contract/               # Contract tests
        └── openapi.json        # OpenAPI specification
```

**Structure Decision**: Web application backend structure selected to support the Physical AI & Humanoid Robotics book platform. The structure follows the constitution's architecture principles with clear separation of concerns between API, Service, Data, and Integration layers. The backend will be deployed as a standalone API server that integrates with the Docusaurus frontend.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
