# Create Physical AI & Humanoid Robotics Book Platform Specification

Create a complete specification for the Physical AI & Humanoid Robotics book platform backend with all the required components.

## Qdrant Configuration
- **Cluster Endpoint**: https://649cf293-7c71-4831-89a5-fda46a3f47cd.us-east4-0.gcp.cloud.qdrant.io
- **API Key**: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.xwgQykUtge_6CQ2UTRTDM3KaegYCNOO7Pnm7iAnP_ZQ
- **Cluster ID**: 649cf293-7c71-4831-89a5-fda46a3f47cd

## Requirements to Include

### Backend Stack
- FastAPI (Python, fully typed)
- Async architecture with httpx + asyncpg
- Neon Serverless Postgres (primary DB)
- Qdrant Cloud (vector DB) - use the endpoint above
- OpenAI ChatKit/Agents for LLM logic
- Supports Windows + WSL2 development

### Document Ingestion System
- Admin-only upload of Markdown/MDX, HTML, PDF, and raw text
- Content chunking (300-500 tokens)
- Metadata storage in Postgres
- Embedding generation with OpenAI text-embedding-3-small
- Vector storage in Qdrant
- Queue-based processing with retry logic

### API Limits for Free Tier
- Chatbot API: 30 queries/day, 5 RAG queries/minute, 5k tokens/response
- Search API: 50 queries/day
- User data limits: 50 notes, 50 highlights, 10 bookmarks
- Rate limiting: 100 requests/minute per user

### Data Models
- User, Chapter, Lesson, LessonSection, EmbeddingDocument, RAGQuery, ChatHistory, Highlight, Note, APIUsageQuota, IngestionJob, BookMetadata
- Each with Pydantic schemas, DB tables, CRUD operations, and validation

### API Endpoints
- Ingestion APIs (admin only)
- RAG APIs for search and queries
- User management APIs
- Book content APIs
- Middleware for free tier enforcement

### Developer Experience
- FastAPI folder structure
- Environment templates
- Local development with hot reload
- Docusaurus integration guidelines

Generate a complete specification document that follows the project constitution and includes all these requirements.