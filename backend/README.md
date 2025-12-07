# Physical AI & Humanoid Robotics Book Platform Backend

## Overview

This is a comprehensive backend system for the Physical AI & Humanoid Robotics book platform, featuring:

- FastAPI-based backend with async architecture
- RAG (Retrieval Augmented Generation) chatbot with Qdrant vector search
- Neon serverless Postgres for primary database
- Document ingestion pipeline supporting Markdown, HTML, PDF, and text files
- Free tier API limits and resource management
- Optional user authentication and personalization features

## Architecture

The backend follows a clean architecture with clear separation of concerns:

- **API Layer**: FastAPI endpoints with request/response validation
- **Service Layer**: Business logic and cross-cutting concerns
- **Database Layer**: Database operations and CRUD operations
- **Core Layer**: Core functionality (embeddings, vector store, LLM client)

## Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Set up environment variables (copy `.env.example` to `.env` and fill in your values)

## Configuration

The application uses environment variables for configuration. Copy `.env.example` to `.env` and set the following:

- `DATABASE_URL`: Connection string for Neon Postgres database
- `QDRANT_URL`: URL for Qdrant vector store
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `OPENAI_API_KEY`: API key for OpenAI (optional, for embeddings)
- `HUGGINGFACE_API_KEY`: API key for Hugging Face (optional)
- `OLLAMA_BASE_URL`: Base URL for Ollama (default: http://localhost:11434)
- `JWT_SECRET_KEY`: Secret key for JWT tokens
- Various other settings for rate limiting, resource management, etc.

## Running the Application

To run the application in development mode:

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## API Documentation

Interactive API documentation is available at:
- Swagger UI: `http://localhost:8000/api/v1/docs`
- ReDoc: `http://localhost:8000/api/v1/redoc`

## Features

### Document Ingestion
- Admin-only upload of various file formats (Markdown, HTML, PDF, text)
- Automatic content chunking and metadata extraction
- Embedding generation and storage in Qdrant vector store
- Ingestion job tracking and status monitoring

### RAG Chatbot
- Semantic search against ingested content
- Context-aware AI responses using local models (Ollama/Hugging Face)
- Hallucination prevention with strict source referencing
- Support for selected-text → RAG API workflow

### Content Management
- CRUD operations for books, chapters, lessons, and sections
- Markdown/MDX content storage with frontmatter
- Progress tracking per user per lesson

### User Management (Optional)
- Optional JWT-based authentication
- Personalization features (notes, highlights, bookmarks)
- API usage tracking and rate limiting

### Resource Management
- Free tier with enforced limits (30 queries/day, 50 search/day, etc.)
- Rate limiting middleware (100 requests/minute per user)
- Usage monitoring and reporting

## Technologies Used

- **FastAPI**: Modern, fast web framework for building APIs
- **SQLAlchemy**: SQL toolkit and ORM for database operations
- **Qdrant**: Vector similarity search engine
- **Neon**: Serverless Postgres database
- **Ollama/Hugging Face**: Free LLM models for RAG functionality
- **Pydantic**: Data validation and settings management
- **AsyncIO**: Asynchronous programming for high concurrency

## License

This project is licensed under the terms specified in the project constitution.

## Contributing

Please refer to the project constitution for contribution guidelines.