---
title: Physical AI & Humanoid Robotics Book Platform
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_file: app.py
pinned: false
---

# Physical AI & Humanoid Robotics Book Platform Backend

This is a comprehensive backend system for the Physical AI & Humanoid Robotics book platform, featuring:

- FastAPI-based backend with async architecture
- RAG (Retrieval Augmented Generation) chatbot with Qdrant vector search
- Translation functionality with MyMemory → OpenRouter → Gemini → OpenAI fallback
- Neon serverless Postgres for primary database
- Document ingestion pipeline supporting Markdown, HTML, PDF, and text files
- Free tier API limits and resource management
- Optional user authentication and personalization features

## Features

### RAG Chatbot
- Semantic search against ingested content
- Context-aware AI responses using configurable LLM clients (OpenRouter > OpenAI > Gemini fallback)
- Hallucination prevention with strict source referencing
- Support for selected-text → RAG API workflow

### Translation Service
- **MyMemory** as primary translation service (free, handles Unicode/Urdu text)
- **OpenRouter** as first fallback for translation
- **Google Gemini** as second fallback for translation
- **OpenAI** as final fallback for translation
- Proper URL encoding for Unicode characters

### Content Management
- CRUD operations for books, chapters, lessons, and sections
- Markdown/MDX content storage with frontmatter
- Progress tracking per user per lesson

### User Management (Optional)
- Optional JWT-based authentication
- Personalization features (notes, highlights, bookmarks)
- API usage tracking and rate limiting

## Configuration

This Space requires the following environment variables to be set:

- `OPENAI_API_KEY`: API key for OpenAI (optional, for embeddings and fallback)
- `GOOGLE_GEMINI_API_KEY`: API key for Google Gemini (optional, for embeddings and fallback)
- `OPENROUTER_API_KEY`: API key for OpenRouter (for translation fallback)
- `QDRANT_HOST`: URL for Qdrant vector store
- `DATABASE_URL`: Connection string for database
- `JWT_SECRET_KEY`: Secret key for JWT tokens

## Technologies Used

- **FastAPI**: Modern, fast web framework for building APIs
- **SQLAlchemy**: SQL toolkit and ORM for database operations
- **Qdrant**: Vector similarity search engine
- **OpenRouter/OpenAI/Google Gemini**: LLM models for RAG and translation
- **Pydantic**: Data validation and settings management
- **AsyncIO**: Asynchronous programming for high concurrency

## API Documentation

Interactive API documentation is available at:
- Swagger UI: `/api/v1/docs`
- ReDoc: `/api/v1/redoc`

## License

This project is licensed under the terms specified in the project constitution.