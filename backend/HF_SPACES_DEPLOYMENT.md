# Hugging Face Spaces Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics Book Platform on Hugging Face Spaces.

## Requirements for Hugging Face Spaces

For optimal performance on Hugging Face Spaces, use the minimal requirements file:

```
fastapi==0.104.1
uvicorn[standard]==0.24.0
sqlalchemy==2.0.23
pydantic==2.5.0
pydantic-settings==2.1.0
qdrant-client>=1.7.0
httpx==0.25.2
python-multipart==0.0.6
passlib[bcrypt]==1.7.4
python-jose[cryptography]==3.3.0
python-dotenv==1.0.0
openai>=1.0.0
google-generativeai>=0.4.0
requests>=2.31.0
aiofiles>=23.0.0
PyJWT>=2.8.0
aiosqlite>=0.19.0
```

## Key Differences from Full Version

1. **sentence-transformers excluded**: To reduce memory usage and startup time
2. **cohere excluded**: Not currently used in the codebase
3. **beautifulsoup4 excluded**: Not currently used in the codebase

## Environment Variables Required

For full functionality, set these environment variables in your Hugging Face Space:

- `OPENAI_API_KEY` (recommended) - For embeddings and chat functionality
- `GOOGLE_GEMINI_API_KEY` (recommended) - For Gemini chat functionality
- `OPENROUTER_API_KEY` - For translation fallback
- `DATABASE_URL` - For database connection
- `QDRANT_HOST` - For vector store connection

## Fallback Behavior

When sentence-transformers is not available:
1. The system will try OpenAI embeddings first
2. Then Gemini embeddings
3. Local embeddings will be skipped if sentence-transformers is not installed

## Docker Configuration

The Dockerfile should use the Hugging Face optimized requirements:

```dockerfile
FROM python:3.10-slim

WORKDIR /app

COPY requirements_hf_spaces.txt .
RUN pip install --no-cache-dir -r requirements_hf_spaces.txt

COPY ./src ./src

EXPOSE 7860

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

## Performance Considerations

- Without sentence-transformers, the system relies on API calls which may have rate limits
- Ensure your API keys have sufficient quotas for your expected usage
- The system will function but may be slower without local embedding generation