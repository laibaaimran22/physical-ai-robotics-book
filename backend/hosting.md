# Deploying on Hugging Face Spaces

This guide will help you deploy the Physical AI & Humanoid Robotics backend on Hugging Face Spaces.

## Quick Deploy

[![Duplicate Space](https://huggingface.co/datasets/huggingface/badges/raw/main/duplicate-space-sm.svg)](https://huggingface.co/new-space?template=)

## Manual Deployment Steps

1. Fork this repository or create a copy
2. Create a new Space on Hugging Face with the following settings:
   - **Repository**: Your forked repository
   - **Space SDK**: Docker
   - **Hardware**: Choose based on your needs (CPU is sufficient for basic usage)
   - **Visibility**: Public or Private as per your preference

3. Add the required environment variables in the Space settings:
   - `GEMINI_API_KEY` - Your Google Gemini API key
   - `QDRANT_URL` - Your Qdrant vector database URL
   - `QDRANT_API_KEY` - Your Qdrant API key
   - `DATABASE_URL` - Your database connection string
   - `ENVIRONMENT` - Set to "production"

4. The Space will automatically build and deploy using the provided Dockerfile

## Environment Variables

Make sure to set these secrets in your Hugging Face Space settings:

```bash
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_database_url
ENVIRONMENT=production
```

## API Endpoints

Once deployed, your API will be available at:
- `https://your-username-space-name.hf.space/chat` - Chat endpoint
- `https://your-username-space-name.hf.space/docs` - API documentation

## Notes

- The application uses FastAPI and will automatically generate API documentation
- For production use, ensure you have proper API keys and database connections
- The chatbot functionality requires a valid GEMINI_API_KEY to work