# Quickstart Guide: Physical AI & Humanoid Robotics Book Platform Backend with Urdu Translation

## Overview
This guide provides instructions for setting up, running, and using the Physical AI & Humanoid Robotics book platform backend, including the Urdu Translation Feature. The feature allows logged-in users to translate chapter content into Urdu with a single click.

## Prerequisites
- Python 3.11+
- Docker and Docker Compose (for optional containerized setup)
- Access to Google Gemini API (for translation)
- Access to OpenAI API (for embeddings and RAG)
- Access to Qdrant Cloud
- Access to Neon Serverless Postgres
- Docusaurus v2.x or v3.x for frontend
- Node.js 16+ for frontend development

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Copy the `.env` template and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your specific configuration:
```env
# Qdrant Configuration
QDRANT_URL=https://<your-cluster-id>.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=<your-api-key>
QDRANT_CLUSTER_ID=<your-cluster-id>

# Database Configuration
NEON_DATABASE_URL=postgresql://username:password@ep-xxxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# API Configuration
OPENAI_API_KEY=<openai-key-for-embeddings>
GEMINI_API_KEY=<google-gemini-api-key-for-translation>

# Application Configuration
APP_ENV=development
APP_HOST=0.0.0.0
APP_PORT=8000
DEBUG=true

# JWT Configuration
JWT_SECRET_KEY=<generate-a-secure-random-key>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7
```

## Running the Application

### 1. Database Setup
Initialize the database and run migrations:

```bash
python scripts/init_db.py
python scripts/run_migrations.py
```

### 2. Start the Application
```bash
# Using Uvicorn directly
uvicorn backend.src.main:app --host 0.0.0.0 --port 8000 --reload

# Or using the run script if available
python -m backend.src.main
```

The API will be available at `http://localhost:8000`

### 3. API Documentation
Access the automatic API documentation at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Key API Endpoints

### Authentication
- `POST /auth/login` - User login
- `POST /auth/signup` - User registration
- `POST /auth/refresh` - Token refresh

### Ingestion (Admin Only)
- `POST /ingest/upload` - Upload content files
- `POST /ingest/run` - Start ingestion process
- `GET /ingest/status` - Check ingestion status
- `DELETE /ingest/remove/:id` - Remove ingested content

### RAG (Retrieval Augmented Generation)
- `POST /rag/query` - Query the RAG system
- `POST /rag/query-selected` - Query with selected text
- `POST /rag/search` - Search content
- `POST /rag/semantic-search` - Semantic search
- `GET /rag/metadata` - Get RAG metadata

### Translation (New Feature)
- `POST /api/v1/translate` - Translate content to Urdu

### User Management
- `GET /user/notes` - Get user notes
- `POST /user/notes` - Create user note
- `GET /user/highlights` - Get user highlights
- `POST /user/highlights` - Create user highlight
- `GET /user/bookmarks` - Get user bookmarks
- `POST /user/bookmarks` - Create user bookmark
- `GET /user/progress` - Get user progress

### Book Content
- `GET /book/chapters` - Get all chapters
- `GET /book/lessons/:id` - Get specific lesson
- `GET /book/search` - Search book content

## Urdu Translation Feature Setup

### Backend Setup
The translation endpoint is available at:
```
POST /api/v1/translate
```

### Frontend Setup
The following components are available in the frontend:

1. `TranslationButton` - Button component that triggers translation
2. `TranslationWrapper` - Wrapper component that manages content translation
3. `useTranslation` - Hook for translation state management

### Integration with Docusaurus Pages
To add translation functionality to a Docusaurus page, wrap your content with the `TranslationWrapper` component:

```jsx
import TranslationWrapper from './components/TranslationWrapper';

function MyPage() {
  return (
    <div>
      <TranslationWrapper>
        {/* Your page content here */}
      </TranslationWrapper>
    </div>
  );
}
```

## Document Ingestion Process

### 1. Prepare Content
Ensure your content files are in one of these formats:
- Markdown (.md, .mdx)
- HTML (.html, .htm)
- PDF (.pdf)
- Plain text (.txt)

### 2. Upload Content (Admin Required)
Use the ingestion API to upload your content files:

```bash
curl -X POST "http://localhost:8000/ingest/upload" \
  -H "Authorization: Bearer <admin-token>" \
  -F "file=@path/to/your/content.md"
```

### 3. Start Ingestion
Trigger the ingestion process:

```bash
curl -X POST "http://localhost:8000/ingest/run" \
  -H "Authorization: Bearer <admin-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "file_path": "/path/to/uploaded/file.md",
    "file_type": "markdown"
  }'
```

### 4. Monitor Progress
Check the ingestion status:

```bash
curl -X GET "http://localhost:8000/ingest/status" \
  -H "Authorization: Bearer <admin-token>"
```

## Free Tier API Limits

The platform enforces the following limits for free tier users:

- **Chatbot API**: 30 queries/day, 5 RAG queries/minute, max 5k tokens per response
- **Search API**: 50 search queries/day
- **Translation API**: Included in the general rate limit (100 requests/minute per user)
- **User Data Storage**: 50 notes, 50 highlights, 10 bookmarks
- **Rate Limits**: 100 requests/minute per user

## Translation Feature Usage

### For End Users
1. Navigate to any chapter page in the book
2. Click the "Translate to Urdu" button at the top of the page
3. Wait for the translation to complete (progress indicator shown)
4. The page content will be replaced with Urdu translation
5. Use the "Back to English" button to restore the original content

### API Documentation for Translation

#### Translation Request
```
POST /api/v1/translate
```

**Request Body:**
```json
{
  "content": "The English content to translate",
  "source_language": "en",
  "target_language": "ur"
}
```

**Response:**
```json
{
  "translated_content": "مترجم کردہ مواد یہاں ہوگا",
  "source_content": "The original English content",
  "source_language": "en",
  "target_language": "ur",
  "success": true,
  "error_message": null
}
```

## Testing

Run the test suite:

```bash
# Unit tests
pytest tests/unit/ -v

# Integration tests
pytest tests/integration/ -v

# All tests with coverage
pytest --cov=backend --cov-report=html
```

## Development

### Code Structure
The backend follows a layered architecture:
- `api/` - API routes and endpoints
- `api/translation.py` - Translation API endpoints (new)
- `services/` - Business logic
- `models/` - Data models and schemas
- `database/` - Database operations (CRUD, repositories)
- `utils/` - Utility functions
- `core/` - Core functionality (embeddings, vector store, OpenAI client)

### Frontend Structure
- `src/components/TranslationButton/` - Translation button component
- `src/components/TranslationWrapper/` - Content wrapper for translation
- `src/hooks/useTranslation.ts` - Translation state management hook

### Adding New Models
1. Create the SQLAlchemy model in `models/`
2. Create the corresponding Pydantic schemas
3. Add CRUD operations in `database/crud/`
4. Create service functions in `services/`
5. Add API endpoints in `api/`

## Deployment

### Docker Deployment
Build and run with Docker:

```bash
docker build -t physical-ai-backend .
docker run -p 8000:8000 --env-file .env physical-ai-backend
```

### Production Considerations
- Use a production WSGI server like Gunicorn
- Set `APP_ENV=production` and `DEBUG=false`
- Implement proper logging and monitoring
- Set up a reverse proxy (nginx, Apache)
- Configure SSL certificates
- Use a load balancer for scaling
- Monitor API usage for translation service costs