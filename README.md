# Physical AI & Humanoid Robotics Book Platform

A comprehensive educational platform featuring a Retrieval-Augmented Generation (RAG) chatbot integrated with the Physical AI & Humanoid Robotics book content.

## 🚀 Features

- **RAG Chatbot**: Ask questions about the book content and get AI-powered answers
- **User-Selected Text Queries**: Ask questions specifically about highlighted text
- **Semantic Search**: Find relevant content using vector search
- **Document Ingestion**: Add new content to the knowledge base
- **Free Tier Limits**: API usage limits for free users
- **Neon Serverless Postgres**: Scalable database backend
- **Qdrant Vector Storage**: Cloud-based semantic search

## 🛠️ Tech Stack

- **Backend**: FastAPI (Python)
- **Frontend**: Docusaurus (React-based documentation)
- **Database**: Neon Serverless Postgres
- **Vector Store**: Qdrant Cloud
- **LLM**: OpenAI/Cohere integration
- **Embeddings**: text-embedding-3-small

## 📁 Project Structure

```
├── backend/                 # FastAPI backend
│   ├── src/
│   │   ├── api/            # API routes
│   │   ├── services/       # Business logic
│   │   ├── models/         # Database models
│   │   ├── core/           # Core utilities (LLM, embeddings, vector store)
│   │   └── config/         # Configuration
├── physical-ai-robotics-book/  # Docusaurus frontend
│   ├── src/
│   │   └── components/     # React components
│   │       └── RAGChatbot/ # Chatbot component
│   └── docs/               # Book content
└── README.md
```

## 🚦 Getting Started

### Prerequisites

- Python 3.8+
- Node.js 18+

### Backend Setup

1. Install Python dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Configure your environment variables in `.env`:
   ```env
   # Database Configuration - Neon Serverless Postgres
   DATABASE_URL=postgresql://neondb_owner:npg_WCvOq9HsUAJ1@ep-misty-morning-ad36vr3e-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

   # Qdrant Configuration - Qdrant Cloud Free Tier
   QDRANT_URL=https://649cf293-7c71-4831-89a5-fda46a3f47cd.us-east4-0.gcp.cloud.qdrant.io
   QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.xwgQykUtge_6CQ2UTRTDM3KaegYCNOO7Pnm7iAnP_ZQ

   # Cohere API Configuration
   COHERE_API_KEY=4UwVuDtLsu8kjIIseBiPu6oX535pudfF9DgqmAfU

   # OpenAI API Configuration (optional, for fallback)
   OPENAI_API_KEY=your_openai_api_key_here
   ```

3. Start the backend server:
   ```bash
   cd backend
   python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
   ```

### Frontend Setup

1. Install Node.js dependencies:
   ```bash
   cd physical-ai-robotics-book
   npm install
   ```

2. Start the frontend:
   ```bash
   npm start
   ```

## 🤖 API Endpoints

- `GET /` - Main API information
- `GET /health` - Health check
- `POST /api/v1/rag/query` - RAG query endpoint
- `POST /api/v1/rag/query-selected` - Query with selected text
- `POST /api/v1/rag/search` - Semantic search
- `POST /api/v1/ingest/document` - Document ingestion
- `GET /api/v1/docs` - Interactive API documentation

## 📚 Usage

1. Navigate to the frontend at `http://localhost:3001`
2. Use the integrated chatbot to ask questions about the book content
3. Select text in lessons and use the "Ask about Selection" feature
4. Access the API directly at `http://localhost:8000` for programmatic access

## ⚙️ Configuration

### Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `DATABASE_URL` | Neon Postgres connection string | Yes |
| `QDRANT_URL` | Qdrant Cloud endpoint | Yes |
| `QDRANT_API_KEY` | Qdrant Cloud API key | Yes |
| `COHERE_API_KEY` | Cohere API key for embeddings/LLM | Recommended |
| `OPENAI_API_KEY` | OpenAI API key (fallback) | Optional |

### Rate Limits

- Free tier: 30 RAG queries per day per user
- Free tier: 50 search queries per day per user
- General rate limit: 100 requests per minute

## 🧪 Testing

To test the API endpoints:

```bash
# Test health check
curl http://localhost:8000/health

# Test RAG query
curl -X POST http://localhost:8000/api/v1/rag/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "top_k": 5}'
```

## 🚀 Deployment

For production deployment:

1. Update the `.env` with production settings
2. Set `APP_ENV=production`
3. Use proper domain names in `ALLOWED_ORIGINS`
4. Deploy backend to a cloud provider with Neon & Qdrant connectivity
5. Deploy frontend to static hosting (GitHub Pages, Vercel, etc.)

## 📄 License

This project is licensed under the MIT License.