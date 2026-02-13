# Physical AI RAG Backend

FastAPI backend for the Physical AI textbook RAG chatbot.

## Features

- **RAG Pipeline**: Retrieval-Augmented Generation using Cohere API
- **Vector Search**: Qdrant for semantic search
- **Conversation Storage**: Neon Postgres for chat history
- **Text Selection Context**: Support for querying with selected text

## Tech Stack

- FastAPI (Python 3.11+)
- Cohere API (embeddings + generation)
- Qdrant Cloud (vector database)
- Neon Serverless Postgres (conversation storage)

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
```

Required environment variables:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Qdrant cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `NEON_DATABASE_URL`: Neon Postgres connection string
- `FRONTEND_URL`: Frontend URL for CORS

### 3. Setup Database

Run the schema on your Neon database:

```bash
psql $NEON_DATABASE_URL < app/db/schema.sql
```

### 4. Ingest Content

Parse MDX files and upload to Qdrant:

```bash
python scripts/ingest_content.py
```

This will:
- Parse all 11 chapters from `docs/chapters/`
- Create ~80-100 semantic chunks
- Generate embeddings via Cohere
- Upload to Qdrant

### 5. Run Server

```bash
uvicorn app.main:app --reload --port 8000
```

API will be available at `http://localhost:8000`

## API Endpoints

### Chat

**POST /api/chat/query**
```json
{
  "query": "What is Physical AI?",
  "conversation_id": "uuid-optional",
  "filters": { "chapter": 1 }
}
```

**POST /api/chat/query-with-context**
```json
{
  "query": "Explain this",
  "selected_text": "Physical AI systems...",
  "selection_metadata": {
    "chapter_title": "Introduction",
    "url": "/docs/chapters/physical-ai-intro"
  }
}
```

**POST /api/chat/conversations**
Create a new conversation.

**GET /api/chat/conversations/{id}**
Get conversation with all messages.

### Health

**GET /api/health**
Basic health check.

**GET /api/health/detailed**
Detailed health check with database status.

## Deployment

### Railway (Recommended)

1. Create Railway project
2. Connect GitHub repo
3. Set environment variables
4. Deploy command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Render

1. Create new Web Service
2. Connect GitHub repo
3. Build command: `pip install -r requirements.txt`
4. Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

## Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI app
│   ├── config.py            # Settings
│   ├── models/
│   │   ├── chat.py         # Chat models
│   │   └── document.py     # Document models
│   ├── services/
│   │   ├── embeddings.py   # Cohere embeddings
│   │   ├── generation.py   # Cohere generation
│   │   ├── retrieval.py    # Qdrant search
│   │   └── rag_pipeline.py # Main RAG logic
│   ├── db/
│   │   ├── postgres.py     # Neon client
│   │   ├── qdrant.py       # Qdrant client
│   │   └── schema.sql      # Database schema
│   └── api/
│       └── routes/
│           ├── chat.py     # Chat endpoints
│           └── health.py   # Health endpoints
├── scripts/
│   └── ingest_content.py   # Content ingestion
└── requirements.txt
```

## Development

Run with auto-reload:
```bash
uvicorn app.main:app --reload
```

View API docs:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Cost Estimate

- Cohere: ~$5-10/month (moderate usage)
- Qdrant Cloud: Free (1GB tier)
- Neon Postgres: Free tier
- Railway: Free (500 hours/month)

**Total: ~$5-10/month**
