# Physical AI Textbook - RAG Chatbot

## 🎯 Overview

This project includes a complete RAG (Retrieval-Augmented Generation) chatbot system that helps students learn from the Physical AI textbook by answering questions based on the content.

## 🏗️ Architecture

```
┌─────────────────┐
│   Frontend      │
│  (Docusaurus)   │
│   + ChatBot     │
└────────┬────────┘
         │ HTTPS
         ▼
┌─────────────────┐
│  FastAPI        │
│  Backend        │
│  (Railway)      │
└────┬────────┬───┘
     │        │
     ▼        ▼
┌─────────┐ ┌──────────┐
│ Qdrant  │ │  Neon    │
│ Vectors │ │ Postgres │
└─────────┘ └──────────┘
     ▲
     │
┌────┴─────┐
│  Cohere  │
│   API    │
└──────────┘
```

## 📦 What's Included

### Backend (`backend/`)
- FastAPI server with RAG pipeline
- Cohere integration (embeddings + generation)
- Qdrant vector database client
- Neon Postgres for conversation storage
- Content ingestion script
- Health check endpoints

### Frontend (`src/`)
- ChatBot React component
- Text selection capture
- API client with TypeScript types
- Responsive design with dark mode

### Documentation
- `RAG-QUICK-START.md` - Step-by-step deployment checklist
- `RAG-DEPLOYMENT-GUIDE.md` - Detailed deployment instructions
- `RAG-IMPLEMENTATION-SUMMARY.md` - Technical implementation details
- `backend/README.md` - Backend-specific documentation

## 🚀 Quick Start

### Prerequisites
- Node.js 18+
- Python 3.11+
- Cohere API key
- Qdrant Cloud account
- Neon Postgres account
- Railway or Render account

### 1. Install Dependencies

**Frontend:**
```bash
npm install
```

**Backend:**
```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment

**Backend (.env):**
```bash
cd backend
cp .env.example .env
# Edit .env with your credentials
```

**Frontend (Vercel):**
```bash
vercel env add NEXT_PUBLIC_API_URL
# Enter your backend URL
```

### 3. Deploy Backend

**Using Railway:**
```bash
cd backend
railway login
railway init
railway up
railway domain
```

**Or using Render:**
- Connect GitHub repo
- Set root directory to `backend`
- Add environment variables
- Deploy

### 4. Ingest Content

```bash
cd backend
python scripts/ingest_content.py
```

This will parse all 11 chapters and upload ~80-100 chunks to Qdrant.

### 5. Deploy Frontend

```bash
npm run build
vercel --prod
```

### 6. Test

Visit your deployed site and click the chat button (💬) in the bottom-right corner.

## 📚 Features

### Core Functionality
- ✅ Natural language Q&A about textbook content
- ✅ Semantic search across all chapters
- ✅ Source citations with clickable links
- ✅ Multi-turn conversations with context
- ✅ Text selection for contextual queries

### User Experience
- ✅ Floating chat widget
- ✅ Expandable panel (400×600px)
- ✅ Loading states and error handling
- ✅ Dark mode support
- ✅ Mobile responsive
- ✅ Keyboard shortcuts

## 🧪 Testing

### Backend Tests
```bash
cd backend
./test-backend.sh
```

Or manually:
```bash
# Health check
curl https://your-backend.railway.app/api/health

# Test query
curl -X POST https://your-backend.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is Physical AI?"}'
```

### Frontend Tests
1. Visit deployed site
2. Click chat button
3. Ask: "What is Physical AI?"
4. Verify response with sources
5. Test text selection feature
6. Test dark mode
7. Test on mobile

## 💰 Cost Estimate

**Free Tier:**
- Cohere: ~$5-10/month (moderate usage)
- Qdrant Cloud: $0 (1GB free tier)
- Neon Postgres: $0 (free tier)
- Railway: $0 (500 hours/month)

**Total: ~$5-10/month**

## 📖 API Documentation

Once backend is deployed, visit:
- Swagger UI: `https://your-backend.railway.app/docs`
- ReDoc: `https://your-backend.railway.app/redoc`

### Key Endpoints

**POST /api/chat/query**
```json
{
  "query": "What is Physical AI?",
  "conversation_id": "optional-uuid",
  "filters": { "chapter": 1 }
}
```

**POST /api/chat/query-with-context**
```json
{
  "query": "Explain this",
  "selected_text": "Physical AI systems...",
  "conversation_id": "optional-uuid"
}
```

**GET /api/health**
Basic health check.

**GET /api/health/detailed**
Detailed health with database status.

## 🔧 Development

### Run Backend Locally
```bash
cd backend
uvicorn app.main:app --reload
```

### Run Frontend Locally
```bash
npm start
```

### Environment Variables

**Backend:**
- `COHERE_API_KEY` - Cohere API key
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DATABASE_URL` - Neon connection string
- `FRONTEND_URL` - Frontend URL for CORS
- `QDRANT_COLLECTION_NAME` - Collection name (default: physical_ai_textbook)
- `ENVIRONMENT` - Environment (development/production)

**Frontend:**
- `NEXT_PUBLIC_API_URL` - Backend API URL

## 📊 Monitoring

### Railway Logs
```bash
railway logs --tail
```

### Check Collection Status
```bash
curl https://your-backend.railway.app/api/health/detailed
```

### Database Queries
```sql
-- Check conversation count
SELECT COUNT(*) FROM conversations;

-- Check message count
SELECT COUNT(*) FROM messages;

-- Recent messages
SELECT * FROM messages ORDER BY created_at DESC LIMIT 10;
```

## 🆘 Troubleshooting

### Backend won't start
- Check Railway logs
- Verify all environment variables are set
- Ensure Python 3.11+ is being used

### No search results
- Verify content was ingested
- Check Qdrant dashboard for vectors
- Re-run ingestion script if needed

### Frontend can't reach backend
- Verify `NEXT_PUBLIC_API_URL` is set
- Check CORS settings in backend
- Ensure backend URL is accessible

### High costs
- Monitor Cohere usage dashboard
- Implement rate limiting
- Consider caching frequent queries

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is part of the Physical AI Textbook educational resource.

## 🙏 Acknowledgments

- Cohere for AI capabilities
- Qdrant for vector search
- Neon for serverless Postgres
- Railway for easy deployment

## 📞 Support

For issues or questions:
1. Check the documentation in `RAG-DEPLOYMENT-GUIDE.md`
2. Review the troubleshooting section
3. Check Railway/Render logs
4. Open an issue on GitHub

---

**Ready to deploy?** Follow the [Quick Start Guide](RAG-QUICK-START.md) for step-by-step instructions.
