# RAG Chatbot Quick Start Checklist

Follow this checklist to deploy the RAG chatbot from scratch.

## ☐ Phase 1: Service Setup (15 minutes)

### 1.1 Cohere API
- [ ] Go to https://dashboard.cohere.com/api-keys
- [ ] Sign up / Log in
- [ ] Create new API key
- [ ] Copy and save the key

### 1.2 Qdrant Cloud
- [ ] Go to https://cloud.qdrant.io/
- [ ] Sign up / Log in
- [ ] Create new cluster (Free tier)
- [ ] Copy cluster URL (format: `https://xxx.qdrant.io`)
- [ ] Copy API key from cluster settings

### 1.3 Neon Postgres
- [ ] Go to https://neon.tech/
- [ ] Sign up / Log in
- [ ] Create new project
- [ ] Copy connection string
- [ ] Open SQL Editor
- [ ] Run schema from `backend/app/db/schema.sql`

### 1.4 Railway (or Render)
- [ ] Go to https://railway.app/
- [ ] Sign up / Log in with GitHub
- [ ] Ready to deploy in Phase 2

## ☐ Phase 2: Backend Deployment (20 minutes)

### 2.1 Push Code to GitHub
```bash
git add backend/
git commit -m "Add RAG chatbot backend"
git push origin main
```

### 2.2 Deploy to Railway

**Option A: Railway Dashboard**
- [ ] Click "New Project"
- [ ] Select "Deploy from GitHub repo"
- [ ] Choose your repository
- [ ] Set root directory: `backend`
- [ ] Add environment variables (see below)
- [ ] Wait for deployment
- [ ] Generate domain in Settings

**Option B: Railway CLI**
```bash
npm i -g @railway/cli
railway login
cd backend
railway init
railway variables set COHERE_API_KEY=xxx
railway variables set QDRANT_URL=xxx
railway variables set QDRANT_API_KEY=xxx
railway variables set NEON_DATABASE_URL=xxx
railway variables set FRONTEND_URL=https://physical-ai-textbook.vercel.app
railway up
railway domain
```

### 2.3 Environment Variables
Set these in Railway:
- [ ] `COHERE_API_KEY` = Your Cohere API key
- [ ] `QDRANT_URL` = Your Qdrant cluster URL
- [ ] `QDRANT_API_KEY` = Your Qdrant API key
- [ ] `NEON_DATABASE_URL` = Your Neon connection string
- [ ] `FRONTEND_URL` = `https://physical-ai-textbook.vercel.app`
- [ ] `QDRANT_COLLECTION_NAME` = `physical_ai_textbook`
- [ ] `ENVIRONMENT` = `production`

### 2.4 Verify Backend
- [ ] Copy your Railway URL (e.g., `https://xxx.railway.app`)
- [ ] Test health: `curl https://xxx.railway.app/api/health`
- [ ] Should return: `{"status":"healthy","service":"Physical AI RAG Backend"}`

## ☐ Phase 3: Content Ingestion (10 minutes)

### 3.1 Setup Local Environment
```bash
cd backend
cp .env.example .env
# Edit .env with your credentials
```

### 3.2 Install Dependencies
```bash
pip install -r requirements.txt
```

### 3.3 Run Ingestion Script
```bash
python scripts/ingest_content.py
```

Expected output:
```
🚀 Starting content ingestion...
📦 Creating Qdrant collection...
📚 Found 11 chapters
📖 Parsing Chapter 1: 01-physical-ai-intro
   ✓ Created 8 chunks
...
📊 Total chunks: 87
🔮 Generating embeddings...
✅ Ingestion complete!
```

### 3.4 Verify Ingestion
- [ ] Check Qdrant dashboard: Should show ~80-100 vectors
- [ ] Test search: `curl -X POST https://xxx.railway.app/api/chat/query -H "Content-Type: application/json" -d '{"query":"What is Physical AI?"}'`

## ☐ Phase 4: Frontend Integration (10 minutes)

### 4.1 Install Dependencies
```bash
npm install
```

### 4.2 Add Backend URL to Vercel
```bash
vercel env add NEXT_PUBLIC_API_URL
# Enter: https://xxx.railway.app
# Select: Production, Preview, Development
```

Or in Vercel Dashboard:
- [ ] Go to Project Settings → Environment Variables
- [ ] Add `NEXT_PUBLIC_API_URL` = `https://xxx.railway.app`
- [ ] Apply to: Production, Preview, Development

### 4.3 Deploy Frontend
```bash
git add .
git commit -m "Add RAG chatbot frontend integration"
git push origin main
```

Or manually:
```bash
vercel --prod
```

## ☐ Phase 5: Testing (5 minutes)

### 5.1 Backend Tests
- [ ] Health check: `curl https://xxx.railway.app/api/health`
- [ ] Detailed health: `curl https://xxx.railway.app/api/health/detailed`
- [ ] Query test: `curl -X POST https://xxx.railway.app/api/chat/query -H "Content-Type: application/json" -d '{"query":"What is Physical AI?"}'`

### 5.2 Frontend Tests
- [ ] Visit your deployed site
- [ ] Chat button appears in bottom-right corner
- [ ] Click chat button → Panel opens
- [ ] Type "What is Physical AI?" → Send
- [ ] Response appears with sources
- [ ] Click source link → Navigates to chapter
- [ ] Test text selection:
  - [ ] Select text on a chapter page
  - [ ] Open chat
  - [ ] See "Selected: ..." context
  - [ ] Ask question about selection
- [ ] Test dark mode toggle
- [ ] Test on mobile device

### 5.3 Integration Tests
- [ ] Multi-turn conversation:
  - [ ] Ask: "What is Physical AI?"
  - [ ] Follow-up: "How does it differ from traditional AI?"
  - [ ] Verify context is maintained
- [ ] Code question: "Show me a ROS 2 publisher example"
- [ ] Out-of-scope: "What's the weather?" → Should say not in textbook

## ☐ Phase 6: Monitoring Setup (5 minutes)

### 6.1 Railway Monitoring
- [ ] Check Railway dashboard for metrics
- [ ] Set up log alerts if needed
- [ ] Monitor resource usage

### 6.2 Cohere Usage
- [ ] Check Cohere dashboard for API usage
- [ ] Set up billing alerts
- [ ] Monitor costs

### 6.3 Database Monitoring
- [ ] Check Qdrant dashboard for vector count
- [ ] Check Neon dashboard for storage usage
- [ ] Verify within free tier limits

## 📊 Success Criteria

All of these should be true:
- ✅ Backend health check returns 200
- ✅ Qdrant has ~80-100 vectors
- ✅ Postgres has conversations and messages tables
- ✅ Frontend chat button visible
- ✅ Can send message and get response
- ✅ Sources are clickable and navigate correctly
- ✅ Text selection feature works
- ✅ Dark mode works
- ✅ Mobile responsive

## 🎯 Total Time: ~60 minutes

- Service setup: 15 min
- Backend deployment: 20 min
- Content ingestion: 10 min
- Frontend integration: 10 min
- Testing: 5 min
- Monitoring: 5 min

## 🆘 Troubleshooting

### Backend won't start
```bash
railway logs
# Check for missing environment variables
```

### No search results
```bash
# Re-run ingestion
python scripts/ingest_content.py
```

### Frontend can't reach backend
- Verify `NEXT_PUBLIC_API_URL` is set in Vercel
- Check CORS settings in backend
- Ensure backend URL is accessible

### Costs too high
- Check Cohere usage dashboard
- Consider caching frequent queries
- Implement rate limiting

## 📚 Documentation

- Full deployment guide: `RAG-DEPLOYMENT-GUIDE.md`
- Implementation summary: `RAG-IMPLEMENTATION-SUMMARY.md`
- Backend README: `backend/README.md`
- API docs: `https://xxx.railway.app/docs`

## 🎉 You're Done!

Your RAG chatbot is now live and ready to help students learn about Physical AI!
