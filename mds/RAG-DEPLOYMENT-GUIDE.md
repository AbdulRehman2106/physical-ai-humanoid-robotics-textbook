# RAG Chatbot Deployment Guide

This guide walks you through deploying the RAG chatbot backend and integrating it with the frontend.

## Prerequisites

- Cohere API key ([Get one here](https://dashboard.cohere.com/api-keys))
- Qdrant Cloud account ([Sign up](https://cloud.qdrant.io/))
- Neon Postgres account ([Sign up](https://neon.tech/))
- Railway or Render account for backend hosting

## Step 1: Set Up Qdrant Cloud

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a new cluster (Free tier: 1GB)
3. Note your cluster URL and API key
4. The collection will be created automatically during content ingestion

## Step 2: Set Up Neon Postgres

1. Go to [Neon](https://neon.tech/)
2. Create a new project
3. Copy the connection string
4. Run the database schema:

```bash
# Using psql
psql "your-neon-connection-string" < backend/app/db/schema.sql

# Or using the Neon SQL Editor in the dashboard
# Copy and paste the contents of backend/app/db/schema.sql
```

## Step 3: Deploy Backend to Railway

### Option A: Using Railway CLI

1. Install Railway CLI:
```bash
npm i -g @railway/cli
```

2. Login and initialize:
```bash
railway login
cd backend
railway init
```

3. Set environment variables:
```bash
railway variables set COHERE_API_KEY=your_cohere_key
railway variables set QDRANT_URL=your_qdrant_url
railway variables set QDRANT_API_KEY=your_qdrant_key
railway variables set NEON_DATABASE_URL=your_neon_connection_string
railway variables set FRONTEND_URL=https://physical-ai-textbook.vercel.app
railway variables set QDRANT_COLLECTION_NAME=physical_ai_textbook
railway variables set ENVIRONMENT=production
```

4. Deploy:
```bash
railway up
```

5. Get your backend URL:
```bash
railway domain
```

### Option B: Using Railway Dashboard

1. Go to [Railway](https://railway.app/)
2. Click "New Project" → "Deploy from GitHub repo"
3. Select your repository
4. Set root directory to `backend`
5. Add environment variables in the Variables tab:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `NEON_DATABASE_URL`
   - `FRONTEND_URL`
   - `QDRANT_COLLECTION_NAME`
   - `ENVIRONMENT`
6. Railway will auto-detect Python and deploy
7. Generate a domain in the Settings tab

## Step 4: Ingest Content

After backend is deployed, run the ingestion script locally:

1. Create a local `.env` file in the `backend` directory:
```bash
cd backend
cp .env.example .env
# Edit .env with your credentials
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Run ingestion:
```bash
python scripts/ingest_content.py
```

This will:
- Parse all 11 chapters (~80-100 chunks)
- Generate embeddings via Cohere
- Upload to Qdrant
- Takes ~5-10 minutes

## Step 5: Update Frontend Environment

1. Add backend URL to Vercel:

```bash
# Using Vercel CLI
vercel env add NEXT_PUBLIC_API_URL

# Or in Vercel Dashboard:
# Settings → Environment Variables → Add
# Name: NEXT_PUBLIC_API_URL
# Value: https://your-backend.railway.app
```

2. Redeploy frontend:
```bash
vercel --prod
```

## Step 6: Verify Deployment

### Backend Health Check

```bash
curl https://your-backend.railway.app/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "Physical AI RAG Backend"
}
```

### Detailed Health Check

```bash
curl https://your-backend.railway.app/api/health/detailed
```

Should show Qdrant and Postgres as healthy.

### Test Chat Query

```bash
curl -X POST https://your-backend.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?"
  }'
```

### Frontend Test

1. Visit your deployed site
2. Click the chat button (💬) in bottom-right
3. Ask: "What is Physical AI?"
4. Should receive answer with source citations

## Alternative: Deploy to Render

If you prefer Render over Railway:

1. Go to [Render](https://render.com/)
2. Create new Web Service
3. Connect GitHub repo
4. Settings:
   - Root Directory: `backend`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as Railway)
6. Deploy

## Troubleshooting

### Backend won't start

- Check logs: `railway logs` or Render dashboard
- Verify all environment variables are set
- Ensure Python 3.11+ is being used

### Qdrant connection fails

- Verify cluster URL format: `https://xxx.qdrant.io`
- Check API key is correct
- Ensure cluster is active (not paused)

### Postgres connection fails

- Verify connection string format
- Check Neon project is active
- Ensure schema was created successfully

### Frontend can't reach backend

- Verify CORS settings in `backend/app/main.py`
- Check `NEXT_PUBLIC_API_URL` is set correctly
- Ensure backend URL is accessible (not localhost)

### No search results

- Verify content was ingested: Check Qdrant dashboard
- Run ingestion script again if needed
- Check collection name matches in config

## Cost Monitoring

### Free Tier Limits

- **Cohere**: 100 API calls/minute, pay-as-you-go after
- **Qdrant Cloud**: 1GB storage (enough for ~100k chunks)
- **Neon**: 0.5GB storage, 100 hours compute/month
- **Railway**: 500 hours/month, $5 credit

### Expected Monthly Costs

- Light usage (100 queries/day): ~$5-10/month
- Medium usage (500 queries/day): ~$20-30/month
- Heavy usage (2000 queries/day): ~$50-80/month

Most cost comes from Cohere API usage.

## Monitoring

### Railway Metrics

```bash
railway logs --tail
```

### Check Collection Status

```bash
curl https://your-backend.railway.app/api/health/detailed
```

### View Conversations

Access Neon SQL Editor to query conversations:

```sql
SELECT COUNT(*) FROM conversations;
SELECT COUNT(*) FROM messages;
SELECT * FROM messages ORDER BY created_at DESC LIMIT 10;
```

## Next Steps

1. Monitor usage and costs
2. Add rate limiting if needed
3. Implement user feedback collection
4. Add analytics tracking
5. Consider caching frequent queries

## Support

If you encounter issues:

1. Check Railway/Render logs
2. Verify environment variables
3. Test backend endpoints directly
4. Check browser console for frontend errors
5. Review Cohere API usage dashboard
