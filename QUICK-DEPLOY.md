# Quick Deployment Guide

## 🚀 Deploy in 3 Steps

### Step 1: Deploy Backend on Railway

1. Go to [railway.app](https://railway.app) and sign in with GitHub
2. Click "New Project" → "Deploy from GitHub repo"
3. Select this repository
4. Add these environment variables:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   QDRANT_COLLECTION_NAME=physical_ai_textbook
   NEON_DATABASE_URL=your_neon_postgres_url
   FRONTEND_URL=https://your-site.vercel.app
   ENVIRONMENT=production
   ```
5. Copy your Railway backend URL (e.g., `https://your-app.railway.app`)

### Step 2: Update Frontend Config

1. Edit `static/config.js`
2. Replace `your-backend-url.railway.app` with your actual Railway URL
3. Commit and push:
   ```bash
   git add static/config.js
   git commit -m "Update backend URL for production"
   git push origin main
   ```

### Step 3: Deploy Frontend on Vercel

1. Go to [vercel.com](https://vercel.com) and sign in with GitHub
2. Click "Add New" → "Project"
3. Import this repository
4. Vercel will auto-detect Docusaurus settings
5. Click "Deploy"

### Step 4: Update CORS

After both deployments:
1. Go back to Railway
2. Update `FRONTEND_URL` with your actual Vercel URL
3. Redeploy backend

## ✅ Done!

Visit your Vercel URL and test the chatbot.

## 📚 Need More Details?

See [DEPLOYMENT-GUIDE.md](./DEPLOYMENT-GUIDE.md) for complete instructions.

## 🔧 Local Development

```bash
# Start backend
cd backend
python -m uvicorn app.main:app --reload --port 8001

# Start frontend (in another terminal)
npm start
```

## 🆘 Troubleshooting

- **Chatbot not connecting**: Check browser console for CORS errors
- **Backend errors**: Check Railway logs
- **Build failures**: Check Vercel build logs

## 📝 Environment Variables Needed

### Backend (Railway)
- `COHERE_API_KEY` - Get from [cohere.com](https://cohere.com)
- `QDRANT_URL` - Get from [qdrant.io](https://qdrant.io)
- `QDRANT_API_KEY` - From Qdrant dashboard
- `NEON_DATABASE_URL` - Get from [neon.tech](https://neon.tech)
- `FRONTEND_URL` - Your Vercel URL
- `ENVIRONMENT` - Set to `production`

### Frontend (Vercel)
No environment variables needed - backend URL is in `static/config.js`
