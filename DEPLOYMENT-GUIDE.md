# Deployment Guide - Physical AI Textbook

## Architecture Overview

- **Frontend**: Docusaurus site deployed on Vercel
- **Backend**: Python FastAPI + RAG pipeline deployed on Railway

## Step 1: Deploy Backend on Railway

### 1.1 Create Railway Account
1. Go to [railway.app](https://railway.app)
2. Sign up with GitHub

### 1.2 Deploy Backend
1. Click "New Project" → "Deploy from GitHub repo"
2. Select your repository
3. Railway will auto-detect the `backend/` directory
4. Set the following environment variables in Railway dashboard:

```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_textbook
NEON_DATABASE_URL=your_neon_postgres_url
FRONTEND_URL=https://your-vercel-domain.vercel.app
ENVIRONMENT=production
```

### 1.3 Get Backend URL
- After deployment, Railway will provide a URL like: `https://your-app.railway.app`
- Copy this URL - you'll need it for frontend configuration

## Step 2: Update Frontend Configuration

### 2.1 Update Backend URL
Edit `static/config.js` and replace `your-backend-url.railway.app` with your actual Railway URL:

```javascript
window.BACKEND_URL = window.location.hostname === 'localhost'
  ? 'http://localhost:8001'
  : 'https://your-actual-backend.railway.app';
```

### 2.2 Commit Changes
```bash
git add static/config.js
git commit -m "Update backend URL for production"
git push origin main
```

## Step 3: Deploy Frontend on Vercel

### 3.1 Import Project
1. Go to [vercel.com](https://vercel.com)
2. Click "Add New" → "Project"
3. Import your GitHub repository
4. Vercel will auto-detect Docusaurus

### 3.2 Configure Build Settings
Vercel should auto-detect these settings:
- **Framework Preset**: Docusaurus
- **Build Command**: `npm run build`
- **Output Directory**: `build`
- **Install Command**: `npm install`

### 3.3 Deploy
Click "Deploy" - Vercel will build and deploy your site

## Step 4: Configure CORS (Important!)

After both deployments, update the backend CORS settings:

1. Go to your Railway backend deployment
2. Update the `FRONTEND_URL` environment variable with your actual Vercel URL
3. Redeploy the backend

## Step 5: Test Deployment

1. Visit your Vercel URL
2. Open the chatbot
3. Ask a question to test the RAG pipeline
4. Check browser console for any errors

## Environment Variables Summary

### Backend (Railway)
```
COHERE_API_KEY=<your-key>
QDRANT_URL=<your-url>
QDRANT_API_KEY=<your-key>
QDRANT_COLLECTION_NAME=physical_ai_textbook
NEON_DATABASE_URL=<your-url>
FRONTEND_URL=<your-vercel-url>
ENVIRONMENT=production
```

### Frontend (Vercel)
No environment variables needed - backend URL is configured in `static/config.js`

## Troubleshooting

### Chatbot not connecting
- Check browser console for CORS errors
- Verify backend URL in `static/config.js`
- Ensure `FRONTEND_URL` is set correctly in Railway

### Backend errors
- Check Railway logs
- Verify all environment variables are set
- Ensure Qdrant collection exists and has data

### Build failures
- Check Node.js version (should be 18+)
- Clear Vercel cache and redeploy
- Check build logs for specific errors

## Quick Commands

```bash
# Test backend locally
cd backend
python -m uvicorn app.main:app --reload

# Test frontend locally
npm start

# Build frontend
npm run build

# Serve production build locally
npm run serve
```

## Support

For issues, check:
- Railway logs: Railway dashboard → Deployments → Logs
- Vercel logs: Vercel dashboard → Deployments → Build Logs
- Browser console: F12 → Console tab
