# 🚀 Backend Deployment Guide - Step by Step

## Problem
Aapka frontend Vercel par hai lekin backend `localhost:3001` ko call kar raha hai, jo production mein kaam nahi karega.

## Solution: Backend Ko Vercel Par Deploy Karein

### Step 1: Backend Ko GitHub Repository Mein Push Karein

```bash
# Backend directory mein jaayein
cd backend

# Git initialize karein (agar already nahi hai)
git init

# .gitignore check karein
cat .gitignore

# Sab files add karein
git add .

# Commit karein
git commit -m "Backend setup for Vercel deployment"

# Main branch set karein
git branch -M main

# GitHub par naya repository banayein aur remote add karein
# GitHub par jaake "physical-ai-backend" naam se repository banayein
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-backend.git

# Push karein
git push -u origin main
```

### Step 2: Vercel Par Backend Deploy Karein

1. **Vercel Dashboard** kholen: https://vercel.com/new

2. **Import Git Repository** click karein

3. **Backend repository** select karein (`physical-ai-backend`)

4. **Configure Project:**
   - **Framework Preset**: Other
   - **Root Directory**: `./` (leave as is)
   - **Build Command**: `npm run build`
   - **Output Directory**: Leave empty (serverless function)
   - **Install Command**: `npm install`

5. **Environment Variables** add karein (IMPORTANT!):

   Click "Environment Variables" aur ye add karein:

   ```
   COHERE_API_KEY = your_cohere_api_key_here
   NODE_ENV = production
   ALLOWED_ORIGINS = https://physical-ai-humanoid-robotics-textb-nine-neon.vercel.app
   LOG_LEVEL = info
   MAX_CONTEXT_TOKENS = 100000
   MAX_MESSAGES_PER_CONVERSATION = 1000
   SESSION_TIMEOUT_MS = 1800000
   ```

6. **Deploy** button click karein

7. **Wait** for deployment to complete (2-3 minutes)

### Step 3: Backend URL Copy Karein

Deploy hone ke baad aapko URL milega jaise:
```
https://physical-ai-backend-abc123.vercel.app
```

Is URL ko copy kar lein!

### Step 4: Frontend Code Update Karein

Ab frontend code mein backend URL update karna hai:

#### 4.1 Update `src/pages/chat.tsx`

File kholen aur line 17-19 ko replace karein:

```typescript
// BEFORE:
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://your-backend-url.vercel.app/api'
  : 'http://localhost:3001/api';

// AFTER (apna actual backend URL dalein):
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://physical-ai-backend-abc123.vercel.app/api'  // ← Yahan apna URL
  : 'http://localhost:3001/api';
```

#### 4.2 Update `src/components/Chatbot/index.tsx`

Same change karein line 18-20 mein:

```typescript
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://physical-ai-backend-abc123.vercel.app/api'  // ← Yahan apna URL
  : 'http://localhost:3001/api';
```

### Step 5: Changes Commit Aur Push Karein

```bash
# Main project directory mein wapas jaayein
cd ..

# Changes add karein
git add src/pages/chat.tsx src/components/Chatbot/index.tsx

# Commit karein
git commit -m "Update backend URL for production deployment"

# Push karein
git push origin main
```

Vercel automatically frontend ko redeploy kar dega!

### Step 6: Test Karein

1. 2-3 minutes wait karein (deployment ke liye)
2. Apni Vercel site kholen: `https://physical-ai-humanoid-robotics-textb-nine-neon.vercel.app`
3. Chat page par jaayein: `/chat`
4. Message bhej kar test karein

## Troubleshooting

### ❌ CORS Error Still Coming?

Backend ke `server.ts` mein apna frontend URL check karein:

```typescript
origin: [
  'https://physical-ai-humanoid-robotics-textb-nine-neon.vercel.app',
  'http://localhost:3000'
]
```

Agar URL match nahi kar raha to backend redeploy karein.

### ❌ Backend 404 Error?

Backend URL check karein:
- Health endpoint test karein: `https://your-backend-url.vercel.app/api/health`
- Agar 404 aaye to `backend/vercel.json` check karein

### ❌ Environment Variables Missing?

Vercel dashboard mein:
1. Backend project kholen
2. Settings → Environment Variables
3. Sab variables properly set hain ya nahi check karein
4. Agar missing hain to add karke redeploy karein

## Quick Commands Summary

```bash
# Backend deploy
cd backend
git init
git add .
git commit -m "Backend for Vercel"
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-backend.git
git push -u origin main

# Frontend update
cd ..
# (Update chat.tsx and Chatbot/index.tsx with backend URL)
git add src/
git commit -m "Update backend URL"
git push origin main
```

## Need Help?

Agar koi step clear nahi hai to batayein, main help karunga! 🚀
