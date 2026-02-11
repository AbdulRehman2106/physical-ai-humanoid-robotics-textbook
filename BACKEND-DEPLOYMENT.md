# Backend Deployment Guide - Vercel

## Step 1: Backend Ko Vercel Par Deploy Karein

### 1.1 Backend Directory Ko Separate Repository Banayein (Optional)

Agar aap backend ko alag deploy karna chahte hain:

```bash
cd backend
git init
git add .
git commit -m "Initial backend commit"
```

### 1.2 Vercel Par Backend Deploy Karein

1. **Vercel Dashboard** par jaayein: https://vercel.com
2. **"Add New Project"** click karein
3. **Backend folder** select karein ya separate repo import karein
4. **Environment Variables** add karein:
   - `COHERE_API_KEY`: Your Cohere API key
   - `NODE_ENV`: production
   - `ALLOWED_ORIGINS`: Your frontend URL (comma-separated if multiple)

### 1.3 Backend URL Note Karein

Deploy hone ke baad aapko ek URL milega jaise:
```
https://your-backend-name.vercel.app
```

## Step 2: Frontend Code Update Karein

Backend deploy hone ke baad, frontend code mein backend URL update karein:

### 2.1 Update `src/pages/chat.tsx`

```typescript
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://your-backend-name.vercel.app/api'  // ← Yahan apna backend URL dalein
  : 'http://localhost:3001/api';
```

### 2.2 Update `src/components/Chatbot/index.tsx`

```typescript
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://your-backend-name.vercel.app/api'  // ← Yahan apna backend URL dalein
  : 'http://localhost:3001/api';
```

## Step 3: Frontend Ko Redeploy Karein

```bash
git add .
git commit -m "Update backend URL for production"
git push origin main
```

Vercel automatically redeploy kar dega.

## Step 4: Test Karein

1. Apni Vercel site kholen
2. Chat page par jaayein
3. Message bhej kar test karein

## Troubleshooting

### CORS Error
Agar CORS error aaye to backend ke `server.ts` mein apna frontend URL add karein:

```typescript
origin: [
  'https://your-frontend-url.vercel.app',
  'http://localhost:3000'
]
```

### Environment Variables Missing
Vercel dashboard mein jaake Environment Variables check karein:
- Settings → Environment Variables
- `COHERE_API_KEY` properly set hai ya nahi

### Backend Not Responding
- Vercel logs check karein: Deployments → Your deployment → Logs
- Health endpoint test karein: `https://your-backend-url.vercel.app/api/health`

## Alternative: Backend Ko Same Project Mein Deploy Karein

Agar aap backend ko same Vercel project mein deploy karna chahte hain:

1. Root `vercel.json` update karein
2. Backend routes ko `/api/*` par map karein
3. Serverless functions use karein

Is approach ke liye alag guide chahiye to batayein!
