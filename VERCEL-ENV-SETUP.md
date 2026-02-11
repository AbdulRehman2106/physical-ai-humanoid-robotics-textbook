# Vercel Environment Variables Setup Guide

## Frontend Environment Variables

Vercel dashboard mein ye environment variables add karein:

### Step 1: Vercel Dashboard Kholen

1. https://vercel.com/dashboard par jaayein
2. **physical-ai-humanoid-robotics-textbook** project kholen (frontend)
3. **Settings** tab click karein
4. Left sidebar mein **Environment Variables** select karein

### Step 2: Environment Variable Add Karein

**Variable 1: Backend URL**
```
Name: BACKEND_URL
Value: https://physical-ai-backend-green.vercel.app/api
Environment: Production, Preview, Development (sab select karein)
```

### Step 3: Redeploy Karein

Environment variables add karne ke baad:
1. **Deployments** tab par jaayein
2. Latest deployment par **"Redeploy"** click karein
3. Ya phir koi bhi change push karein GitHub par

## Local Development Ke Liye

Local development mein `.env.local` file banayein (root directory mein):

```bash
# .env.local
BACKEND_URL=http://localhost:3001/api
```

**Note:** `.env.local` file `.gitignore` mein already hai, commit nahi hogi.

## Verification

Environment variable sahi se set hai ya nahi check karne ke liye:
1. Vercel dashboard → Project → Settings → Environment Variables
2. `BACKEND_URL` variable dikhna chahiye

## Current Status

✅ Code already updated hai environment variable use karne ke liye
⏳ Abhi sirf Vercel dashboard mein variable add karna hai
✅ Redeploy hone ke baad sab kaam karega

---

**Next Step:** Vercel dashboard mein jaake `BACKEND_URL` environment variable add karein!
