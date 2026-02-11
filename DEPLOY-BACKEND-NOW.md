# Backend Deployment - Quick Start

## Option 1: Backend Ko Vercel Par Deploy Karein (Recommended)

### Step 1: Backend Ko GitHub Par Push Karein

```bash
cd backend
git init
git add .
git commit -m "Initial backend setup for Vercel"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-backend.git
git push -u origin main
```

### Step 2: Vercel Par Deploy Karein

1. **Vercel Dashboard** kholen: https://vercel.com/new
2. **Import Git Repository** click karein
3. Backend repository select karein
4. **Configure Project:**
   - Framework Preset: Other
   - Root Directory: `./` (backend folder)
   - Build Command: `npm run build`
   - Output Directory: `dist`
   - Install Command: `npm install`

5. **Environment Variables** add karein:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   NODE_ENV=production
   ALLOWED_ORIGINS=https://physical-ai-humanoid-robotics-textb-nine-neon.vercel.app
   PORT=3001
   LOG_LEVEL=info
   ```

6. **Deploy** button click karein

### Step 3: Backend URL Copy Karein

Deploy hone ke baad URL milega jaise:
```
https://physical-ai-backend-xyz.vercel.app
```

## Option 2: Backend Ko Same Project Mein Integrate Karein (Easier)

Agar aap alag repository nahi banana chahte, to backend ko same project mein API routes ke through deploy kar sakte hain.

### Kya aap chahte hain:
- **Option 1**: Backend ko alag Vercel project mein deploy karein (better for scaling)
- **Option 2**: Backend ko same project mein integrate karein (easier setup)

Batayein kaunsa option prefer karte hain?
