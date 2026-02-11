# 🚀 Vercel Deployment Guide - Complete Step by Step

## 📋 Overview
Aap 3 separate projects deploy karenge:
1. **Main Docusaurus Site** (Documentation)
2. **Frontend App** (React + Vite Chatbot UI)
3. **Backend API** (Express + Cohere AI)

---

## 🎯 Step 1: GitHub Repository Tayyar Karna

### 1.1 Files Push Karna
```bash
cd "F:\MY WORK\NEW PROJECTS\GOVERNER HOUSE\HACKATHON\Q4_Hackathone\Physical-Ai-Text-Book"
git add .
git commit -m "Add Vercel deployment configurations"
git push origin main
```

✅ **Verify:** GitHub par jaa kar check karein ke saari files push ho gayi hain.

---

## 🌐 Step 2: Vercel Account Setup

### 2.1 Vercel Account Banana
1. **Website:** https://vercel.com
2. **Sign Up with GitHub** button par click karein
3. GitHub se authorize karein
4. Email verify karein

### 2.2 GitHub Repository Connect Karna 
1. Vercel Dashboard par jaayein
2. **"Add New..."** → **"Project"** select karein
3. **"Import Git Repository"** section mein apni repository search karein:
   - Repository name: `physical-ai-humanoid-robotics-textbook`
4. **"Import"** button par click karein

---

## 📚 Step 3: Main Docusaurus Site Deploy Karna

### 3.1 Project Configuration
1. **Project Name:** `physical-ai-textbook` (ya apni pasand ka naam)
2. **Framework Preset:** Docusaurus (auto-detect hoga)
3. **Root Directory:** `./` (root folder)
4. **Build Command:** `npm run build`
5. **Output Directory:** `build`
6. **Install Command:** `npm install`

### 3.2 Environment Variables (Optional)
- Is project ke liye koi environment variable nahi chahiye

### 3.3 Deploy Karna
1. **"Deploy"** button par click karein
2. Wait karein (2-3 minutes)
3. ✅ **Success!** Aapko URL milega jaise: `https://physical-ai-textbook.vercel.app`

---

## 💬 Step 4: Frontend App Deploy Karna

### 4.1 Naya Project Banana
1. Vercel Dashboard par wapas jaayein
2. **"Add New..."** → **"Project"** select karein
3. Same repository select karein: `physical-ai-humanoid-robotics-textbook`
4. **"Import"** button par click karein

### 4.2 Project Configuration
1. **Project Name:** `physical-ai-frontend`
2. **Framework Preset:** Vite
3. **Root Directory:** `frontend` ⚠️ **IMPORTANT: Yeh change karna zaroori hai!**
   - "Edit" button par click karein
   - `frontend` folder select karein
4. **Build Command:** `npm run build`
5. **Output Directory:** `dist`
6. **Install Command:** `npm install`

### 4.3 Environment Variables Setup
⚠️ **IMPORTANT:** Backend URL add karna zaroori hai!

1. **"Environment Variables"** section mein jaayein
2. Add karein:
   - **Name:** `VITE_API_URL`
   - **Value:** `https://physical-ai-backend.vercel.app` (Step 5 ke baad update karenge)
   - **Environment:** Production, Preview, Development (sab select karein)

### 4.4 Deploy Karna
1. **"Deploy"** button par click karein
2. Wait karein (2-3 minutes)
3. ✅ **Success!** URL milega: `https://physical-ai-frontend.vercel.app`

---

## 🔧 Step 5: Backend API Deploy Karna

### 5.1 Naya Project Banana
1. Vercel Dashboard par wapas jaayein
2. **"Add New..."** → **"Project"** select karein
3. Same repository select karein
4. **"Import"** button par click karein

### 5.2 Project Configuration
1. **Project Name:** `physical-ai-backend`
2. **Framework Preset:** Other (ya Node.js)
3. **Root Directory:** `backend` ⚠️ **IMPORTANT!**
   - "Edit" button par click karein
   - `backend` folder select karein
4. **Build Command:** `npm run build`
5. **Output Directory:** `dist`
6. **Install Command:** `npm install`

### 5.3 Environment Variables Setup
⚠️ **CRITICAL:** Yeh variables zaroori hain!

1. **"Environment Variables"** section mein jaayein
2. Add karein:

   **Variable 1:**
   - **Name:** `COHERE_API_KEY`
   - **Value:** `your_actual_cohere_api_key_here` (apni real API key dalein)
   - **Environment:** Production, Preview, Development

   **Variable 2:**
   - **Name:** `NODE_ENV`
   - **Value:** `production`
   - **Environment:** Production

   **Variable 3:**
   - **Name:** `FRONTEND_URL`
   - **Value:** `https://physical-ai-frontend.vercel.app`
   - **Environment:** Production, Preview, Development

### 5.4 Deploy Karna
1. **"Deploy"** button par click karein
2. Wait karein (2-3 minutes)
3. ✅ **Success!** URL milega: `https://physical-ai-backend.vercel.app`

---

## 🔄 Step 6: Frontend Environment Variable Update Karna

⚠️ **IMPORTANT:** Ab backend ka URL mil gaya hai, toh frontend update karna hoga!

### 6.1 Frontend Project Mein Jaayein
1. Vercel Dashboard → **Projects** → `physical-ai-frontend`
2. **"Settings"** tab par click karein
3. **"Environment Variables"** section mein jaayein

### 6.2 VITE_API_URL Update Karna
1. `VITE_API_URL` variable ko edit karein
2. **New Value:** `https://physical-ai-backend.vercel.app` (apna actual backend URL)
3. **Save** karein

### 6.3 Redeploy Karna
1. **"Deployments"** tab par jaayein
2. Latest deployment par **"..."** (three dots) click karein
3. **"Redeploy"** select karein
4. Wait karein (1-2 minutes)

---

## ✅ Step 7: Testing Karna

### 7.1 Main Docusaurus Site Test
1. URL open karein: `https://physical-ai-textbook.vercel.app`
2. Check karein:
   - ✅ Homepage load ho raha hai
   - ✅ Navigation kaam kar raha hai
   - ✅ Chapters accessible hain

### 7.2 Frontend App Test
1. URL open karein: `https://physical-ai-frontend.vercel.app`
2. Check karein:
   - ✅ UI load ho raha hai
   - ✅ Chat interface visible hai
   - ✅ Message send kar sakte hain

### 7.3 Backend API Test
1. Browser mein open karein: `https://physical-ai-backend.vercel.app/health`
2. Response aana chahiye:
   ```json
   {
     "status": "healthy",
     "timestamp": "2024-02-10T..."
   }
   ```

### 7.4 End-to-End Test
1. Frontend app mein message type karein
2. Send button press karein
3. ✅ AI response aana chahiye

---

## 🔧 Troubleshooting

### Problem 1: Frontend Backend Se Connect Nahi Ho Raha
**Solution:**
1. Frontend environment variables check karein
2. Backend CORS settings verify karein
3. Browser console mein errors check karein

### Problem 2: Backend API Key Error
**Solution:**
1. Vercel Dashboard → Backend Project → Settings → Environment Variables
2. `COHERE_API_KEY` verify karein
3. Redeploy karein

### Problem 3: Build Failed
**Solution:**
1. Deployment logs check karein
2. Root Directory setting verify karein
3. Build command correct hai ya nahi check karein

---

## 📝 Important URLs (Save Kar Lein)

After deployment, aapke paas 3 URLs honge:

1. **Documentation Site:** `https://physical-ai-textbook.vercel.app`
2. **Frontend App:** `https://physical-ai-frontend.vercel.app`
3. **Backend API:** `https://physical-ai-backend.vercel.app`

---

## 🎉 Deployment Complete!

Congratulations! Aapka complete application ab live hai Vercel par!

### Next Steps:
1. ✅ Custom domain add kar sakte hain (optional)
2. ✅ Analytics enable kar sakte hain
3. ✅ Automatic deployments on git push (already enabled)

---

## 📞 Support

Agar koi problem aaye toh:
1. Vercel deployment logs check karein
2. GitHub repository issues section mein post karein
3. Vercel documentation: https://vercel.com/docs

---

**Happy Deploying! 🚀**
