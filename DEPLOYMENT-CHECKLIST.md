# Vercel Deployment Checklist

## ✅ Pre-Deployment Tasks

### 1. Code Preparation
- [ ] All changes committed to Git
- [ ] No console.log or debug code
- [ ] All TypeScript errors resolved
- [ ] Build tested locally (`npm run build`)
- [ ] All tests passing

### 2. Environment Variables
- [ ] Backend `.env.example` created
- [ ] Frontend `.env.example` created
- [ ] Cohere API key ready
- [ ] All sensitive data in `.env` (not committed)

### 3. Configuration Files
- [ ] Root `vercel.json` created
- [ ] Backend `vercel.json` created
- [ ] Frontend `vercel.json` created
- [ ] `.gitignore` updated with `.vercel`

### 4. Dependencies
- [ ] All dependencies in `package.json`
- [ ] No missing peer dependencies
- [ ] Node version specified (18.x)
- [ ] Lock files committed

### 5. Build Verification
```bash
# Test Docusaurus build
npm run build

# Test Backend
cd backend && npm install && npm run dev

# Test Frontend
cd frontend && npm install && npm run build
```

## 🚀 Deployment Steps

### Option A: Vercel CLI (Recommended)

#### 1. Install Vercel CLI
```bash
npm install -g vercel
```

#### 2. Login
```bash
vercel login
```

#### 3. Deploy Main Site
```bash
vercel --prod
```

#### 4. Deploy Backend
```bash
cd backend
vercel --prod
# Note the deployment URL
```

#### 5. Deploy Frontend
```bash
cd frontend
# Update VITE_API_URL with backend URL
vercel env add VITE_API_URL production
vercel --prod
```

### Option B: Vercel Dashboard

1. **Main Site**
   - Import repository
   - Root directory: `/`
   - Framework: Docusaurus
   - Build: `npm run build`
   - Output: `build`

2. **Backend**
   - Import same repository
   - Root directory: `backend`
   - Add env: `COHERE_API_KEY`
   - Deploy

3. **Frontend**
   - Import same repository
   - Root directory: `frontend`
   - Framework: Vite
   - Add env: `VITE_API_URL` (backend URL)
   - Deploy

## 🔐 Environment Variables Setup

### Backend (Vercel Dashboard)
```
COHERE_API_KEY=your_key_here
NODE_ENV=production
```

### Frontend (Vercel Dashboard)
```
VITE_API_URL=https://your-backend.vercel.app
```

## ✅ Post-Deployment Verification

### 1. Check Deployments
- [ ] Main site loads: `https://your-site.vercel.app`
- [ ] Backend API responds: `https://your-backend.vercel.app/api/chat`
- [ ] Frontend loads: `https://your-frontend.vercel.app`

### 2. Test Features
- [ ] Theme selector works
- [ ] Chat functionality works
- [ ] All pages load correctly
- [ ] Mobile responsive
- [ ] No console errors

### 3. Performance
- [ ] Lighthouse score > 90
- [ ] Images optimized
- [ ] Fast page load
- [ ] No broken links

## 🐛 Common Issues

### Build Fails
- Check Node version (18.x)
- Verify all dependencies installed
- Clear Vercel cache
- Check build logs

### API Not Connecting
- Verify VITE_API_URL is correct
- Check CORS settings
- Verify backend deployed successfully

### Environment Variables
- Redeploy after adding variables
- Check variable names (case-sensitive)
- Verify in Vercel dashboard

## 📊 Monitoring

### Vercel Dashboard
- Check deployment logs
- Monitor function invocations
- Review analytics
- Check error rates

### URLs to Monitor
```
Main Site: https://physical-ai-textbook.vercel.app
Backend: https://physical-ai-backend.vercel.app
Frontend: https://physical-ai-frontend.vercel.app
```

## 🎯 Final Checklist

- [ ] All three projects deployed
- [ ] Environment variables set
- [ ] URLs updated in frontend
- [ ] All features tested
- [ ] No errors in console
- [ ] Mobile tested
- [ ] Performance verified
- [ ] Documentation updated

---

**Status: Ready for Deployment** ✅

Run `./deploy.sh` or use Vercel Dashboard to deploy!
