# Vercel Deployment Guide - Physical AI Textbook

## 📋 Pre-Deployment Checklist

### ✅ Files Created
- [x] `/vercel.json` - Main Docusaurus site configuration
- [x] `/backend/vercel.json` - Backend API configuration
- [x] `/frontend/vercel.json` - Frontend React app configuration
- [x] `.env.example` - Environment variables template
- [x] `DEPLOYMENT.md` - This deployment guide

### ✅ Required Environment Variables

#### Backend (.env)
```bash
COHERE_API_KEY=your_cohere_api_key_here
NODE_ENV=production
PORT=3000
```

#### Frontend (.env)
```bash
VITE_API_URL=https://your-backend-url.vercel.app
```

## 🚀 Deployment Steps

### Option 1: Deploy via Vercel CLI

#### 1. Install Vercel CLI
```bash
npm install -g vercel
```

#### 2. Login to Vercel
```bash
vercel login
```

#### 3. Deploy Main Docusaurus Site
```bash
# From project root
vercel --prod
```

#### 4. Deploy Backend API
```bash
cd backend
vercel --prod
```

#### 5. Deploy Frontend
```bash
cd frontend
vercel --prod
```

### Option 2: Deploy via Vercel Dashboard

#### 1. Main Docusaurus Site
1. Go to https://vercel.com/new
2. Import Git repository
3. Select root directory
4. Framework Preset: Docusaurus
5. Build Command: `npm run build`
6. Output Directory: `build`
7. Click "Deploy"

#### 2. Backend API
1. Create new project in Vercel
2. Import same repository
3. Root Directory: `backend`
4. Framework Preset: Other
5. Build Command: `npm run build` (if needed)
6. Add Environment Variables:
   - `COHERE_API_KEY`
   - `NODE_ENV=production`
7. Click "Deploy"

#### 3. Frontend
1. Create new project in Vercel
2. Import same repository
3. Root Directory: `frontend`
4. Framework Preset: Vite
5. Build Command: `npm run build`
6. Output Directory: `dist`
7. Add Environment Variable:
   - `VITE_API_URL` (use backend URL from step 2)
8. Click "Deploy"

## 🔐 Environment Variables Setup

### In Vercel Dashboard:
1. Go to Project Settings
2. Navigate to "Environment Variables"
3. Add each variable:
   - Name: `COHERE_API_KEY`
   - Value: Your API key
   - Environment: Production, Preview, Development

### Using Vercel CLI:
```bash
# Add environment variable
vercel env add COHERE_API_KEY production

# Pull environment variables
vercel env pull
```

## 🔧 Build Configuration

### Docusaurus (Main Site)
- **Framework**: Docusaurus
- **Build Command**: `npm run build`
- **Output Directory**: `build`
- **Node Version**: 18.x

### Backend API
- **Framework**: Node.js
- **Build Command**: None (serverless)
- **Entry Point**: `src/api/server.ts`
- **Node Version**: 18.x

### Frontend
- **Framework**: Vite
- **Build Command**: `npm run build`
- **Output Directory**: `dist`
- **Node Version**: 18.x

## 🌐 Custom Domain Setup (Optional)

### Add Custom Domain:
1. Go to Project Settings → Domains
2. Add your domain (e.g., `physical-ai-textbook.com`)
3. Configure DNS records as instructed
4. Wait for SSL certificate provisioning

### DNS Configuration:
```
Type: A
Name: @
Value: 76.76.21.21

Type: CNAME
Name: www
Value: cname.vercel-dns.com
```

## 📊 Post-Deployment Verification

### Check Deployments:
```bash
# List all deployments
vercel ls

# Check deployment logs
vercel logs [deployment-url]
```

### Test URLs:
- Main Site: `https://physical-ai-textbook.vercel.app`
- Backend API: `https://physical-ai-backend.vercel.app/api/chat`
- Frontend: `https://physical-ai-frontend.vercel.app`

## 🔄 Continuous Deployment

### Automatic Deployments:
- **Production**: Pushes to `main` branch
- **Preview**: Pull requests and other branches
- **Development**: Local development with `vercel dev`

### Branch Configuration:
```bash
# Set production branch
vercel --prod --scope your-team

# Deploy specific branch
vercel --target production
```

## 🐛 Troubleshooting

### Build Failures:
1. Check build logs in Vercel dashboard
2. Verify all dependencies are in `package.json`
3. Ensure Node version matches (18.x)
4. Clear build cache: Settings → General → Clear Cache

### Environment Variables:
1. Verify all required variables are set
2. Check variable names (case-sensitive)
3. Redeploy after adding variables

### API Connection Issues:
1. Verify backend URL in frontend `.env`
2. Check CORS settings in backend
3. Ensure API routes are correct

## 📝 Important Notes

### Before Deploying:
- ✅ Test build locally: `npm run build`
- ✅ Verify all environment variables
- ✅ Check `.gitignore` excludes sensitive files
- ✅ Update API URLs in frontend
- ✅ Test all features locally

### Security:
- ❌ Never commit `.env` files
- ✅ Use Vercel environment variables
- ✅ Enable HTTPS only
- ✅ Set up proper CORS policies

### Performance:
- ✅ Enable caching headers
- ✅ Optimize images before deployment
- ✅ Minify CSS/JS (automatic with Vite/Docusaurus)
- ✅ Use CDN for static assets

## 🎯 Deployment Checklist

- [ ] All code committed to Git
- [ ] Environment variables documented
- [ ] Build tested locally
- [ ] vercel.json files created
- [ ] .gitignore updated
- [ ] Dependencies up to date
- [ ] API endpoints tested
- [ ] Frontend connects to backend
- [ ] Theme system working
- [ ] Chat functionality working
- [ ] Mobile responsive verified
- [ ] SEO meta tags added
- [ ] Analytics configured (optional)

## 📞 Support

### Vercel Documentation:
- https://vercel.com/docs
- https://vercel.com/docs/frameworks/docusaurus
- https://vercel.com/docs/frameworks/vite

### Project Repository:
- GitHub: [Your Repository URL]
- Issues: [Your Issues URL]

---

**Ready to Deploy!** 🚀

When you're ready, run:
```bash
vercel --prod
```

Or use the Vercel Dashboard for GUI deployment.
