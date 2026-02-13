# Hugging Face Backend Update Guide

## Option 1: Manual Update (Easiest)

1. Go to: https://huggingface.co/spaces/abdul18/rag-chatbot
2. Click on **"Files and versions"** tab
3. Navigate to `app/main.py`
4. Click **"Edit"** button
5. Update the CORS section with this code:

```python
# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        settings.frontend_url,
        "http://localhost:3000",
        "https://*.vercel.app",
        "https://physical-ai-textbook-wine.vercel.app",  # Production Vercel URL
        "https://abdul18-rag-chatbot.hf.space",  # Hugging Face Spaces
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

6. Click **"Commit changes to main"**
7. Wait 1-2 minutes for rebuild

## Option 2: Git Push (Advanced)

If you have your Hugging Face Space connected to Git:

```bash
# Clone your space
git clone https://huggingface.co/spaces/abdul18/rag-chatbot

# Copy updated backend files
cp -r backend/* rag-chatbot/

# Commit and push
cd rag-chatbot
git add .
git commit -m "Update CORS for Vercel production"
git push
```

## Option 3: Sync from GitHub (Recommended for Future)

Set up automatic sync from GitHub to Hugging Face:

1. Go to your Space settings
2. Enable "Sync from GitHub"
3. Connect your GitHub repository
4. Future updates will sync automatically

## After Update

1. Wait for Hugging Face to rebuild (1-2 minutes)
2. Check logs: https://huggingface.co/spaces/abdul18/rag-chatbot/logs
3. Test health endpoint: https://abdul18-rag-chatbot.hf.space/api/health
4. Test chatbot on Vercel: https://physical-ai-textbook-wine.vercel.app

## Verification

After updating, test the chatbot:

1. Visit: https://physical-ai-textbook-wine.vercel.app
2. Click the chatbot button (💬)
3. Ask: "What is Physical AI?"
4. Should get a response from the RAG system

If you see CORS errors in browser console, the update hasn't taken effect yet. Wait a bit longer and refresh.
