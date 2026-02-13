from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import get_settings
from app.api.routes import chat, health

# Initialize settings
settings = get_settings()

# Create FastAPI app
app = FastAPI(
    title="Physical AI RAG Backend",
    description="RAG-powered chatbot backend for Physical AI textbook",
    version="1.0.0"
)

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

# Include routers
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(chat.router, prefix="/api/chat", tags=["chat"])


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI RAG Backend",
        "version": "1.0.0",
        "docs": "/docs"
    }
