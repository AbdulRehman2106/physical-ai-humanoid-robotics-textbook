from fastapi import APIRouter, HTTPException
from uuid import UUID
import logging
import traceback
from app.models.chat import (
    ChatQuery,
    ChatQueryWithContext,
    ChatResponse,
    Conversation
)
from app.services.rag_pipeline import RAGPipeline
from app.db.postgres import PostgresDB

router = APIRouter()
rag_pipeline = RAGPipeline()
db = PostgresDB()
logger = logging.getLogger(__name__)


@router.post("/query", response_model=ChatResponse)
async def query_chat(request: ChatQuery):
    """Process a chat query."""
    try:
        response = rag_pipeline.process_query(
            query=request.query,
            conversation_id=request.conversation_id,
            filters=request.filters
        )
        return response
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        logger.error(traceback.format_exc())
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.post("/query-with-context", response_model=ChatResponse)
async def query_with_context(request: ChatQueryWithContext):
    """Process a chat query with selected text context."""
    try:
        response = rag_pipeline.process_query(
            query=request.query,
            conversation_id=request.conversation_id,
            selected_text=request.selected_text,
            filters=request.filters
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.post("/conversations", response_model=dict)
async def create_conversation():
    """Create a new conversation."""
    try:
        conversation_id = db.create_conversation()
        return {"conversation_id": conversation_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating conversation: {str(e)}")


@router.get("/conversations/{conversation_id}", response_model=Conversation)
async def get_conversation(conversation_id: UUID):
    """Get a conversation with all its messages."""
    try:
        conversation = db.get_conversation(conversation_id)
        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")
        return conversation
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving conversation: {str(e)}")
