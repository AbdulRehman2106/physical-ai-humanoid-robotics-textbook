from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
from uuid import UUID


class ChatQuery(BaseModel):
    """Request model for chat queries."""
    query: str = Field(..., min_length=1, max_length=1000)
    conversation_id: Optional[UUID] = None
    filters: Optional[Dict[str, Any]] = None


class ChatQueryWithContext(BaseModel):
    """Request model for chat queries with selected text context."""
    query: str = Field(..., min_length=1, max_length=1000)
    selected_text: Optional[str] = None
    selection_metadata: Optional[Dict[str, Any]] = None
    conversation_id: Optional[UUID] = None
    filters: Optional[Dict[str, Any]] = None


class SourceReference(BaseModel):
    """Reference to a source document chunk."""
    chapter_number: int
    chapter_title: str
    section_title: str
    url: str
    relevance_score: float


class ChatResponse(BaseModel):
    """Response model for chat queries."""
    answer: str
    sources: List[SourceReference]
    conversation_id: UUID
    message_id: UUID


class Message(BaseModel):
    """Message model."""
    id: UUID
    conversation_id: UUID
    role: str
    content: str
    context_used: Optional[List[str]] = None
    created_at: datetime
    metadata: Optional[Dict[str, Any]] = None


class Conversation(BaseModel):
    """Conversation model."""
    id: UUID
    created_at: datetime
    metadata: Optional[Dict[str, Any]] = None
    messages: Optional[List[Message]] = None
