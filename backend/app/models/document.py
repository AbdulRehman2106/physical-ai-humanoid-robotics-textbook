from pydantic import BaseModel
from typing import Optional, List


class DocumentChunk(BaseModel):
    """Model for a document chunk to be indexed."""
    chunk_id: str
    chapter_number: int
    chapter_title: str
    section_title: str
    content: str
    content_type: str  # text, code, callout, quiz
    url: str
    keywords: Optional[List[str]] = None
    word_count: int


class SearchResult(BaseModel):
    """Model for a search result from Qdrant."""
    chunk_id: str
    chapter_number: int
    chapter_title: str
    section_title: str
    content: str
    content_type: str
    url: str
    score: float
