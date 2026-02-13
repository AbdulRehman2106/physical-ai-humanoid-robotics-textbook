from typing import List, Optional, Dict, Any
from app.db.qdrant import QdrantDB
from app.services.embeddings import EmbeddingService
from app.models.document import SearchResult


class RetrievalService:
    """Service for retrieving relevant document chunks."""

    def __init__(self):
        self.qdrant = QdrantDB()
        self.embeddings = EmbeddingService()

    def retrieve(
        self,
        query: str,
        limit: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[SearchResult]:
        """Retrieve relevant chunks for a query."""
        # Generate query embedding
        query_vector = self.embeddings.embed_query(query)

        # Search in Qdrant
        results = self.qdrant.search(
            query_vector=query_vector,
            limit=limit,
            filters=filters
        )

        return results
