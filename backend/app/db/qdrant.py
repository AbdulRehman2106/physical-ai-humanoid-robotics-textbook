from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from typing import List, Dict, Any, Optional
from app.config import get_settings
from app.models.document import SearchResult


class QdrantDB:
    """Qdrant vector database client."""

    def __init__(self):
        self.settings = get_settings()
        self.client = QdrantClient(
            url=self.settings.qdrant_url,
            api_key=self.settings.qdrant_api_key
        )
        self.collection_name = self.settings.qdrant_collection_name

    def create_collection(self, vector_size: int = 1024):
        """Create the collection if it doesn't exist."""
        try:
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except Exception:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}'")

    def upsert_chunks(self, chunks: List[Dict[str, Any]], vectors: List[List[float]]):
        """Insert or update document chunks with their embeddings."""
        points = [
            PointStruct(
                id=chunk['chunk_id'],
                vector=vector,
                payload=chunk
            )
            for chunk, vector in zip(chunks, vectors)
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[SearchResult]:
        """Search for similar chunks."""
        # Build filter if provided
        search_filter = None
        if filters:
            conditions = []
            if 'chapter' in filters:
                conditions.append(
                    FieldCondition(
                        key="chapter_number",
                        match=MatchValue(value=filters['chapter'])
                    )
                )
            if conditions:
                search_filter = Filter(must=conditions)

        # Perform search using query_points
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=limit,
            query_filter=search_filter
        ).points

        # Convert to SearchResult models
        return [
            SearchResult(
                chunk_id=result.payload['chunk_id'],
                chapter_number=result.payload['chapter_number'],
                chapter_title=result.payload['chapter_title'],
                section_title=result.payload['section_title'],
                content=result.payload['content'],
                content_type=result.payload['content_type'],
                url=result.payload['url'],
                score=result.score
            )
            for result in results
        ]

    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection."""
        return self.client.get_collection(self.collection_name)
