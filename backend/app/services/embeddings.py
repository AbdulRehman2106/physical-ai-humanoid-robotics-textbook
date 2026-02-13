import cohere
from typing import List
from app.config import get_settings


class EmbeddingService:
    """Service for generating embeddings using Cohere."""

    def __init__(self):
        self.settings = get_settings()
        self.client = cohere.Client(self.settings.cohere_api_key)
        self.model = self.settings.cohere_embed_model

    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type="search_document"
        )
        return response.embeddings[0]

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type="search_document"
        )
        return response.embeddings

    def embed_query(self, query: str) -> List[float]:
        """Generate embedding for a search query."""
        response = self.client.embed(
            texts=[query],
            model=self.model,
            input_type="search_query"
        )
        return response.embeddings[0]
