from typing import Optional, Dict, Any
from uuid import UUID
from app.services.retrieval import RetrievalService
from app.services.generation import GenerationService
from app.db.postgres import PostgresDB
from app.models.chat import ChatResponse, SourceReference


class RAGPipeline:
    """Main RAG pipeline orchestrating retrieval and generation."""

    def __init__(self):
        self.retrieval = RetrievalService()
        self.generation = GenerationService()
        self.db = PostgresDB()

    def process_query(
        self,
        query: str,
        conversation_id: Optional[UUID] = None,
        selected_text: Optional[str] = None,
        filters: Optional[Dict[str, Any]] = None
    ) -> ChatResponse:
        """Process a user query through the RAG pipeline."""

        # Create conversation if needed
        if not conversation_id:
            conversation_id = self.db.create_conversation()

        # Get conversation history
        conversation_history = self.db.get_conversation_history(conversation_id)

        # Retrieve relevant chunks
        retrieved_chunks = self.retrieval.retrieve(
            query=query,
            limit=5,
            filters=filters
        )

        # Generate response
        answer = self.generation.generate_response(
            query=query,
            retrieved_chunks=retrieved_chunks,
            selected_text=selected_text,
            conversation_history=conversation_history
        )

        # Store user message
        self.db.add_message(
            conversation_id=conversation_id,
            role="user",
            content=query,
            metadata={
                "selected_text": selected_text,
                "filters": filters
            }
        )

        # Store assistant message
        context_used = [chunk.chunk_id for chunk in retrieved_chunks]
        message_id = self.db.add_message(
            conversation_id=conversation_id,
            role="assistant",
            content=answer,
            context_used=context_used
        )

        # Build source references with localhost URLs for development
        sources = [
            SourceReference(
                chapter_number=chunk.chapter_number,
                chapter_title=chunk.chapter_title,
                section_title=chunk.section_title,
                url=chunk.url.replace(
                    "https://physical-ai-textbook.vercel.app",
                    "http://localhost:3000"
                ),
                relevance_score=chunk.score
            )
            for chunk in retrieved_chunks
        ]

        return ChatResponse(
            answer=answer,
            sources=sources,
            conversation_id=conversation_id,
            message_id=message_id
        )
