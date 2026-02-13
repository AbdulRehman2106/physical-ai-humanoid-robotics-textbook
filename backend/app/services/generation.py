import cohere
from typing import List
from app.config import get_settings
from app.models.document import SearchResult


class GenerationService:
    """Service for generating responses using Cohere."""

    def __init__(self):
        self.settings = get_settings()
        self.client = cohere.Client(self.settings.cohere_api_key)
        self.model = self.settings.cohere_generation_model

    def generate_response(
        self,
        query: str,
        retrieved_chunks: List[SearchResult],
        selected_text: str = None,
        conversation_history: List[dict] = None
    ) -> str:
        """Generate a response based on retrieved context."""
        # Build context from retrieved chunks
        context_parts = []
        for i, chunk in enumerate(retrieved_chunks, 1):
            context_parts.append(
                f"[Source {i}: {chunk.chapter_title} - {chunk.section_title}]\n{chunk.content}"
            )
        context = "\n\n".join(context_parts)

        # Build prompt
        system_prompt = """You are an AI teaching assistant for the Physical AI and Humanoid Robotics textbook.

CRITICAL RULES - YOU MUST FOLLOW THESE:
1. ALWAYS provide a direct, complete answer to the user's question
2. NEVER ask questions back to the user (NO "Could you clarify...", NO "What specifically...", NO "Please specify...")
3. If the question is vague, make reasonable assumptions and answer based on the most relevant information in the context
4. If the answer is not in the context, say "I don't have information about that in the textbook" - but still DON'T ask questions
5. Provide educational, clear, and concise answers
6. Use technical terms appropriately and explain them when needed
7. For code-related questions, provide relevant code snippets from the context

Remember: Your job is to ANSWER, not to ask for clarification. Always give the best answer you can based on the available context."""

        user_prompt = f"""Context from the textbook:
{context}
"""

        if selected_text:
            user_prompt += f"\nUser selected this text: \"{selected_text}\"\n"

        user_prompt += f"\nQuestion: {query}\n\nAnswer based on the context above:"

        # Build chat history
        chat_history = []
        if conversation_history:
            for msg in conversation_history[-5:]:  # Last 5 messages for context
                # Map roles to Cohere's expected format
                role_mapping = {
                    "user": "User",
                    "assistant": "Chatbot"
                }
                cohere_role = role_mapping.get(msg['role'], "User")
                chat_history.append({
                    "role": cohere_role,
                    "message": msg['content']
                })

        # Generate response
        response = self.client.chat(
            model=self.model,
            message=user_prompt,
            chat_history=chat_history,
            preamble=system_prompt,
            temperature=0.3,
            max_tokens=1000
        )

        return response.text
