import psycopg2
from psycopg2.extras import RealDictCursor
from contextlib import contextmanager
from typing import Optional, List, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime
from app.config import get_settings


class PostgresDB:
    """PostgreSQL database client for conversation storage."""

    def __init__(self):
        self.settings = get_settings()
        self.connection_string = self.settings.neon_database_url

    @contextmanager
    def get_connection(self):
        """Context manager for database connections."""
        conn = psycopg2.connect(self.connection_string)
        try:
            yield conn
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    def create_conversation(self, metadata: Optional[Dict[str, Any]] = None) -> UUID:
        """Create a new conversation."""
        conversation_id = uuid4()
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO conversations (id, metadata)
                    VALUES (%s, %s)
                    RETURNING id
                    """,
                    (str(conversation_id), psycopg2.extras.Json(metadata or {}))
                )
                result = cur.fetchone()[0]
                return UUID(result) if isinstance(result, str) else result

    def add_message(
        self,
        conversation_id: UUID,
        role: str,
        content: str,
        context_used: Optional[List[str]] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> UUID:
        """Add a message to a conversation."""
        message_id = uuid4()
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO messages (id, conversation_id, role, content, context_used, metadata)
                    VALUES (%s, %s, %s, %s, %s, %s)
                    RETURNING id
                    """,
                    (
                        str(message_id),
                        str(conversation_id),
                        role,
                        content,
                        context_used or [],
                        psycopg2.extras.Json(metadata or {})
                    )
                )
                result = cur.fetchone()[0]
                return UUID(result) if isinstance(result, str) else result

    def get_conversation(self, conversation_id: UUID) -> Optional[Dict[str, Any]]:
        """Get a conversation with all its messages."""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                # Get conversation
                cur.execute(
                    "SELECT * FROM conversations WHERE id = %s",
                    (str(conversation_id),)
                )
                conversation = cur.fetchone()

                if not conversation:
                    return None

                # Get messages
                cur.execute(
                    """
                    SELECT * FROM messages
                    WHERE conversation_id = %s
                    ORDER BY created_at ASC
                    """,
                    (str(conversation_id),)
                )
                messages = cur.fetchall()

                return {
                    **dict(conversation),
                    'messages': [dict(msg) for msg in messages]
                }

    def get_conversation_history(
        self,
        conversation_id: UUID,
        limit: int = 10
    ) -> List[Dict[str, Any]]:
        """Get recent messages from a conversation."""
        with self.get_connection() as conn:
            with conn.cursor(cursor_factory=RealDictCursor) as cur:
                cur.execute(
                    """
                    SELECT * FROM messages
                    WHERE conversation_id = %s
                    ORDER BY created_at DESC
                    LIMIT %s
                    """,
                    (str(conversation_id), limit)
                )
                messages = cur.fetchall()
                return [dict(msg) for msg in reversed(messages)]
