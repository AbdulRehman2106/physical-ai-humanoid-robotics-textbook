from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere API
    cohere_api_key: str
    cohere_embed_model: str = "embed-english-v3.0"
    cohere_generation_model: str = "command-r-08-2024"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "physical_ai_textbook"

    # Neon Postgres
    neon_database_url: str

    # Frontend
    frontend_url: str = "http://localhost:3000"

    # Application
    environment: str = "development"

    class Config:
        env_file = ".env"
        case_sensitive = False


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
