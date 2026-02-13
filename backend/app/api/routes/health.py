from fastapi import APIRouter, HTTPException
from app.db.qdrant import QdrantDB
from app.db.postgres import PostgresDB

router = APIRouter()


@router.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "service": "Physical AI RAG Backend"
    }


@router.get("/health/detailed")
async def detailed_health_check():
    """Detailed health check including database connections."""
    health_status = {
        "status": "healthy",
        "service": "Physical AI RAG Backend",
        "components": {}
    }

    # Check Qdrant
    try:
        qdrant = QdrantDB()
        collection_info = qdrant.get_collection_info()
        health_status["components"]["qdrant"] = {
            "status": "healthy",
            "collection": collection_info.dict()
        }
    except Exception as e:
        health_status["status"] = "degraded"
        health_status["components"]["qdrant"] = {
            "status": "unhealthy",
            "error": str(e)
        }

    # Check Postgres
    try:
        db = PostgresDB()
        with db.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT 1")
        health_status["components"]["postgres"] = {
            "status": "healthy"
        }
    except Exception as e:
        health_status["status"] = "degraded"
        health_status["components"]["postgres"] = {
            "status": "unhealthy",
            "error": str(e)
        }

    if health_status["status"] != "healthy":
        raise HTTPException(status_code=503, detail=health_status)

    return health_status
